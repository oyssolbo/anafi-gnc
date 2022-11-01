#!/usr/bin/python3

import numpy as np
import casadi
from scipy.spatial.transform import Rotation

import rospy
import std_msgs.msg

from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped, PointStamped
from olympe_bridge.msg import AttitudeCommand
from std_srvs.srv import SetBool, SetBoolResponse
from anafi_uav_msgs.msg import PointWithCovarianceStamped

import mpc_helpers.utilities as utilities
import mpc_helpers.mpc as mpc

class ModelPredictiveController():
  def __init__(self) -> None:

    # Initializing node
    node_name = rospy.get_param("~node_name", default = "mpc_node")
    node_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / node_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setting up the controller
    self.mpc_parameters = rospy.get_param("~mpc_config")
    self.mpc_solver = mpc.MPCSolver(mpc_parameters=self.mpc_parameters).get_mpc_solver()

    # Initial values
    self.nx = self.mpc_parameters["tuning_parameters"]["nx"]
    self.nu = self.mpc_parameters["tuning_parameters"]["nu"]
    self.m = self.mpc_parameters["tuning_parameters"]["m"]

    self.velocities : np.ndarray = np.zeros((3, 1))
    self.position : np.ndarray = np.zeros((3, 1))
    self.attitude_rpy : np.ndarray = np.zeros((3, 1))

    self.velocities_timestamp : std_msgs.msg.Time = None
    self.attitude_timestamp : std_msgs.msg.Time = None 
    self.position_timestamp : std_msgs.msg.Time = None
    
    self.last_rotation_matrix_body_to_vehicle : np.ndarray = None

    self.is_controller_active : bool = False

    # Setup services
    rospy.Service("/mpc/service/enable_controller", SetBool, self._enable_controller)

    # Setup subscribers 
    self.use_optical_flow_velocities : bool = rospy.get_param("~use_optical_flow_velocities", default = False)
    if self.use_optical_flow_velocities:
      rospy.loginfo("Node using optical flow velocity estimates as feedback")
      rospy.Subscriber("/anafi/optical_flow_velocities", Vector3Stamped, self._optical_flow_velocities_cb)
    else:
      rospy.loginfo("Node using polled velocity estimates as feedback")
      rospy.Subscriber("/anafi/polled_body_velocities", TwistStamped, self._polled_velocities_cb)

    self.use_ned_pos_from_gnss : bool = rospy.get_param("/use_ned_pos_from_gnss")
    if self.use_ned_pos_from_gnss:
      rospy.loginfo("Node using position estimates from GNSS. Estimates from EKF disabled")
      rospy.Subscriber("/anafi/ned_pos_from_gnss", PointStamped, self._ned_pos_cb)
    else:
      rospy.loginfo("Node using position estimates from EKF. Position estimates from GNSS disabled")
      rospy.Subscriber("/estimate/ekf", PointWithCovarianceStamped, self._ekf_cb)

    rospy.Subscriber("/anafi/attitude", QuaternionStamped, self._attitude_cb)

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)

    # Initializing the solver with zero-input
    rospy.logdebug("Initializing MPC")
    self.mpc_solver.make_step(np.zeros((self.nx, self.m)))
    rospy.loginfo("MPC initialized")


  def _enable_controller(self, msg : SetBool):
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def _optical_flow_velocities_cb(self, msg : Vector3Stamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.velocities_timestamp = msg_timestamp
    self.velocities = np.array([msg.vector.x, msg.vector.y, msg.vector.z]).reshape((3, 1))


  def _polled_velocities_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.velocities_timestamp = msg_timestamp     
    self.velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).reshape((3, 1))


  def _ekf_cb(self, msg : PointWithCovarianceStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.position_timestamp, msg_timestamp):
      # Old message
      return
    
    self.position_timestamp = msg_timestamp
    self.position = -np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float).reshape((3, 1)) 


  def _ned_pos_cb(self, msg : PointStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.position_timestamp, msg_timestamp):
      # Old message
      return

    if self.last_rotation_matrix_body_to_vehicle is None:
      # Impossible to convert positions to body frame
      return
    
    # The MPC-solver has the equations for position implemented in NED, but due to the assumption 
    # that yaw could be neglected, the positions are technically given in body. This means that one must know
    # the yaw angle to convert correctly
    # Luckily in the simulator, this is known exactly
    self.position_timestamp = msg_timestamp
    self.position = self.last_rotation_matrix_body_to_vehicle.T @ np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float).reshape((3, 1)) 


  def _attitude_cb(self, msg : QuaternionStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.attitude_timestamp, msg_timestamp):
      # Old message
      return
    
    self.attitude_timestamp = msg_timestamp
    rotation = Rotation.from_quat([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    self.attitude_rpy = rotation.as_euler('xyz', degrees=False).reshape((3, 1))
    self.last_rotation_matrix_body_to_vehicle = rotation.as_matrix()

  
  def _get_current_state_estimate(self) -> np.ndarray:
    if  self.position_timestamp is None or self.velocities_timestamp is None or self.attitude_timestamp is None:
      return np.zeros((self.nx, self.m))
    
    return np.vstack(
      [
        self.position,
        self.attitude_rpy
      ]
    )

  def _prevent_underflow(
        self, 
        arr       : np.ndarray, 
        min_value : float       = 1e-6
      ) -> np.ndarray:
    """
    Preventing underflow to setting all values with an absolute value below
    @p min_value to 0. It assumes that 
    """
    with np.nditer(arr, op_flags=['readwrite']) as iterator:
      for val in iterator:
        if np.abs(val) < min_value:
          val[...] = 0
    return arr


  def spin(self) -> None:
    # self.is_controller_active = True
    while not rospy.is_shutdown():
      if self.is_controller_active:
        x = self._get_current_state_estimate()
        u = self.mpc_solver.make_step(x)
        u = self._prevent_underflow(arr=u, min_value=1e-6)

        attitude_cmd_msg = AttitudeCommand()
        attitude_cmd_msg.header.stamp = rospy.Time.now()
        attitude_cmd_msg.roll = u[0]   
        attitude_cmd_msg.pitch = u[1]
        attitude_cmd_msg.yaw = u[2]
        attitude_cmd_msg.gaz = u[3]

        self.attitude_ref_pub.publish(attitude_cmd_msg)

      self.rate.sleep()


def main():
  node = ModelPredictiveController()
  node.spin()


if __name__ == "__main__":
  main()
