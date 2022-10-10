#!/usr/bin/python3

import numpy as np

import rospy
import std_msgs.msg

import velocity_control_helpers.velocity_reference_model as velocity_reference_model
import velocity_control_helpers.attitude_reference_model as attitude_reference_model
import velocity_control_helpers.utilities as utilities

from geometry_msgs.msg import TwistStamped, Vector3Stamped
from olympe_bridge.msg import AttitudeCommand
from std_srvs.srv import SetBool, SetBoolResponse

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

class VelocityController():

  def __init__(self) -> None:

    # Initializing node
    node_name = rospy.get_param("~node_name", default = "attitude_controller_node")
    node_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / node_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Initializing reference models
    pid_controller_parameters = rospy.get_param("~pid")
    attitude_limits = rospy.get_param("~attitude_limits")
    self.attitude_reference_model = attitude_reference_model.PIDReferenceGenerator(
      params=pid_controller_parameters,
      limits=attitude_limits 
    )

    velocity_reference_model_parameters = rospy.get_param("~velocity_reference_model")
    velocity_reference_omegas = velocity_reference_model_parameters["omegas"]
    velocity_reference_zetas = velocity_reference_model_parameters["zetas"]
    velocity_reference_T_z = velocity_reference_model_parameters["T_z"]
    velocity_reference_K_z = velocity_reference_model_parameters["K_z"]
    self.velocity_reference_model = velocity_reference_model.VelocityReferenceModel(
      omegas=velocity_reference_omegas,
      zetas=velocity_reference_zetas,
      T_z=velocity_reference_T_z,
      K_z=velocity_reference_K_z
    )

    # Setup services
    rospy.Service("/velocity_controller/service/enable_controller", SetBool, self._enable_controller)

    # Setup subscribers 
    self.use_optical_flow_velocities : bool = rospy.get_param("~use_optical_flow_velocities", default = 1)
    if self.use_optical_flow_velocities:
      rospy.loginfo("Node using optical flow velocity estimates as feedback")
      rospy.Subscriber("/anafi/optical_flow_velocities", Vector3Stamped, self._optical_flow_velocities_cb)
    else:
      rospy.loginfo("Node using polled velocity estimates as feedback")
      rospy.Subscriber("/anafi/polled_velocities", TwistStamped, self.__polled_velocities_cb)

    use_pure_pursuit_guidance : bool = rospy.get_param("~use_pure_pursuit_guidance", default = 0)
    if use_pure_pursuit_guidance:
      rospy.loginfo("Node using pure pursuit guidance used to generate velocity references")
      rospy.Subscriber("/guidance/pure_pursuit/velocity_reference", TwistStamped, self._reference_velocities_cb)
    else:
      rospy.loginfo("Node using PID-based guidance used to generate velocity references")
      rospy.Subscriber("/guidance/pid/velocity_reference", TwistStamped, self._reference_velocities_cb)

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)

    # Initial values
    self.guidance_reference_velocities : np.ndarray = np.zeros((3, 1))
    self.polled_velocities : np.ndarray = np.zeros((3, 1))
    self.optical_flow_velocities : np.ndarray = np.zeros((3, 1))

    self.polled_velocities_timestamp : std_msgs.msg.Time = None
    self.optical_flow_velocities_timestamp : std_msgs.msg.Time = None
    self.guidance_timestamp : std_msgs.msg.Time = None

    self.is_controller_active : bool = False


  def _reference_velocities_cb(self, msg : TwistStamped):
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.guidance_timestamp, msg_timestamp):
      # Old message
      return

    self.guidance_timestamp = msg_timestamp
    self.guidance_reference_velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def _enable_controller(self, msg : SetBool):
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def _polled_velocities_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.polled_velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.polled_velocities_timestamp = msg_timestamp     
    self.polled_velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def _optical_flow_velocities_cb(self, msg : Vector3Stamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.optical_flow_velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.optical_flow_velocities_timestamp = msg_timestamp     
    
    # Important: 
    # The signs and order of elements are different for the optical flow compared to the 
    # polled-velocities. This is due to having the frame defined in the images, and not in body
    self.optical_flow_velocities = np.array([-msg.vector.y, -msg.vector.x, -msg.vector.z]).T 


  def publish_attitude_ref(self) -> None:
    attitude_cmd_msg = AttitudeCommand()

    x_d = np.zeros((5, 1))
    # self.is_controller_active = True
    while not rospy.is_shutdown():
      if self.is_controller_active:
        x_d = self.velocity_reference_model.get_filtered_reference(
          xd_prev=x_d, 
          v_ref_raw=self.guidance_reference_velocities,
          dt=self.dt
        )
        
        if self.use_optical_flow_velocities:
          v = self.optical_flow_velocities
          ts = self.optical_flow_velocities_timestamp
        else:
          v = self.polled_velocities
          ts = self.polled_velocities_timestamp

        att_ref = self.attitude_reference_model.get_attitude_reference(
          v_ref=(x_d[:2]).reshape((2, 1)),
          v=(v[:2]).reshape((2, 1)), 
          ts=ts,
          debug=False
        )

        att_ref_3D = np.array([att_ref[0], -att_ref[1], 0, x_d[4]], dtype=np.float64) 
        attitude_cmd_msg.header.stamp = rospy.Time.now()
        attitude_cmd_msg.roll = att_ref_3D[0]   
        attitude_cmd_msg.pitch = att_ref_3D[1]
        attitude_cmd_msg.yaw = att_ref_3D[2]
        attitude_cmd_msg.gaz = att_ref_3D[3]

        self.attitude_ref_pub.publish(attitude_cmd_msg)

      else:
        self.guidance_reference_velocities = np.zeros((3, 1))
        x_d = np.zeros((5, 1))
      
      self.rate.sleep()


def main():
  node = VelocityController()
  node.publish_attitude_ref()


if __name__ == "__main__":
  main()
