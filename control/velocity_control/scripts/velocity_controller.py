#!/usr/bin/python3

import numpy as np

import rospy
import std_msgs.msg

import velocity_control_helpers.velocity_reference_model as velocity_reference_model
import velocity_control_helpers.attitude_reference_model as attitude_reference_model
import velocity_control_helpers.utilities as utilities

from geometry_msgs.msg import TwistStamped
from olympe_bridge.msg import AttitudeCommand
from anafi_uav_msgs.msg import EkfOutput
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
    self.velocity_reference_model = velocity_reference_model.VelocityReferenceModel(
      omegas=velocity_reference_omegas,
      zetas=velocity_reference_zetas
    )

    # Setup services
    # services = rospy.get_param("services")
    rospy.Service("/velocity_controller/service/enable_controller", SetBool, self.__enable_controller)

    # Setup subscribers 
    # topics = rospy.get_param("topics")
    rospy.Subscriber("/anafi/twist_body" , TwistStamped, self.__twist_cb)
    rospy.Subscriber("/estimate/ekf", EkfOutput, self.__ekf_cb)
    rospy.Subscriber("/guidance/velocity_reference", TwistStamped, self.__reference_velocities_cb)

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)

    # Initial values
    self.reference_velocities : np.ndarray = np.zeros((3, 1))
    self.velocities_body : np.ndarray = np.zeros((3, 1))
    self.velocities_relative_to_helipad : np.ndarray = np.zeros((3, 1))

    self.ekf_timestamp : std_msgs.msg.Time = None
    self.twist_timestamp : std_msgs.msg.Time = None
    self.guidance_timestamp : std_msgs.msg.Time = None

    self.is_controller_active : bool = False


  def __reference_velocities_cb(self, msg : TwistStamped):
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.guidance_timestamp, msg_timestamp):
      # Old message
      return

    self.guidance_timestamp = msg_timestamp
    self.reference_velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def __enable_controller(self, msg : SetBool):
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def __ekf_cb(self, msg : EkfOutput) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.velocities_relative_to_helipad = np.array([msg.u_r, msg.v_r, msg.w_r]).T


  def __twist_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.twist_timestamp, msg_timestamp):
      # Old message
      return
    
    self.twist_timestamp = msg_timestamp
    self.velocities_body = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T # Body-velocities are not used atm


  def publish_attitude_ref(self) -> None:
    attitude_cmd_msg = AttitudeCommand()

    v_d = np.zeros((4, 1))
    while not rospy.is_shutdown():
      if self.is_controller_active:
        stamp = rospy.Time.now()

        v_d = self.velocity_reference_model.get_filtered_reference(
          xd_prev=v_d, 
          v_ref_raw=self.reference_velocities[:2],
          dt=self.dt
        )
        v_d[2] = self.reference_velocities[2]

        att_ref = self.attitude_reference_model.get_attitude_reference(
          v_ref=v_d[:2],
          v=self.velocities_relative_to_helipad[:2],
          ts=stamp
        )

        att_ref_3D = np.array([att_ref[0], att_ref[1], 0, v_d[2]], dtype=np.float64).T 
        attitude_cmd_msg.roll = rospy.Time.now()
        attitude_cmd_msg.roll = np.rad2deg(att_ref_3D[0])   # Message requires rpy in deg
        attitude_cmd_msg.pitch = np.rad2deg(att_ref_3D[1])
        attitude_cmd_msg.yaw = np.rad2deg(att_ref_3D[2])
        attitude_cmd_msg.gaz = att_ref_3D[3]

        self.attitude_ref_pub.publish(attitude_cmd_msg)

      else:
        self.reference_velocities = np.zeros((3, 1))
        v_d = np.zeros((4, 1))
      
      self.rate.sleep()


def main():
  node = VelocityController()
  node.publish_attitude_ref()


if __name__ == "__main__":
  main()
