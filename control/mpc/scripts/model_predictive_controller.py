#!/usr/bin/python3

import numpy as np
import casadi

import rospy
import std_msgs.msg

from geometry_msgs.msg import TwistStamped
from olympe_bridge.msg import AttitudeCommand
from std_srvs.srv import SetBool, SetBoolResponse

import mpc_helpers.utilities as utilities

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

class ModelPredictiveController():

  def __init__(self) -> None:

    # Initializing node
    node_name = rospy.get_param("~node_name", default = "mpc_node")
    node_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / node_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup services
    rospy.Service("/mpc/service/enable_controller", SetBool, self.__enable_controller)

    # Setup subscribers 
    # Attitude estimate from the anafi
    # Optical flow feedback from the bridge
    # Position estimate from the EKF

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)

    # Initial values
    self.guidance_reference_velocities : np.ndarray = np.zeros((3, 1))
    self.velocities_body : np.ndarray = np.zeros((3, 1))

    self.velocities_body_timestamp : std_msgs.msg.Time = None
    self.guidance_timestamp : std_msgs.msg.Time = None

    self.is_controller_active : bool = False


  def __reference_velocities_cb(self, msg : TwistStamped):
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.guidance_timestamp, msg_timestamp):
      # Old message
      return

    self.guidance_timestamp = msg_timestamp
    self.guidance_reference_velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def __enable_controller(self, msg : SetBool):
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def __optical_flow_velocities_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.velocities_body_timestamp, msg_timestamp):
      # Old message
      return
    
    self.velocities_body_timestamp = msg_timestamp
    self.velocities_body = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def spin(self) -> None:
    while not rospy.is_shutdown():
      self.rate.sleep()


def main():
  node = ModelPredictiveController()
  node.spin()


if __name__ == "__main__":
  main()
