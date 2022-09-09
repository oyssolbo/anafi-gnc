#!/usr/bin/env python3

from distutils.command.config import config
import rospy 
import anafi_uav_msgs
import std_msgs

import numpy as np

import utilities

"""
How to improve this file:
  - use it as a proper ROS-node:
      - take in the desired position
      - take in the current state
      - output a reference-velocity based on the current  
  - use proper method for reading the parameters / config
"""


class GuidanceLaw():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = "guidance"
    config_file = utilities.load_config_file(node_name)

    controller_rate = config_file["rate_Hz"]
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    rospy.Rate(controller_rate)

    # Set up subscribers 
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.EkfOutput, self.__ekf_cb)
    # rospy.Subscriber("/estimate/target_velocity", anafi_uav_msgs.msg.) # Would be nice to predict target movement

    # Set up publishers
    self.reference_velocity_publisher = rospy.Publisher(
      "/attitude_controller/reference_states", anafi_uav_msgs.msg.ReferenceStates, queue_size=1)

    # Set up services
    rospy.Service("/guidance/desired_pos", anafi_uav_msgs.srv.SetDesiredPose, self.__set_pos)

    # Initialize parameters
    params = config_file["pp"]
    limits = config_file["velocity_limits"]
    
    self.ua_max = params["ua_max"]
    self.lookahead = params["lookahead"]
    # self.kappa = params["kappa"]

    self.vx_limits = limits["vx"]
    self.vy_limits = limits["vy"]
    self.vz_limits = limits["vz"]

    self.ekf_timestamp : std_msgs.msg.Time = None
    self.pose_timestamp : std_msgs.msg.Time = None

    self.desired_pos : np.ndarray = np.zeros((3, 1))  # [xd, yd, zd]
    self.pos : np.ndarray = np.zeros((3, 1))          # [x, y, z]


  def __set_pos(self, msg : anafi_uav_msgs.srv.SetDesiredPose):
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.pose_timestamp, msg_timestamp):
      # Old message
      return False

    self.pose_timestamp = msg_timestamp
    self.desired_pos = np.array([msg.x, msg.y, msg.z]).T
    return True


  def __ekf_cb(self, msg : anafi_uav_msgs.msg.EkfOutput) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos = np.array([msg.x, msg.y, msg.z]).T


  def __clamp(
        self, 
        value: float, 
        limits: tuple
      ) -> float:
    if value < limits[0]:
      return limits[0]
    elif value > limits[1]:
      return limits[1]
    else:
      return value


  def calculate_velocity_reference(self) -> None:
    """
    Generate a velocity reference from a position error using the pure
    pursuit guidance law as defined in Fossen 2021.
    """
    reference_msg = anafi_uav_msgs.msg.ReferenceStates()
    vel_target = np.zeros((3, 1)) # Extend the guidance-law to use constant bearing
  
    while not rospy.is_shutdown():
      pos_error = self.pos - self.desired_pos
      pos_error_normed = np.linalg.norm(pos_error)

      kappa = (pos_error_normed @ self.ua_max) / (np.sqrt(pos_error_normed + self.lookahead**2))

      vel_ref_unclamped = vel_target - (kappa @ pos_error) / (pos_error_normed) 

      vel_ref_x = self.__clamp(vel_ref_unclamped[0], self.vx_limits)
      vel_ref_y = self.__clamp(vel_ref_unclamped[1], self.vy_limits)
      vel_ref_z = self.__clamp(vel_ref_unclamped[2], self.vz_limits)

      reference_msg.u_ref = vel_ref_x
      reference_msg.v_ref = vel_ref_y
      reference_msg.w_ref = vel_ref_z

      self.reference_velocity_publisher.publish(reference_msg)
      rospy.Rate.sleep()


def main():
  guidance_law = GuidanceLaw()
  guidance_law.calculate_velocity_reference()


if __name__ == "__main__":
  main()
