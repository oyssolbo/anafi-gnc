#!/usr/bin/python3

import rospy 
import std_msgs

from anafi_uav_msgs.msg import EkfOutput, ReferenceStates
from anafi_uav_msgs.srv import SetDesiredPose, SetDesiredPoseResponse

import numpy as np

import guidance_helpers.utilities as utilities

class PurePursuitGuidanceLaw():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 

  TODO:
    - add guidance for multiple frames
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "pure_pursuit_guidance_node")
    controller_rate = rospy.get_param("~rate_Hz", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up subscribers 
    rospy.Subscriber("/estimate/ekf", EkfOutput, self.__ekf_cb)
    # rospy.Subscriber("/estimate/target_velocity", anafi_uav_msgs.msg.) # Would be nice to predict target movement

    # Set up publishers
    self.reference_velocity_publisher = rospy.Publisher(
      "/guidance/velocity_reference", ReferenceStates, queue_size=1)

    # Set up services
    rospy.Service("/guidance/service/desired_pos", SetDesiredPose, self.__set_pos)

    # Initialize parameters
    pure_pursuit_params = rospy.get_param("~pure_pursuit_parameters")
    velocity_limits = rospy.get_param("~velocity_limits")
    
    self.ua_max = pure_pursuit_params["ua_max"]
    self.lookahead = pure_pursuit_params["lookahead"]
    # self.kappa = pure_pursuit_params["kappa"]

    self.vx_limits = velocity_limits["vx"]
    self.vy_limits = velocity_limits["vy"]
    self.vz_limits = velocity_limits["vz"]

    self.ekf_timestamp : std_msgs.msg.Time = None
    # self.pose_timestamp : std_msgs.msg.Time = None

    self.desired_pos : np.ndarray = np.zeros((3, 1))  # [xd, yd, zd]
    self.pos : np.ndarray = np.zeros((3, 1))          # [x, y, z]

    self.pos_relative_to_helipad : np.ndarray = None 


  def __set_pos(self, msg : SetDesiredPose):
    frame = msg.coordinate_frame # May want to add guidance with respect to multiple frames TODO
    self.desired_pos = np.array([msg.x, msg.y, msg.z]).T

    res = SetDesiredPoseResponse()
    res.success = True
    return res


  def __ekf_cb(self, msg : EkfOutput) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos_relative_to_helipad = np.array([msg.x, msg.y, msg.z]).T


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


  def __get_valid_pos_error(self, prev_pos_error_normed : float) -> np.ndarray:
    """
    Returns a valid error for position
    """
    max_error = rospy.get_param("~max_error_normed", default=10)

    if (self.ekf_timestamp is None\
      or prev_pos_error_normed is None\
      or prev_pos_error_normed > max_error
    ):
      return np.zeros((3, 1))

    return self.pos_relative_to_helipad    


  def calculate_velocity_reference(self) -> None:
    """
    Generate a velocity reference from a position error using the pure
    pursuit guidance law as defined in Fossen 2021.
    """
    reference_msg = ReferenceStates()
    
    vel_target = np.zeros((3, 1)) # Possible extension to use constant bearing guidance in the future
    prev_pos_error_normed = None

    while not rospy.is_shutdown():

      pos_error = self.__get_valid_pos_error(prev_pos_error_normed=prev_pos_error_normed)
      pos_error_normed = np.linalg.norm(pos_error)
      prev_pos_error_normed = pos_error_normed

      if pos_error_normed > 1e-3:
        kappa = (pos_error_normed * self.ua_max) / (np.sqrt(pos_error_normed + self.lookahead**2))
        vel_ref_unclamped = vel_target - (kappa @ pos_error) / (pos_error_normed) 
      else:
        vel_ref_unclamped = np.zeros((3, 1))

      vel_ref_x = self.__clamp(vel_ref_unclamped[0], self.vx_limits)
      vel_ref_y = self.__clamp(vel_ref_unclamped[1], self.vy_limits)
      vel_ref_z = self.__clamp(vel_ref_unclamped[2], self.vz_limits)

      reference_msg.u_ref = vel_ref_x
      reference_msg.v_ref = vel_ref_y
      reference_msg.w_ref = vel_ref_z

      self.reference_velocity_publisher.publish(reference_msg)
      self.rate.sleep()


def main():
  guidance_law = PurePursuitGuidanceLaw()
  guidance_law.calculate_velocity_reference()


if __name__ == "__main__":
  main()
