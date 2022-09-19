#!/usr/bin/python3

import rospy 
import std_msgs.msg
import sensor_msgs.msg
import pyproj
from enum import Enum

from geometry_msgs.msg import TwistStamped

from anafi_uav_msgs.msg import EkfOutput, ReferenceStates
from anafi_uav_msgs.srv import SetDesiredPose, SetDesiredPoseRequest, SetDesiredPoseResponse

import numpy as np
import guidance_helpers.utilities as utilities

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

class GuidanceState(Enum):
  ECEF = 0
  RELATIVE_TO_HELIPAD = 1
  IDLE = 2


class PurePursuitGuidanceLaw():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "pure_pursuit_guidance_node")
    controller_rate = rospy.get_param("~rate_Hz", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up subscribers 
    rospy.Subscriber("/estimate/ekf", EkfOutput, self.__ekf_cb)
    rospy.Subscriber("/anafi/gnss_location", sensor_msgs.msg.NavSatFix, self.__drone_gnss_cb)
    # rospy.Subscriber("/platform/out/gps", sensor_msgs.msg.NavSatFix, self.__target_gnss_cb) # Could be nice to subscribe to the estimated GNSS data about the target
    # rospy.Subscriber("/estimate/target_velocity", anafi_uav_msgs.msg.) # Would be nice to predict target movement

    # Set up publishers
    self.reference_velocity_publisher = rospy.Publisher("/guidance/velocity_reference", TwistStamped, queue_size=1)

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
    self.gnss_timestamp : std_msgs.msg.Time = None

    self.desired_ecef_pos : np.ndarray = None  # [xd, yd, zd]
    self.ecef_pos : np.ndarray = None          # [x, y, z]

    self.pos_relative_to_helipad : np.ndarray = None 

    self.guidance_state : GuidanceState = GuidanceState.RELATIVE_TO_HELIPAD


  def __set_pos(self, msg : SetDesiredPoseRequest) -> SetDesiredPoseResponse:
    self.desired_ecef_pos = np.array([msg.x_d, msg.y_d, msg.z_d]).T

    res = SetDesiredPoseResponse()
    res.success = True
    return res


  def __drone_gnss_cb(self, msg : sensor_msgs.msg.NavSatFix) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.gnss_timestamp, msg_timestamp):
      # Old message
      return

    gnss_status = msg.status
    if gnss_status == -1:
      # No position fix
      return 

    latitude_rad = msg.latitude * np.pi / 180.0
    longitude_rad = msg.longitude * np.pi / 180.0
    altitude_m = msg.altitude 

    # Convertion from long / lat / altitude to ECEF
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, longitude_rad, latitude_rad, altitude_m, radians=True)
    
    self.gnss_timestamp = msg_timestamp
    self.ecef_pos = np.array([x, y, z]).T


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


  def __request_target_ecef_position(self):
    pass


  def __get_valid_pos_error(self) -> np.ndarray:
    """
    Returns a valid error for position
    """
    zeros = np.zeros((3, 1))
    if self.guidance_state == GuidanceState.ECEF:
      if (self.gnss_timestamp is None\
        or self.ecef_pos is None\
        or self.desired_ecef_pos is None
      ):
        return zeros

      return self.ecef_pos - self.desired_ecef_pos

    if self.guidance_state == GuidanceState.RELATIVE_TO_HELIPAD:
      if (self.ekf_timestamp is None):
        return np.zeros((3, 1))

      return self.pos_relative_to_helipad    

    return zeros


  def calculate_velocity_reference(self) -> None:
    """
    Generate a velocity reference from a position error using the pure
    pursuit guidance law as defined in Fossen 2021.
    """
    twist_ref_msg = TwistStamped()

    zeros_3_1 = np.zeros((3, 1))
    vel_target = zeros_3_1 # Possible extension to use constant bearing guidance in the future

    while not rospy.is_shutdown():

      pos_error = self.__get_valid_pos_error()
      pos_error_normed = np.linalg.norm(pos_error)

      if pos_error_normed > 1e-3:
        kappa = (pos_error_normed * self.ua_max) / (np.sqrt(pos_error_normed + self.lookahead**2))
        vel_ref_unclamped = vel_target - (kappa @ pos_error) / (pos_error_normed) 
      else:
        vel_ref_unclamped = zeros_3_1

      vel_ref_x = self.__clamp(vel_ref_unclamped[0], self.vx_limits)
      vel_ref_y = self.__clamp(vel_ref_unclamped[1], self.vy_limits)
      vel_ref_z = self.__clamp(vel_ref_unclamped[2], self.vz_limits)

      twist_ref_msg.twist.linear.x = vel_ref_x
      twist_ref_msg.twist.linear.y = vel_ref_y
      twist_ref_msg.twist.linear.z = vel_ref_z

      self.reference_velocity_publisher.publish(twist_ref_msg)

      # Should have some logic for switching to the ECEF-coordinate frame in case 
      # the error becomes too large. Requires the guidance to have an overview of 
      # the target position
      # max_error = rospy.get_param("~max_error_normed", default=10)
      # if self.guidance_state == GuidanceState.RELATIVE_TO_HELIPAD and pos_error_normed > max_error:
      #   # May consider requesting ECEF-position of the target
      #   # Idle, such that it will not move until the position is updated
      #   # self.guidance_state = GuidanceState.IDLE
      #   # self.__request_target_ecef_position() # Set the state to ECEF
      #   pass

        # Might consider having the action-executor to perform this action

      self.rate.sleep()


def main():
  guidance_law = PurePursuitGuidanceLaw()
  guidance_law.calculate_velocity_reference()


if __name__ == "__main__":
  main()
