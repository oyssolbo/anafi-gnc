#!/usr/bin/python3

import rospy 
import std_msgs.msg
import sensor_msgs.msg

from scipy.spatial.transform import Rotation

from geometry_msgs.msg import TwistStamped, PointStamped, QuaternionStamped

from anafi_uav_msgs.msg import PointWithCovarianceStamped
from anafi_uav_msgs.srv import SetDesiredPosition, SetDesiredPositionRequest, SetDesiredPositionResponse

import numpy as np
import guidance_helpers.utilities as utilities

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning) 

class PurePursuitGuidanceLaw():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "pure_pursuit_guidance_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Initialize parameters
    pure_pursuit_params = rospy.get_param("~pure_pursuit_parameters")
    velocity_limits = rospy.get_param("~velocity_limits")
    
    self.ua_max = pure_pursuit_params["ua_max"]
    self.lookahead = pure_pursuit_params["lookahead"]
    self.fixed_kappa = pure_pursuit_params["kappa"]

    self.vx_limits = velocity_limits["vx"]
    self.vy_limits = velocity_limits["vy"]
    self.vz_limits = velocity_limits["vz"]

    self.desired_altitude : float = -1.0

    self.position_timestamp : std_msgs.msg.Time = None

    self.desired_position : np.ndarray = None  # [xd, yd, zd]
    self.position : np.ndarray = None 

    self.last_rotation_matrix_body_to_vehicle : np.ndarray = None

    # Set up subscribers 
    rospy.Subscriber("/estimate/ekf", PointWithCovarianceStamped, self._ekf_cb)

    self.use_ned_pos_from_gnss : bool = rospy.get_param("/use_ned_pos_from_gnss")
    if self.use_ned_pos_from_gnss:
      rospy.loginfo("Node using position estimates from GNSS. Estimates from EKF disabled")
      rospy.Subscriber("/anafi/ned_pos_from_gnss", PointStamped, self._ned_pos_cb)
      rospy.Subscriber("/anafi/attitude", QuaternionStamped, self._attitude_cb)
    else:
      rospy.loginfo("Node using position estimates from EKF. Position estimates from GNSS disabled")
      rospy.Subscriber("/estimate/ekf", PointWithCovarianceStamped, self._ekf_cb)

    # Set up publishers
    self.reference_velocity_publisher = rospy.Publisher("/guidance/pure_pursuit/velocity_reference", TwistStamped, queue_size=1)

    # Set up services
    rospy.Service("/guidance/service/set_desired_position", SetDesiredPosition, self._set_desired_position_cb)


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
    
    # Positions must be transformed to body
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


  def _set_desired_position_cb(self, position_req : SetDesiredPositionRequest) -> SetDesiredPositionResponse:
    self.desired_position = np.array([position_req.x_d, position_req.y_d, position_req.z_d], dtype=np.float) 

    res = SetDesiredPositionRequest()
    res.success = True
    return res 


  def _clamp(
        self, 
        value: float, 
        limits: tuple
      ) -> float:
    return np.min([np.max([value, limits[0]]), limits[1]])


  def _get_valid_pos_error(self) -> np.ndarray:
    """
    Returns a valid error for position
    """
    if (self.position_timestamp is None):
      return np.zeros((3, 1))

    # Using a target-position above the helipad to guide safely
    # target_position = np.array([0, 0, 0.25]).reshape((3, 1))
    # error = -self.position #- target_position
    # altitude_error = (self.desired_altitude + self.position[2]) 
    if np.linalg.norm(self.position[:2]) >= 0.2 and np.abs(self.position[2]) < 1.0:
      altitude_error = 0
    return np.array([self.position[0], self.position[1], altitude_error])


  def calculate_velocity_reference(self) -> None:
    """
    Generate a velocity reference from a position error using the pure
    pursuit guidance law as defined in Fossen 2021.
    """
    twist_ref_msg = TwistStamped()

    zeros_3_1 = np.zeros((3, 1))
    vel_target = zeros_3_1 # Possible extension to use constant bearing guidance in the future

    while not rospy.is_shutdown():
      if self.position is None:
        self.rate.sleep()
        continue

      pos_error = self._get_valid_pos_error()
      pos_error_normed = np.linalg.norm(pos_error)
      horizontal_error_normed = np.linalg.norm(pos_error[:2])

      # Control vertical position error when low horizontal error
      if horizontal_error_normed > 0.25:
        self.desired_altitude = -1.0 # This should utilize a sigmoid-function or something
      else:
        self.desired_altitude = 0.0

      if pos_error_normed > 1e-3:
        kappa = (pos_error_normed * self.ua_max) / (np.sqrt(pos_error_normed + self.lookahead**2))
        vel_ref_unclamped = vel_target - (kappa * pos_error) / (pos_error_normed) 
      else:
        vel_ref_unclamped = zeros_3_1

      vel_ref_x = self._clamp(vel_ref_unclamped[0], self.vx_limits)
      vel_ref_y = self._clamp(vel_ref_unclamped[1], self.vy_limits)
      vel_ref_z = self._clamp(vel_ref_unclamped[2], self.vz_limits)

      twist_ref_msg.header.stamp = rospy.Time.now()
      twist_ref_msg.twist.linear.x = vel_ref_x
      twist_ref_msg.twist.linear.y = vel_ref_y
      twist_ref_msg.twist.linear.z = vel_ref_z

      self.reference_velocity_publisher.publish(twist_ref_msg)
      self.rate.sleep()


def main():
  guidance_law = PurePursuitGuidanceLaw()
  guidance_law.calculate_velocity_reference()


if __name__ == "__main__":
  main()
