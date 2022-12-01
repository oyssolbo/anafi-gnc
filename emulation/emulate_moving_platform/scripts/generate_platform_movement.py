#!/usr/bin/python3

import rospy 
import numpy as np
from anafi_uav_msgs.msg import PointWithCovarianceStamped
from geometry_msgs.msg import QuaternionStamped, PointStamped

from scipy.spatial.transform import Rotation

class EmulateMovingPlatform():
  def __init__(self) -> None:

    rospy.init_node("emulate_moving_platform")
    node_rate = rospy.get_param("node_rate")
    self.dt = 1.0 / node_rate 
    self.rate = rospy.Rate(node_rate)

    # Initialize private variables
    self.anafi_position_ned : np.ndarray = np.zeros((3, 1))
    self.last_rotation_matrix_body_to_vehicle : np.ndarray = None

    # Set up publishers
    self.anafi_body_position_measurement_pub = rospy.Publisher("/estimate/ekf", PointWithCovarianceStamped, queue_size=1)
    self.platform_ned_position_pub = rospy.Publisher("/emulation/platform_position_ned", PointStamped, queue_size=1)

    # Set up subscribers
    rospy.Subscriber("/anafi/attitude", QuaternionStamped, self._anafi_attitude_cb)
    rospy.Subscriber("/anafi/ned_pos_from_gnss", PointStamped, self._anafi_ned_pos_cb)


  def _anafi_attitude_cb(self, msg : QuaternionStamped) -> None:    
    rotation = Rotation.from_quat([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    self.attitude_rpy = rotation.as_euler('xyz', degrees=False).reshape((3, 1))
    self.last_rotation_matrix_body_to_vehicle = rotation.as_matrix()

  def _anafi_ned_pos_cb(self, msg : PointStamped) -> None:
    if self.last_rotation_matrix_body_to_vehicle is None:
      # Impossible to convert positions to body frame
      return
    
    # Converting from NED origin-to-drone position to Body drone-to-origin position  
    self.anafi_position_ned = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float) 


  def emulate_platform_movement(self) -> None:
    # Assuming that the ship is at sea. Using first order Nomoto model
    # with a sinusoidal heave motion
    
    deg_to_rad : float = np.pi / 180.0

    U : float = 0.5   # [m/s] Absolute value
    wave_height : float = 0.1

    K_z : float = 0.5  # Frequency of 0.1 rad

    T_r : float = 1
    K_r : float = 1

    mu_dot_std : float = 0.1

    delta_min : float = -15 * deg_to_rad
    delta_max : float = 15 * deg_to_rad
    delta_mean = 2.5 * deg_to_rad

    t : float = 0 

    pos_ned : np.ndarray = np.zeros((3, 1))
    vel_body : np.ndarray = np.array([U, 0, 0]) 
    psi : float = 0
    r : float = 0
    delta : float = 0
    mu : float = 0

    while not rospy.is_shutdown():
      # Standard TTK4190 stuff
      R_z : np.ndarray = Rotation.from_euler("z", psi).as_matrix()

      pos_ned_dot : np.ndarray = R_z @ vel_body
      z_dot : float = wave_height * K_z * np.cos(K_z * t)
      r_dot : float = (1 / T_r) * (-r + K_r * delta)
      mu_dot : float = np.random.normal(0, mu_dot_std)
      delta = np.max([np.min([delta_mean + mu, delta_max]), delta_min])

      pos_ned = pos_ned.reshape(3) + (self.dt * pos_ned_dot)
      vel_body[2] = z_dot
      psi = psi + self.dt * r
      r = r + self.dt * r_dot 
      mu = mu + self.dt * mu_dot

      t = t + self.dt

      # Calculate the positional error in body frame
      if not self.last_rotation_matrix_body_to_vehicle is None:
        position_diff_ned : np.ndarray = pos_ned - self.anafi_position_ned
        position_diff_body : np.ndarray = self.last_rotation_matrix_body_to_vehicle.T @ position_diff_ned
      else:
        rospy.logwarn_throttle(1, "Unable to acquire attitude estimates from the Anafi")
        position_diff_body : np.ndarray = np.zeros((3, 1))

      # Publish the data
      position_error_body_msg = PointWithCovarianceStamped()
      position_error_body_msg.header.stamp = rospy.Time.now()
      position_error_body_msg.position.x = position_diff_body[0]
      position_error_body_msg.position.y = position_diff_body[1]
      position_error_body_msg.position.z = position_diff_body[2]
      self.anafi_body_position_measurement_pub.publish(position_error_body_msg)

      platform_ned_pos_msg = PointStamped()
      platform_ned_pos_msg.header.stamp = rospy.Time.now()
      platform_ned_pos_msg.point.x = pos_ned[0]
      platform_ned_pos_msg.point.y = pos_ned[1]
      platform_ned_pos_msg.point.z = pos_ned[2]
      self.platform_ned_position_pub.publish(platform_ned_pos_msg)

      self.rate.sleep()


def main():
  emulate_moving_platform = EmulateMovingPlatform()
  emulate_moving_platform.emulate_platform_movement()


if __name__ == "__main__":
  main()
