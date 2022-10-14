#!/usr/bin/python3

import rospy 
import numpy as np
from anafi_uav_msgs.msg import PointWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

import pyproj
from scipy.spatial.transform import Rotation

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

class GeneratePositionErrors():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_position_error_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.position_error_pub = rospy.Publisher("/estimate/ekf", PointWithCovarianceStamped, queue_size=1)   
 
    # Set up private variables
    self.use_external_pos_estimates : bool = rospy.get_param("/use_external_pos_estimates")
    if self.use_external_pos_estimates:
      use_qualisys : bool = rospy.get_param("/use_qualisys")

      self.pos_estimate_timestamp : rospy.Time = None 
      self.estimated_pos_ned : np.ndarray = None 
      self.initial_pos : np.ndarray = None
      self.rot_matrix_body_to_vehicle : np.ndarray = None

      if use_qualisys:
        rospy.Subscriber("/qualisys/Anafi/pose", PoseStamped, self._qualisys_cb)
      else:
        rospy.Subscriber("/anafi/pose", PoseStamped, self._sim_cb)

    self.time_between_switch : float = 5


  def _is_new_msg_stamp(self, oldest_stamp : rospy.Time, newest_stamp : rospy.Time) -> bool:
    if oldest_stamp is None:
      return True
    return (newest_stamp - oldest_stamp).to_sec() > 0


  def _sim_cb(self, msg : PoseStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not self._is_new_msg_stamp(self.pos_estimate_timestamp, msg_timestamp):
      # Old message
      return

    # If these are zero, no GNSS-message was received
    pos_list = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    quat_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    if all(val == 0 for val in pos_list):
      # No position fix
      return 

    latitude_deg = pos_list[0]
    longitude_deg = pos_list[1]
    altitude_m = pos_list[2]

    # Convertion from long / lat / altitude to ECEF
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, longitude_deg, latitude_deg, altitude_m, radians=False)
    
    self.pos_estimate_timestamp = msg_timestamp
    rpy = Rotation.from_quat(quat_list).as_euler("xyz", degrees=False)
    rpy[0] = 0
    rpy[1] = 0
    self.rot_matrix_body_to_vehicle = Rotation.from_euler("xyz", rpy).as_matrix()

    # Note that this is in ECEF. Assumption to use it for NED only valid for small movements
    estimated_pos_ecef = np.array([x, y, z], dtype=np.float).T   

    if self.initial_pos is None:
      self.initial_pos = estimated_pos_ecef
    self.estimated_pos_ned = estimated_pos_ecef


  def _qualisys_cb(self, msg : PoseStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not self._is_new_msg_stamp(self.pos_estimate_timestamp, msg_timestamp):
      # Old message
      return

    self.pos_estimate_timestamp = msg_timestamp     
    estimated_pos_ned = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float)
    quat_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rpy = Rotation.from_quat(quat_list).as_euler("xyz", degrees=False)
    rpy[0] = 0
    rpy[1] = 0
    self.rot_matrix_body_to_vehicle = Rotation.from_euler("xyz", rpy).as_matrix()

    if self.initial_pos is None:
      self.initial_pos = estimated_pos_ned
    self.estimated_pos_ned = estimated_pos_ned


  def generate_position_errors(self) -> None:
    """
    Testing in the simulator shows that the EKF generates position estimates of 
    the platform relative to the drone. 

      x > 0, means the drone is 'behind' the platform. x > 0 should induce a forward movement ()
      y > 0, means the drone is to the left of the platform. y > 0 should induce a movement to the right
      z > 0, means the drone is above the platform. z > 0 should induce a downward movement
    """

    start_time = rospy.Time.now()

    xs_ned = [0, 0.5, 0, -0.5, 0]
    ys_ned = [0, 0, 0.5, 0, -0.5]
    zs_ned = [0] * 5 
    
    idx = 0

    while not rospy.is_shutdown():
      dx = 0
      dy = 0
      dz = 0
      
      if self.use_external_pos_estimates:
        if self.initial_pos is not None and self.rot_matrix_body_to_vehicle is not None:
          x_des_ned = xs_ned[idx]
          y_des_ned = ys_ned[idx]
          z_des_ned = zs_ned[idx]

          # Subtracting the initial is to express things with the origin as the starting location 
          pos_desired_vehicle = np.array([x_des_ned, y_des_ned, z_des_ned], dtype=np.float) + self.initial_pos
          pos_vehicle = (self.estimated_pos_ned) 

          pos_diff_vehicle = pos_desired_vehicle - pos_vehicle
          pos_diff_body = self.rot_matrix_body_to_vehicle.T @ pos_diff_vehicle 

          # Circle of acceptance
          if np.linalg.norm(pos_diff_body[:2]) < 0.15: 
            dx = 0
            dy = 0
            dz = 0

            idx += 1
            if idx >= len(xs_ned):
              idx = 0
            
          else:
            dx = pos_diff_body[0]
            dy = pos_diff_body[1]
            dz = pos_diff_body[2]                # Assuming a constant altitude, meaning that the drone is already airborne


      else:
        dx = xs_ned[idx]
        dy = ys_ned[idx]
        dz = 0 #zs_ned[idx]

        # x = x + self.time_between_switch * np.random.normal(0.0, 0.05) 
        # y = y + self.time_between_switch * np.random.normal(0.0, 0.05) 
        # z = z + self.time_between_switch * np.random.normal(0.0, 0.01)

        if (rospy.Time.now() - start_time).to_sec() > self.time_between_switch:
          idx += 1
          if idx >= len(xs_ned):
            idx = 0

            start_time = rospy.Time.now()

      point_msg = PointWithCovarianceStamped()
      point_msg.header.stamp = rospy.Time.now()
      point_msg.position.x = dx
      point_msg.position.y = dy
      point_msg.position.z = dz
      self.position_error_pub.publish(point_msg)

      self.rate.sleep()


def main():
  generate_position_errors = GeneratePositionErrors()
  generate_position_errors.generate_position_errors()


if __name__ == "__main__":
  main()
