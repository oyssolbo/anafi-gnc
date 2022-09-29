#!/usr/bin/python3

import rospy 
import std_msgs.msg
import sensor_msgs.msg
import pyproj

from geometry_msgs.msg import TwistStamped

from anafi_uav_msgs.msg import PointWithCovarianceStamped, ReferenceStates
from anafi_uav_msgs.srv import SetDesiredPose, SetDesiredPoseRequest, SetDesiredPoseResponse

import numpy as np
import guidance_helpers.utilities as utilities

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)


class PIDGuidanceLaw():

  def __init__(self):

    # Directly from the config file Martin used
    # pid:
    #   x_axis:
    #     kp: 0.5
    #     ki: 0
    #     kd: 0
    #   y_axis:
    #     kp: 0.5
    #     ki: 0
    #     kd: 0
    #   z_axis:
    #     kp: 0.2
    #     ki: 0
    #     kd: 0

    # velocity_limits:
    #   vx: [-0.3, 0.3]
    #   vy: [-0.3, 0.3]
    #   vz: [-0.1, 0.1]


    self._Kp_x = 0.5  # params["x_axis"]["kp"]
    self._Ki_x = 0    # params["x_axis"]["ki"]
    self._Kd_x = 0    # params["x_axis"]["kd"]

    self._Kp_y = 0.5  # params["y_axis"]["kp"]
    self._Ki_y = 0    # params["y_axis"]["ki"]
    self._Kd_y = 0    # params["y_axis"]["kd"]

    self._Kp_z = 0.2  # params["z_axis"]["kp"]
    self._Ki_z = 0    # params["z_axis"]["ki"]
    self._Kd_z = 0    # params["z_axis"]["kd"]

    self._vx_limits = (-0.3, 0.3) # limits["vx"]
    self._vy_limits = (-0.3, 0.3) # limits["vy"]
    self._vz_limits = (-0.1, 0.1) # limits["vz"]

    print(10*"=", "Position controller control params", 10*"=")
    print(f"X-axis: \tKp: {self._Kp_x} \tKi: {self._Ki_x} \tKd: {self._Kd_x} \tLimits: {self._vx_limits}")
    print(f"Y-axis: \tKp: {self._Kp_y} \tKi: {self._Ki_y} \tKd: {self._Kd_y} \tLimits: {self._vy_limits}")
    print(f"Z-axis: \tKp: {self._Kp_z} \tKi: {self._Ki_z} \tKd: {self._Kd_z} \tLimits: {self._vz_limits}")
    print(56*"=")

    self._prev_ts = None
    self._error_int = np.zeros(3)
    self._prev_error = np.zeros(3)


    rospy.init_node("pid_guidance_node")
    self.rate = rospy.Rate(20) # Hardcoded from the pure-pursuit guidance

    rospy.Subscriber("/estimate/ekf", PointWithCovarianceStamped, self.__ekf_cb)
    self.reference_velocity_publisher = rospy.Publisher("/guidance/pid/velocity_reference", TwistStamped, queue_size=1)

    self.ekf_timestamp : rospy.Time = None


  def _clamp(self, value: float, limits: tuple):
    if value < limits[0]:
      return limits[0]
    elif value > limits[1]:
      return limits[1]
    else:
      return value


  def __ekf_cb(self, msg: PointWithCovarianceStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos_relative_to_helipad = np.array([msg.position.x, msg.position.y, msg.position.z]).T

  def __get_position_error(self) -> np.ndarray:
    if (self.ekf_timestamp is None):
      return np.zeros((3, 1))

    # Using a target-position above the helipad to guide safely
    target_position = np.array([0, 0, 0.25])
    error = self.pos_relative_to_helipad - target_position
    return -error


  def get_velocity_reference(self, pos_error_body: np.ndarray, ts: float, debug=False) -> np.ndarray:

    control3D = (pos_error_body.shape[0] == 3)

    e_x = pos_error_body[0]
    e_y = pos_error_body[1]

    if control3D:
      e_z = pos_error_body[2]

    if self._prev_ts is not None and ts != self._prev_ts:
      dt = (ts - self._prev_ts).to_sec()

      e_dot_x = (e_x - self._prev_error[0]) / dt
      e_dot_y = (e_y - self._prev_error[1]) / dt

      if control3D:
        e_dot_z = (e_z - self._prev_error[2]) / dt

      if control3D:
        self._prev_error = pos_error_body
      else:
        self._prev_error = np.hstack((pos_error_body, 0))

      # Avoid integral windup
      if self._vx_limits[0] <= self._error_int[0] <= self._vx_limits[1]:
        self._error_int[0] += e_x * dt

      if self._vy_limits[0] <= self._error_int[1] <= self._vy_limits[1]:
        self._error_int[1] += e_y * dt

      if control3D:
        if self._vz_limits[0] <= self._error_int[2] <= self._vz_limits[1]:
          self._error_int[2] += e_z * dt

    else:
      e_dot_x = e_dot_y = e_dot_z = 0

    self._prev_ts = ts

    vx_reference = self._Kp_x*e_x + self._Kd_x*e_dot_x + self._Ki_x*self._error_int[0]
    vy_reference = self._Kp_y*e_y + self._Kd_y*e_dot_y + self._Ki_y*self._error_int[1]

    vx_reference = self._clamp(vx_reference, self._vx_limits)
    vy_reference = self._clamp(vy_reference, self._vy_limits)

    if control3D:
      vz_reference = self._Kp_z*e_z + self._Kd_z*e_dot_z + self._Ki_z*self._error_int[2]
      vz_reference = self._clamp(vz_reference, self._vz_limits)
      velocity_reference = np.array([vx_reference, vy_reference, vz_reference])
    else:
      velocity_reference = np.array([vx_reference, vy_reference, 0])

    if debug:
      print(f"Timestamp: {ts}")
      print(f"Vx gains:\tP: {self._Kp_x*e_x:.3f}\tI: {self._Ki_x*self._error_int[0]:.3f}\tD: {self._Kd_x*e_dot_x:.3f} ")
      print(f"Vy gains:\tP: {self._Kp_y*e_y:.3f}\tI: {self._Ki_y*self._error_int[1]:.3f}\tD: {self._Kd_y*e_dot_y:.3f} ")
      if control3D:
        print(f"Vz gains:\tP: {self._Kp_z*e_z:.3f}\tI: {self._Ki_z*self._error_int[2]:.3f}\tD: {self._Kd_z*e_dot_y:.3f} ")
        print(f"Velocity references:\t vx: {vx_reference:.3f}\t vy: {vy_reference:.3f} vz: {vz_reference:.3f}")
      else:
        print(f"Velocity references:\t vx: {vx_reference:.3f}\t vy: {vy_reference:.3f}")
      print()

    return velocity_reference


  def run(self) -> None:
    while not rospy.is_shutdown():
      timestamp = rospy.Time.now()
      pos_error = self.__get_position_error()

      velocity_reference = self.get_velocity_reference(pos_error_body=pos_error, ts=timestamp)

      twist_msg = TwistStamped()
      twist_msg.header.stamp = timestamp
      twist_msg.twist.linear.x = velocity_reference[0]
      twist_msg.twist.linear.y = velocity_reference[1]
      twist_msg.twist.linear.z = velocity_reference[2]

      self.reference_velocity_publisher.publish(twist_msg)

      self.rate.sleep()



def main():
  pid_guidance_law = PIDGuidanceLaw()
  pid_guidance_law.run()

if __name__ == '__main__':
  main()

