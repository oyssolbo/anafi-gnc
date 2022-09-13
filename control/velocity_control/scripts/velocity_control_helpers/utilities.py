#!/usr/bin/python3

import numpy as np

import rospy 
from anafi_uav_msgs.msg import AttitudeSetpoint


def pack_attitude_ref_msg(att_ref: np.ndarray) -> AttitudeSetpoint:
    """Generate a attitude setpoint message attitude setpoints.

    Parameters
    ----------
    att_ref : np.ndarray
        Attitude setpoint. Format: [roll, pitch, yaw_rate, climb_rate]
        All angles in degrees, climb_rate positive up.

    Returns
    -------
    drone_interface.msg.AttitudeSetpoint
        ROS message with the attitude setpoint

    Author
    ------ 
    Martin Falang (2021-2022)
    
    """

    msg = AttitudeSetpoint()
    msg.header.stamp = rospy.Time.now()
    msg.roll = att_ref[0]
    msg.pitch = att_ref[1]
    msg.yaw_rate = att_ref[2]
    msg.climb_rate = att_ref[3]

    return msg


def calculate_timestamp_difference_s(
      oldest_stamp : rospy.Time,
      newest_stamp : rospy.Time
    ) -> float:
  """
  Calculates the time difference between two timestamps

  Returns the time difference in ns 
  """
  if oldest_stamp is None:
    return 0.0
  return (newest_stamp - oldest_stamp).to_sec()


def is_new_msg_timestamp(
      own_timestamp : rospy.Time, 
      msg_timestamp : rospy.Time
    ) -> bool:
  if (own_timestamp is not None):
    if calculate_timestamp_difference_s(oldest_stamp=own_timestamp, newest_stamp=msg_timestamp) <= 0.0:
      # Old message
      return False 
  return True
