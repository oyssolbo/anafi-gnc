#!/usr/bin/python3

import rospy

import numpy as np

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
  