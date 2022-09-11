#!/usr/bin/env python3

import rospy
import std_msgs


def calculate_timestamp_difference_s(
      oldest_stamp,
      newest_stamp
    ) -> float:
  """
  Calculates the time difference between two timestamps

  Returns the time difference in ns 
  """
  if oldest_stamp is None:
    return 0.0
  sec_diff = newest_stamp.secs - oldest_stamp.secs
  nanosec_diff = newest_stamp.nsecs - oldest_stamp.nsecs
  return sec_diff + nanosec_diff * 1e-9

def is_new_msg_timestamp(
      own_timestamp, 
      msg_timestamp
    ) -> bool:
  if (own_timestamp is not None):
    if calculate_timestamp_difference_s(oldest_stamp=own_timestamp, newest_stamp=msg_timestamp) <= 0.0:
      # Old message
      return False 
  return True