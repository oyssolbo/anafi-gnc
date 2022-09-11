#!/usr/bin/env python3

import os
import sys 
import yaml

import numpy as np

import std_msgs


def calculate_timestamp_difference_ns(
      oldest_stamp : std_msgs.msg.Time,
      newest_stamp : std_msgs.msg.Time
    ) -> float:
  """
  Calculates the time difference between two timestamps

  Returns the time difference in ns 
  """
  if oldest_stamp is None:
    return 0.0
  sec_diff = newest_stamp.sec - oldest_stamp.sec
  ns_diff = newest_stamp.nsec - oldest_stamp.nsec
  return (sec_diff * 1e9) + ns_diff

def is_new_msg_timestamp(
      own_timestamp : std_msgs.msg.Time, 
      msg_timestamp : std_msgs.msg.Time
    ) -> bool:
  if (own_timestamp is not None):
    if calculate_timestamp_difference_ns(oldest_stamp=own_timestamp, newest_stamp=msg_timestamp) <= 0.0:
      # Old message
      return False 
  return True