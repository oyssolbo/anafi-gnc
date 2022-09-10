#!/usr/bin/python3

import os
import sys 
import yaml

import numpy as np

import rospy 
import std_msgs.msg
from anafi_uav_msgs.msg import AttitudeSetpoint

def load_config_file(node_name: str) -> dict:
    """
    Load the control parameters when the controller is run from an arbitrary node.

    Parameters
    ----------
    node_name : str
        The name of the node the starting the controller

    Returns
    -------
    dict
        The control parameters config

    Author
    ------ 
    Martin Falang (2021-2022)

    """
    print(f"/../../{node_name}/config_file")
    config_file = rospy.get_param(f"/{node_name}/config_file")
    script_dir = os.path.dirname(os.path.realpath(__file__))

    try:
      with open(f"{script_dir}/../config/{config_file}") as f:
        config = yaml.safe_load(f)
    except Exception as e:
      rospy.logerr(f"Failed to load config: {e}")
      sys.exit()

    return config
    

def load_config(node_name: str, rosparam_name: str) -> dict:
    """
    Load the control parameters when the controller is run from an arbitrary node.

    Parameters
    ----------
    node_name : str
        The name of the node the starting the controller
    rosparam_name : str
        The name of the ROS parameter where the filename of the config file is stored

    Returns
    -------
    dict
        The control parameters config
        

    Author
    ------ 
    Martin Falang (2021-2022)

    """
    config_file = rospy.get_param(f"/{node_name}/{rosparam_name}")
    script_dir = os.path.dirname(os.path.realpath(__file__))

    try:
      with open(f"{script_dir}/../config/{config_file}") as f:
        config = yaml.safe_load(f)
    except Exception as e:
      rospy.logerr(f"Failed to load config: {e}")
      sys.exit()

    return config


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