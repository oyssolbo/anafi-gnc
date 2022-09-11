#!/usr/bin/python3

from importlib.abc import Traversable
import rospy
import rospkg
import subprocess
import numpy as np
from enum import Enum

from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsResponse

import mission_executor_helpers.utilities as utilities

class Actions(Enum):
  TakeOff = 0
  Travel = 1
  Search = 2
  Communicate = 3 
  Drop = 4
  Land = 5
  Idle = 6


class MissionExecutorNode():
  """
  
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default="mission_executor_node")
    node_rate = rospy.get_param("~rate_Hz", default=10)
    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup services
    rospy.Service("/mission_planner/set_current_planned_actions", SetPlannedActions, self.__set_planned_actions_srv_cb)
    rospy.Service("/mission_planner/cancel_planned_actions", SetPlannedActions, self.__cancel_action_srv_cb) # But what will happen during a cancel

    # Setup actions
    self.is_action_list_updated : bool = False
    self.action_update_timestep : std_msgs.msg.Time = None
    self.action_list : list[str] = []
    self.cancel_current_action