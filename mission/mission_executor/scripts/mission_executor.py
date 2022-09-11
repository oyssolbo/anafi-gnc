#!/usr/bin/python3

import rospy
import rospkg
import actionlib

import std_msgs.msg
import perception.msg
import drone_interface.msg

import subprocess
import numpy as np

from enum import Enum
from collections import deque

import anafi_uav_msgs.msg 
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsResponse
from std_srvs.srv import SetBool, Trigger


import mission_executor_helpers.utilities as utilities


class MissionExecutorNode():
  """
  What is left to do (except for the documentation):
  - store information when an action or service fails to perform some operation, and
    find a method for solving this. Must likely be passed to the planner
  - usage of config file for storing topics/actions/services etc
  - finding a method for passing locations  
  """
  def __init__(self) -> None:
    # Setup node
    node_name = rospy.get_param("~node_name", default="mission_executor_node")
    node_rate = rospy.get_param("~node_rate", default=10)
    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup subscribers
    rospy.Subscriber("/drone/out/telemetry", anafi_uav_msgs.msg.AnafiTelemetry, self.__drone_telemetry_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.PointWithCovarianceStamped, self.__ekf_cb)

    # Setup services
    rospy.Service("/mission_planner/set_planned_actions", SetBool, self.__set_planned_actions_srv_cb)

    # Setup actions
    self.is_action_list_updated : bool = False
    self.action_update_timestep : rospy.Time = None
    self.action_list : list[str] = []

    self.last_telemetry_msg : anafi_uav_msgs.msg.AnafiTelemetry = None 
    self.last_ekf_msg : anafi_uav_msgs.msg.PointWithCovarianceStamped = None 


  def __drone_telemetry_cb(
        self, 
        msg: anafi_uav_msgs.msg.AnafiTelemetry
      ) -> None:
    self.last_telemetry_msg = msg
    self.new_telemetry_available = True


  def __ekf_cb(
        self, 
        msg: anafi_uav_msgs.msg.PointWithCovarianceStamped
      ) -> None:
    self.pos = np.array([
      msg.position.x,
      msg.position.y,
      msg.position.z
    ])


  def __set_planned_actions_srv_cb(
        self, 
        request : SetPlannedActions
      ) -> SetPlannedActionsResponse:
    msg_timestamp = request.header.stamp

    response = SetPlannedActionsResponse()
    if utilities.is_new_msg_timestamp(self.action_update_timestamp, msg_timestamp):
      self.action_update_timestamp = msg_timestamp
      num_actions = request.num_actions
      action_list = [""] * num_actions 

      for i in range(num_actions):
        action_list[i] = request.action_list[i]
      
      self.is_action_list_updated = True
      self.action_list = action_list
      self.is_ordered_to_cancel_current_action = request.cancel_current_action
      response.success = True 
    else:
      # Old service-request
      rospy.logerror("[__set_planned_actions_srv_cb()] Old service request received")
      response.success = False

    return response


  def __get_next_action_function(self) -> function:
    if self.action_list is None or self.action_list[0] == "":
      return self.__idle

    self.action_str = deque(self.action_list).popleft()
    self.max_expected_action_time_s = rospy.get_param("~maximum_expected_action_time_s")[self.action_str]
    self.interface_name = rospy.get_param("~interface_name")[self.action_str]
    
    if self.action_str == "takeoff":
      return self.__takeoff
    if self.action_str == "land":
      return self.__land
    if self.action_str == "track":
      return self.__track 
    if self.action_str == "search":
      return self.__search
    if self.action_str == "communicate":
      return self.__communicate
    if self.action_str == "travel_to":
      return self.__travel_to 
    if self.action_str == "drop_bouy":
      return self.__drop_bouy
    return self.__idle
    

  def __get_service_interface(
        self,
        service_type,
        service_name  : str,
        timeout       : float = 2.0
    ):
    rospy.wait_for_service(service_name, timeout=timeout)
    return rospy.ServiceProxy(service_name, service_type)


  def __idle(self):
    rospy.loginfo("Idling")


  def __takeoff(self):
    rospy.loginfo("Trying to take off")

    try:
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.interface_name,
        timeout=2.0
      )()
      if not service_response.success:
        rospy.loginfo("[__takeoff()] {} denied takeoff".format(self.interface_name))

    except:
      rospy.logerr("[__takeoff()] {} unavailable".format(self.interface_name))


  def __land(self):
    rospy.loginfo("Trying to land")

    try:
      service_timeout = 0.5 # Short timeout on landing due to strict schedules
      
      # Disable the attitude controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name="/attitude_controller/service/enable_controller", # Get from config-file
        timeout=service_timeout 
      )(SetBool(False))
      if not service_response.success:
        rospy.logerr("[__land()] Cannot disable attitude-controller")
        raise ValueError()

      # Land
      service_response = self.__get_service_interface(
        service_type=Trigger,
        service_name=self.interface_name,
        timeout=service_timeout 
      )(Trigger())
      if not service_response.success:
        rospy.loginfo("[__land()] {} denied landing".format(self.interface_name))

    except:
      rospy.logerr("[__land()] {} unavailable".format(self.interface_name))


  def __track(self) -> None:
    rospy.loginfo("Trying to enable tracking of helipad")

    try:
      service_timeout = 2.0 
      
      # Enable the attitude controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name="/attitude_controller/service/enable_controller", # Get from config-file
        timeout=service_timeout 
      )(SetBool(True))
      if not service_response.success:
        rospy.logerr("[__track()] Cannot enable attitude-controller")
        raise ValueError()

    except:
      rospy.logerr("[__track()] {} unavailable".format(self.interface_name))
    
  
  def __search(self) -> None:
    rospy.loginfo("Trying to search an area")
    # Must use some trajectory planning to generate paths or trajectories for
    # the search in the desired area
  
  
  def __communicate(self) -> None:
    # Might be split into its own thread or similar
    rospy.loginfo("Communicating")
  

  def __travel_to(self) -> None:
    rospy.loginfo("Traveling")


  def __drop_bouy(self) -> None:
    rospy.loginfo("Dropping bouya")


  def __wait_for_drone_state(
        self, 
        state         : str,
        min_count     : int,
        max_wait_time : float = 5.0
      ) -> bool:
    start_time = rospy.Time.now()

    counter = 0
    duration = 0

    # Could be problematic if the planner updates the mission 
    while not rospy.is_shutdown() and duration < max_wait_time:
      if self.new_telemetry_available:
        flying_state = self._prev_telemetry.flying_state
        if flying_state == state:
          counter += 1
          if counter >= min_count:
            break
        else:
          counter = 0
      self.new_telemetry_available = False
      rospy.sleep(0.1)
      duration = utilities.calculate_timestamp_difference_ns(start_time, rospy.Time.now()) * 1e-9
    
    return duration < max_wait_time
    

  def __load_locations(self):
    # Load the locations to travel to
    return 


  def __check_current_action_finished(self) -> bool:
    """
    Check whether an action fulfills the criteria to be assumed 
    finished

    Cannot use __wait_for_drone_state() as that could block the outerloop
    time-check, causing it to cancel an action

    A more stable method should be implemented
    """

    if self.last_ekf_msg is None or self.last_telemetry_msg is None:
      return False

    if self.action_str == "takeoff":
      # Once the drone records hovering, it has taken off
      return self.last_telemetry_msg.flying_state == "hovering"

    if self.action_str == "land":
      return self.last_telemetry_msg.flying_state == "landing"

    if self.action_str == "track":
      # Check that the error is small enough
      pos_error_normed = np.linalg.norm(self.pos)
      return pos_error_normed < 0.25 # Config-file

    # The different actions are set as True, since these are not implemented
    if self.action_str == "search":
      return True
    
    if self.action_str == "communicate":
      return True
    
    if self.action_str == "travel_to":
      return True 
    
    if self.action_str == "drop_bouy":
      return True

    return False


  def __cancel_action(self) -> None:
    """
    Bit unsure how to develop this correctly, as there are multiple 
    complexities which will be linked to each case
    """
    rospy.loginfo("Cancelling of actions are not implemented")


  def execute_actions(self):
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
      # Things to take into account:
      # 1. What should happen if a new action sequence is received?
      #     Should the current action be cancelled or should it continue?
      # 2. What should occur if actions are taking too long? Should they 
      #     be preempted / cancelled? 
      # 3. How to include information about f.ex. search area or where one 
      #     should travel to? Should this be included in the message?

      # Things that could cause an action to be cancelled:
      # Takes too long
      # Ordered from the planner that the current action must be aborted
      # Current action has finished

      # But what is going to happen if a new set of requested actions are received?
      # Should one continue to finish the current action, or should one just start 
      # with the new actions? Could be some mediation between the planner and the 
      # executor 

      # Have a discussion with Simen about this, because from a planning perspective
      # there will be situations when the system should be replanned

      current_time = rospy.Time.now()
      passed_time_ns = utilities.calculate_timestamp_difference_ns(start_time, current_time)
      passed_time_s = passed_time_ns * 1e-9

      is_current_action_finished = self.__check_current_action_finished()     
      is_action_cancellable = (passed_time_s > self.max_expected_action_time_s) or (self.is_ordered_to_cancel_current_action) 

      if is_current_action_finished:# or is_action_cancellable: 
        # if is_action_cancellable:
        #   # Only a small subset of actions are possible to cancel, and 
        #   # would require some complex logic to get it right
        #   self.is_ordered_to_cancel_current_action = False  
        
        #   rospy.loginfo("Cancelling action: {}".format(self.action_str))
        #   self.__cancel_action()

        #   # Get the helicopter to hover - could pherhaps be done using the relative movement
        #   if not self.__wait_for_drone_state(state="hovering", min_count=5, max_wait_time=10.0):
        #     rospy.logfatal("[execute_actions()] Drone failed to hover after cancelling action")

        # Start the next action
        self.__get_next_action_function()()
        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  mission_executor_node = MissionExecutorNode()
  mission_executor_node.execute_actions()

if __name__ == "__main__":
  main()
