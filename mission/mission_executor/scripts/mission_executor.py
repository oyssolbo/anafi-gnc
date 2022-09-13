#!/usr/bin/python3

import rospy
import rospkg

import pyproj
import numpy as np

from collections import deque
from typing import Callable

import anafi_uav_msgs.msg 
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsRequest, SetPlannedActionsResponse

import std_msgs.msg
import sensor_msgs.msg
from std_srvs.srv import SetBool

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
    rospy.Subscriber("/drone/out/gps", sensor_msgs.msg.NavSatFix, self.__drone_gnss_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.EkfOutput, self.__ekf_cb)

    # Setup services
    rospy.Service("/mission_executor/service/set_planned_actions", SetPlannedActions, self.__set_planned_actions_srv)

    # Setup publishers
    self.takeoff_pub = rospy.Publisher("/drone/cmd/takeoff", std_msgs.msg.Empty, queue_size=1)
    self.land_pub = rospy.Publisher("/drone/cmd/land", std_msgs.msg.Empty, queue_size=1)

    # Initializing values
    self.is_action_list_updated : bool = False
    self.action_list : list[str] = []

    self.last_telemetry_msg : anafi_uav_msgs.msg.AnafiTelemetry = None 

    self.is_ordered_to_cancel_current_action : bool = False

    self.velocity_controller_service_name = rospy.get_param("~enable_controller_service_names")["velocity_controller"]

    self.pos_relative_to_helipad : np.ndarray = None
    self.pos_ecef : np.ndarray = None 
    self.ned_origin_in_ECEF : np.ndarray = None # The origin of the NED-frame expressed in ECEF
    self.desired_ecef_coordinates : np.ndarray = None 

    # self.__initialize_gnss()


  def __drone_telemetry_cb(
        self, 
        msg: anafi_uav_msgs.msg.AnafiTelemetry
      ) -> None:
    self.last_telemetry_msg = msg
    self.new_telemetry_available = True


  def __drone_gnss_cb(self, msg : sensor_msgs.msg.NavSatFix) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.gnss_timestamp, msg_timestamp):
      # Old message
      return

    gnss_status = msg.status
    if gnss_status == -1:
      # No position fix
      return 

    latitude_rad = msg.latitude * np.pi / 180.0
    longitude_rad = msg.longitude * np.pi / 180.0
    altitude_m = msg.altitude 

    # Convertion from long / lat / altitude to ECEF
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, longitude_rad, latitude_rad, altitude_m, radians=True)
    
    self.gnss_timestamp = msg_timestamp
    self.pos_ecef = np.array([x, y, z]).T


  def __ekf_cb(self, msg : anafi_uav_msgs.msg.EkfOutput) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos_relative_to_helipad = np.array([msg.x, msg.y, msg.z]).T


  def __set_planned_actions_srv(
        self, 
        request : SetPlannedActionsRequest
      ) -> SetPlannedActionsResponse:
    response = SetPlannedActionsResponse()

    num_actions = request.num_actions
    action_list = [""] * num_actions 

    for i in range(num_actions):
      action_list[i] = request.action_list[i]
      
    self.is_action_list_updated = True
    self.action_list = action_list
    self.is_ordered_to_cancel_current_action = request.cancel_current_action
    response.success = True 

    return response


  # using this naively, prevented the mission planner to set the mission...
  # def __initialize_gnss(self):
  #   # Try to initialize the location of the local frame 
  #   # Note that it could be possible to have this updated later...
  #   max_wait_time_s = 20 # Config
  #   start_time = rospy.Time.now()
  #   while not rospy.is_shutdown() and utilities.calculate_timestamp_difference_s(start_time, rospy.Time.now()) < max_wait_time_s:
  #     if self.pos_ecef is not None:
  #       self.ned_origin_in_ECEF = self.pos_ecef
  #       rospy.loginfo("Platform's position in ECEF found")
  #       return True
  #     self.rate.sleep()

  #   rospy.logerr("Unable to initialize the platform's position in ECEF...")
  #   return False


  def __get_next_action_function(self) -> Callable:
    if self.action_list is None or not self.action_list or self.action_list[0] == "":
      self.max_expected_action_time_s = rospy.get_param("~maximum_expected_action_time_s")["idle"]
      return self.__idle

    self.action_str = deque(self.action_list).popleft()
    self.max_expected_action_time_s = rospy.get_param("~maximum_expected_action_time_s")[self.action_str]
    
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
      service_timeout = 2.0
      
      # Disable the velocity controller to have a predefined state
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.velocity_controller_service_name,
        timeout=service_timeout 
      )(SetBool(False))
      if not service_response.success:
        rospy.logerr("[__takeoff()] Cannot disable velocity-controller")
        raise ValueError()

      # Taking off
      self.takeoff_pub.publish(std_msgs.msg.Empty())
      rospy.loginfo("Taking off")


    except Exception as e:
      rospy.logerr("[__takeoff()] {} unavailable. Error {}".format(self.velocity_controller_service_name, e.what()))


  def __land(self):
    rospy.loginfo("Trying to land")

    try:
      service_timeout = 0.5 # Short timeout on landing due to strict schedules
      
      # Disable the velocity controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.velocity_controller_service_name, 
        timeout=service_timeout 
      )(SetBool(False))
      if not service_response.success:
        rospy.logerr("[__land()] Cannot disable velocity-controller")
        raise ValueError()

      # Land
      self.land_pub.publish(std_msgs.msg.Empty())
      rospy.loginfo("Landing")

    except Exception as e:
      rospy.logerr("[__land()] {} unavailable. Error: {}".format(self.velocity_controller_service_name, e.what()))


  def __track(self) -> None:
    rospy.loginfo("Trying to enable tracking of helipad")

    try:
      service_timeout = 2.0 
      
      # Enable the velocity controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.velocity_controller_service_name, 
        timeout=service_timeout 
      )(SetBool(True))
      if not service_response.success:
        rospy.logerr("[__track()] Cannot enable velocity-controller")
        raise ValueError()

      # The reference controller is activated
      # Use a guidance law to eliminate the errors in position
      # Could set the desired position in guidance

    except Exception as e:
      rospy.logerr("[__track()] {} unavailable. Error: {}".format(self.velocity_controller_service_name, e.what()))
    
  
  def __search(self) -> None:
    rospy.loginfo("Trying to search an area")
    # Must use some trajectory planning to generate paths or trajectories for
    # the search in the desired area
  
  
  def __communicate(self) -> None:
    # Might be split into its own thread or similar
    rospy.loginfo("Communicating")
  

  def __travel_to(self) -> None:
    rospy.loginfo("Traveling")
    # Must find a method for loading in the desired positions in NED or ECEF
    # Could either use the move_relative command developed previously or set the guidance-error accordingly
    # 
    # Could pherhaps assume that the desired locations are in NED, which could be translated into ECEF by
    # knowing the initial origin of the NED-frame 
    # But this would mean that the guidance-module must have two different states. One for tracking the 
    # helipad, where one tries to eliminate the positional error estimated using the EKF, and one for 
    # tracking a global position using ECEF-coordinates 

    # Get the desired location from somewhere
    desired_location = None 
    ned_coordinates = self.__load_locations_positions_ned(location=desired_location)
    
    if self.ned_origin_in_ECEF is None:
      rospy.logerr("No ECEF-coordinates are available for NED's origin")
      return 

    self.desired_ecef_coordinates = ned_coordinates + self.ned_origin_in_ECEF

    # Send the data to the guidance-module 
    # Discuss with Simen whether the guidance-module should be used
    # Might be simpler to use the move_relative function developed by Martin Falang
    relative_movement = self.desired_ecef_coordinates - self.pos_ecef

    # Should have some checks to prevent the drone from crashing into the water etc
    # Find a method for checking that ned_z > -1.0 or something, where -1.0 means that 
    # the drone is above the ground

    # The maximum speed could be made relative to the distance the system must travel
    relative_movement_normed = np.linalg.norm(relative_movement)
    max_horizontal_speed = 0.5 * np.exp(-relative_movement_normed) + 2.0 * (1 - np.exp(-relative_movement_normed))
    max_vertical_speed = 0.25 * np.exp(-relative_movement_normed) + 0.75 * (1 - np.exp(-relative_movement_normed))

    self.__move_relative(
      d_pos=relative_movement, 
      dpsi=0,
      max_horizontal_speed=max_horizontal_speed,
      max_vertical_speed=max_vertical_speed,
      max_yaw_rotation_speed=np.pi/4
    )


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
    duration_s = 0

    # Could be problematic if the planner updates the mission 
    while not rospy.is_shutdown() and duration_s < max_wait_time:
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
      duration_s = utilities.calculate_timestamp_difference_s(start_time, rospy.Time.now())
    
    return duration_s < max_wait_time
    

  def __load_locations_positions_ned(self, location : str) -> np.ndarray:
    # Load the locations to travel to
    # These should be loaded in either the ECEF-frame or NED-frame
    # return rospy.get_param("~locations_ned")[location]
    return np.zeros((3, 1))


  def __check_current_action_finished(self) -> bool:
    """
    Check whether an action fulfills the criteria to be assumed 
    finished

    Cannot use __wait_for_drone_state() as that could block the outerloop
    time-check, causing it to cancel an action

    A more stable method should be implemented
    """

    if self.last_telemetry_msg is None:
      return False

    if self.action_str == "takeoff":
      # Once the drone records hovering, it has taken off
      return self.last_telemetry_msg.flying_state == "hovering"

    if self.action_str == "land":
      return self.last_telemetry_msg.flying_state == "landing"

    if self.action_str == "track":
      # Check that the error is small enough
      pos_error_normed = np.linalg.norm(self.pos_relative_to_helipad)
      max_pos_tracking_error_normed = rospy.get_param("~max_pos_tracking_error_normed", default=0.25)
      return pos_error_normed < max_pos_tracking_error_normed 

    # The different actions are set as True, since these are not implemented
    if self.action_str == "search":
      return True
    
    if self.action_str == "communicate":
      return True
    
    if self.action_str == "travel_to":
      # Check whether the drone is close enough to the desired point of interest
      # May become slightly problematic when travelling to the platform, as the error might be 
      # too large for the tracking to handle
      horizontal_radius_of_acceptance = rospy.get_param("~horizontal_radius_of_acceptance", default=5.0)
      vertical_radius_of_acceptance = rospy.get_param("~vertical_radius_of_acceptance", default=7.5)
      
      if self.desired_ecef_coordinates is None:
        rospy.logerr("[__check_current_action_finished()] Desired ECEF coordinates are None")
        return False

      pos_error_ecef = self.desired_ecef_coordinates - self.pos_ecef
      return np.linalg.norm(pos_error_ecef[:2]) <= horizontal_radius_of_acceptance and np.abs(pos_error_ecef) <= vertical_radius_of_acceptance
    
    if self.action_str == "drop_bouy":
      return True

    return False

  def __move_relative(
        self, 
        d_pos                 : np.ndarray,
        dpsi                  : float       = 0,
        max_horizontal_speed  : float       = 0.5, 
        max_vertical_speed    : float       = 0.5, 
        max_yaw_rotation_speed: float       = np.pi/4
      ) -> None:
    msg = anafi_uav_msgs.msg.PositionSetpointRelative()
    msg.header.stamp = rospy.Time.now()
    msg.dx = d_pos[0]
    msg.dy = d_pos[1]
    msg.dz = d_pos[2]
    msg.dpsi = dpsi
    msg.max_horizontal_speed = max_horizontal_speed
    msg.max_vertical_speed = max_vertical_speed
    msg.max_yaw_rotation_speed = max_yaw_rotation_speed

    self._move_relative_publisher.publish(msg)


  def __cancel_action(self) -> None:
    """
    Bit unsure how to develop this correctly, as there are multiple 
    complexities which will be linked to each case
    """
    rospy.loginfo("Cancelling of actions are not implemented")


  def execute_actions(self):
    self.__get_next_action_function()
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
      passed_time_s = utilities.calculate_timestamp_difference_s(start_time, current_time)

      is_current_action_finished = self.__check_current_action_finished()     
      # is_action_cancellable = (passed_time_s > self.max_expected_action_time_s) or (self.is_ordered_to_cancel_current_action) 

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
