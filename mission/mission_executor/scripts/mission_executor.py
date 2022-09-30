#!/usr/bin/python3

import rospy
import rospkg

import pyproj
import numpy as np

from collections import deque
from typing import Callable

import anafi_uav_msgs.msg 
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsRequest, SetPlannedActionsResponse
import olympe_bridge.msg 

import std_msgs.msg
import sensor_msgs.msg
from std_srvs.srv import SetBool

import mission_executor_helpers.utilities as utilities
import simple_missions.lab_test as lab_test

import warnings
warnings.filterwarnings('ignore', category=DeprecationWarning)

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
    # rospy.Subscriber("/drone/out/telemetry", anafi_uav_msgs.msg.AnafiTelemetry, self.__drone_telemetry_cb)
    rospy.Subscriber("/anafi/state", std_msgs.msg.String, self.__drone_state_cb) 
    rospy.Subscriber("/anafi/gnss_location", sensor_msgs.msg.NavSatFix, self.__drone_gnss_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.PointWithCovarianceStamped, self.__ekf_cb)

    # Setup services
    rospy.Service("/mission_executor/service/set_planned_actions", SetPlannedActions, self.__set_planned_actions_srv)

    # Setup publishers
    self.takeoff_pub = rospy.Publisher("/anafi/cmd_takeoff", std_msgs.msg.Empty, queue_size=1)
    self.land_pub = rospy.Publisher("/anafi/cmd_land", std_msgs.msg.Empty, queue_size=1)

    self.move_to_ecef_pub = rospy.Publisher("/anafi/cmd_moveto", olympe_bridge.msg.MoveToCommand, queue_size=1)
    self.move_relative_pub = rospy.Publisher("/anafi/cmd_moveby", olympe_bridge.msg.MoveByCommand, queue_size=1)

    # Services and actions to connect to
    self.velocity_controller_service_name = "/velocity_controller/service/enable_controller"

    # Initializing values
    self.pos_ecef : np.ndarray = None 
    self.ned_origin_in_ECEF : np.ndarray = None       # The origin of the NED-frame expressed in ECEF
    self.pos_relative_to_helipad : np.ndarray = None
    self.desired_ecef_coordinates : np.ndarray = None 

    self.is_action_list_updated : bool = False
    self.action_str_list : list[str] = []

    self.drone_state : str = None
    self.prev_drone_state : str = None
    self.drone_state_update_timestamp : bool = False

    self.is_ordered_to_cancel_current_action : bool = False

    self.gnss_timestamp : rospy.Time = None
    self.ekf_timestamp : rospy.Time = None

    testing_config = rospy.get_param("~testing")
    if testing_config["is_mission_testing"]:
      self.is_mission_testing = True
      mission_test_id = testing_config["mission_test_id"]
      self.actions_list = lab_test.generate_actions(mission_id=mission_test_id)

      rospy.loginfo("Running mission-test with test-id: {}".format(mission_test_id))
    else:
      self.is_mission_testing = False 


  def __drone_state_cb(
        self, 
        msg: std_msgs.msg.String
      ) -> None:
    # Possible states:  ["FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"]
    #                   ["LANDED", "MOTOR_RAMPING", "TAKINGOFF", "HOVERING", "FLYING", "LANDING", "EMERGENCY"]
    flying_states = ["FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"]
    if msg.data not in flying_states:
      rospy.logdebug("Unknown status message: {}".format(msg.data))
      return

    if self.prev_drone_state != msg.data:
      self.prev_drone_state = self.drone_state
    self.drone_state = msg.data
    self.new_state_update = True


  def __drone_gnss_cb(
        self, 
        msg : sensor_msgs.msg.NavSatFix
      ) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.gnss_timestamp, msg_timestamp):
      # Old message
      return

    gnss_status = msg.status
    if gnss_status == -1:
      # No position fix
      return 

    latitude_deg = msg.latitude
    longitude_deg = msg.longitude 
    altitude_m = msg.altitude 

    # Convertion from long / lat / altitude to ECEF
    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    x, y, z = pyproj.transform(lla, ecef, longitude_deg, latitude_deg, altitude_m, radians=False)
    
    self.gnss_timestamp = msg_timestamp
    self.pos_ecef = np.array([x, y, z]).T


  def __ekf_cb(
        self, 
        msg : anafi_uav_msgs.msg.PointWithCovarianceStamped
      ) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos_relative_to_helipad = np.array([msg.position.x, msg.position.y, msg.position.z]).T # Is this really correct to say that this is the position relative to the helipad?


  def __set_planned_actions_srv(
        self, 
        request : SetPlannedActionsRequest
      ) -> SetPlannedActionsResponse:
    response = SetPlannedActionsResponse()

    if not self.is_mission_testing:
      rospy.loginfo("Updating actions to execute")
      num_actions = request.num_actions
      action_str_list = [""] * num_actions 

      for i in range(num_actions):
        action_str_list[i] = request.action_str_list[i]
        
      self.is_action_list_updated = True
      self.action_str_list = action_str_list
      self.is_ordered_to_cancel_current_action = request.cancel_current_action
    else:
      rospy.logwarn("Currently performing mission-testing. The actions from the planner will not be followed")
    response.success = True 

    return response


  def __get_action_data(self):
    if self.actions_list is None \
      or not self.actions_list:

      self.action_str = "idle"
      self.action_movement = np.zeros((4, 1))
    
    else:
      # Doing it inefficient, since deque would not remove leftmost element from popleft() 
      action = self.actions_list[0]
      action_list = []
      for idx in range(1, len(self.actions_list)):
        action_list.append(self.actions_list[idx])
      self.actions_list = action_list

      self.action_str = action.get_action_str()
      self.action_movement = action.get_action_movement().reshape((4, 1))

    self.max_expected_action_time_s = rospy.get_param("~maximum_expected_action_time_s")[self.action_str]


  def __get_next_action_function(self) -> Callable:    
    if self.action_str == "takeoff":
      rospy.loginfo("Next action: Takeoff")
      return self.__takeoff
    if self.action_str == "land":
      rospy.loginfo("Next action: Land")
      return self.__land
    if self.action_str == "track":
      rospy.loginfo("Next action: Track")
      return self.__track 
    if self.action_str == "search":
      rospy.loginfo("Next action: Search")
      return self.__search
    if self.action_str == "communicate":
      rospy.loginfo("Next action: Communicate")
      return self.__communicate
    if self.action_str == "travel_to":
      rospy.loginfo("Next action: Travel to")
      return self.__travel_to 
    if self.action_str == "move_relative":
      rospy.loginfo("Next action: Move relative")
      return self.__move_relative
    if self.action_str == "drop_bouy":
      rospy.loginfo("Next action: Drop bouya")
      return self.__drop_bouy
    rospy.loginfo("Next action: Idle")
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
      )(False)
      if not service_response.success:
        rospy.logerr("[__takeoff()] Cannot disable velocity-controller")
        raise ValueError()

      # Taking off
      self.takeoff_pub.publish(std_msgs.msg.Empty()) 

    except Exception as e:
      rospy.logerr("[__takeoff()] {} unavailable. Error {}".format(self.velocity_controller_service_name, e))


  def __land(self):
    rospy.loginfo("Trying to land")

    try:
      service_timeout = 0.5 # Short timeout might be problem due to power drain on sim... 
      
      # Disable the velocity controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.velocity_controller_service_name, 
        timeout=service_timeout 
      )(False)
      if not service_response.success:
        rospy.logerr("[__land()] Cannot disable velocity-controller")
        raise ValueError()

      # Land
      self.land_pub.publish(std_msgs.msg.Empty())

    except Exception as e:
      rospy.logerr("[__land()] {} unavailable. Error: {}".format(self.velocity_controller_service_name, e))


  def __track(self) -> None:
    rospy.loginfo("Trying to enable tracking of helipad")

    try:
      service_timeout = 2.0 
      
      # Enable the velocity controller
      service_response = self.__get_service_interface(
        service_type=SetBool,
        service_name=self.velocity_controller_service_name, 
        timeout=service_timeout 
      )(True)
      if not service_response.success:
        raise ValueError("[__track()] Cannot enable velocity-controller")

    except Exception as e:
      rospy.logerr("[__track()] {} unavailable. Error: {}".format(self.velocity_controller_service_name, e))
    
  
  def __search(self) -> None:
    rospy.loginfo("Trying to search an area")
    # Must use some trajectory planning to generate paths or trajectories for
    # the search in the desired area
  
  
  def __communicate(self) -> None:
    # Might be split into its own thread or similar
    rospy.loginfo("Communicating")
  

  def __travel_to(self) -> None:
    rospy.logwarn("Travelling to will maintain the pose, as there are uncertainties regarding the ECEF-pose")
    desired_ecef_coordinates = self.pos_ecef 

    # TODO: Must find a method for loading in the desired positions in NED or ECEF
    # desired_location = None 
    # ned_coordinates = self.__load_locations_positions_ned(location=desired_location)
    
    # if self.ned_origin_in_ECEF is None:
    #   rospy.logerr("No ECEF-coordinates are available for NED's origin")
    #   return 

    # desired_ecef_coordinates = ned_coordinates + self.ned_origin_in_ECEF
    # desired_ecef_coordinates = self.pos_ecef 

    # TODO: Should have some checks to prevent the drone from crashing into the water etc
    # Find a method for checking that ned_z > -1.0 or something, where -1.0 means that 
    # the drone is above the ground

    # The maximum speed could be made relative to the distance the system must travel

    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    lon, lat, alt = pyproj.transform(
      ecef, 
      lla, 
      desired_ecef_coordinates[0], 
      desired_ecef_coordinates[1], 
      desired_ecef_coordinates[2], 
      radians=False
    )

    move_to_ecef_cmd = olympe_bridge.msg.MoveToCommand()
    move_to_ecef_cmd.header.stamp = rospy.Time.now()
    move_to_ecef_cmd.latitude = lat
    move_to_ecef_cmd.longitude = lon 
    move_to_ecef_cmd.altitude = alt
    move_to_ecef_cmd.heading = 0          # Desired heading towards north
    move_to_ecef_cmd.orientation_mode = 2 # The drone will start to orient itself towards the target before moving

    self.move_to_ecef_pub.publish(move_to_ecef_cmd)


  def __move_relative(self) -> None:
    # TODO: Implement a check to prevent it from crashing into the ground
    move_relative_cmd = olympe_bridge.msg.MoveByCommand()
    move_relative_cmd.header.stamp = rospy.Time.now()
    move_relative_cmd.dx = self.action_movement[0]
    move_relative_cmd.dy = self.action_movement[1]
    move_relative_cmd.dz = self.action_movement[2]
    move_relative_cmd.dyaw = self.action_movement[3] # [rad] 
    
    self.move_relative_pub.publish(move_relative_cmd) 


  def __drop_bouy(self) -> None:
    rospy.loginfo("Dropping bouya")


  def __check_drone_state(
        self, 
        desired_state : str,
        start_time    : rospy.Time,
        current_count : int         = 0,
        min_count     : int         = 2,
        max_wait_time : float       = 5.0
      ) -> tuple:

    is_desired_state_reached = False
    duration = utilities.calculate_timestamp_difference_s(start_time, rospy.Time.now())

    if self.new_state_update:
      if self.drone_state == desired_state:
        current_count += 1
        if current_count >= min_count:
          is_desired_state_reached = True
      else:
        current_count = 0
    return (is_desired_state_reached, duration >= max_wait_time, current_count)

    

  def __load_locations_positions_ned(self, location : str) -> np.ndarray:
    # Load the locations to travel to
    # These should be loaded in either the ECEF-frame or NED-frame
    # return rospy.get_param("~locations_ned")[location]
    return np.zeros((3, 1))


  def __check_current_action_finished(
        self,
        start_time    : rospy.Time,
        current_count : int         
      ) -> tuple:
    """
    Check whether an action fulfills the criteria to be assumed 
    finished. Returns a tuple consisting of 
      - has_current_action_finished: bool
      - has_current_action_exceeded_max_time: bool
      - current_count: int

    A more stable method should be implemented

    Possible flying states as output using rosecho:
    "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"
    """

    if self.drone_state is None:
      rospy.logerr("Drone state not set!")
      return (False, False, 0)


    if self.action_str == "takeoff":
      return self.__check_drone_state(
        desired_state="FS_HOVERING",
        start_time=start_time,
        current_count=current_count,
        min_count=5,
        max_wait_time=self.max_expected_action_time_s
      )


    if self.action_str == "land":
      return self.__check_drone_state(
        desired_state="FS_LANDED",
        start_time=start_time,
        current_count=current_count,
        min_count=2,
        max_wait_time=self.max_expected_action_time_s
      )


    if self.action_str == "track":
      # Check that the error is small enough
      if self.pos_relative_to_helipad is None:
        # No input from the EKF received
        rospy.loginfo("EKF does not appear to be running. Exiting tracking-action")
        return (True, False, 0)
      # TODO: Add a position above the landing pad to achieve before landing
      horizontal_pos_error_normed = np.linalg.norm(self.pos_relative_to_helipad[:2])
      vertical_pos_error = self.pos_relative_to_helipad[2]
      
      horizontal_tracking_error_limit = rospy.get_param("~horizontal_tracking_error_limit", default=0.1)
      vertical_tracking_error_limit = rospy.get_param("~vertical_tracking_error_limit", default=0.2)

      is_drone_close_to_helipad = (
        (horizontal_pos_error_normed < horizontal_tracking_error_limit) 
        and 
        (np.abs(vertical_pos_error) < vertical_tracking_error_limit) 
      )
      print(horizontal_pos_error_normed)
      print(vertical_pos_error)
      print()

      # Obs! This should be kept over some time, such that it will not try to land 
      # if it receives a single faulty-measurement
      if is_drone_close_to_helipad:
        current_count += 1

      return (is_drone_close_to_helipad and current_count >= 3, False, current_count) 


    # The different actions are set as True, since these are not implemented
    if self.action_str == "search":
      return (True, False, 0)


    if self.action_str == "communicate":
      return (True, False, 0)


    if self.action_str == "travel_to":
      # Check whether the drone is close enough to the desired point of interest
      # May become slightly problematic when travelling to the platform, as the error might be 
      # too large for the tracking to handle

      # This function is currently intended for moving with respect to a larger area
      horizontal_radius_of_acceptance = rospy.get_param("~horizontal_radius_of_acceptance", default=1.0)
      vertical_radius_of_acceptance = rospy.get_param("~vertical_radius_of_acceptance", default=2.0)
      
      if self.desired_ecef_coordinates is None:
        rospy.logerr("[__check_current_action_finished()] Desired ECEF coordinates are None")
        return (False, False, 0)

      pos_error_ecef = self.desired_ecef_coordinates - self.pos_ecef
      return np.linalg.norm(pos_error_ecef[:2]) <= horizontal_radius_of_acceptance \
              and np.abs(pos_error_ecef) <= vertical_radius_of_acceptance


    if self.action_str == "move_relative":
      # This might return to quickly if not checking that it has been flying
      # Edge-case in case relative movement is zero
      # But at the same time, 5 counts may be enough...
      # if self.prev_drone_state != "FS_FLYING":
      #   return (False, False, 0)
      return self.__check_drone_state(
        desired_state="FS_HOVERING",
        start_time=start_time,
        current_count=current_count,
        min_count=5,
        max_wait_time=self.max_expected_action_time_s
      )
    

    if self.action_str == "drop_bouy":
      return (True, False, 0)


    return (False, False, 0)


  def __cancel_action(self) -> None:
    """
    Bit unsure how to develop this correctly, as there are multiple 
    complexities which will be linked to each case
    """
    rospy.logerr("Cancelling of actions are not implemented")


  def execute_actions(self) -> None:
    """
    Things to take into account:
    1. What should happen if a new action sequence is received?
        Should the current action be cancelled or should it continue?
    2. What should occur if actions are taking too long? Should they 
        be preempted / cancelled? 
    3. How to include information about f(Mission):included in the message?

    Things that could cause an action to be cancelled:
    Takes too long
    Ordered from the planner that the current action must be aborted
    Current action has finished

    But what is going to happen if a new set of requested actions are received?
    Should one continue to finish the current action, or should one just start 
    with the new actions? Could be some mediation between the planner and the 
    executor 

    Have a discussion with Simen about this, because from a planning perspective
    there will be situations when the system should be replanned
    """

    rospy.loginfo("Initializing")

    self.__get_action_data()
    self.__get_next_action_function()()
    start_time = rospy.Time.now()

    relative_movement_ordered : np.ndarray = np.zeros((3, 1))
    gnss_position_initialized : bool = False
    action_finished_counts = 0
    
    while not rospy.is_shutdown():
      if not gnss_position_initialized:
        # Check if the position has been updated
        if self.pos_ecef is not None:
          # This is only intendended as a rough estimate for longer missions
          # or missions where the EKF estimate will drift
          self.ned_origin_in_ECEF = self.pos_ecef - relative_movement_ordered # TODO: Check that this becomes accurate enough
          gnss_position_initialized = True

      check_finished = self.__check_current_action_finished(
        start_time=start_time,
        current_count=action_finished_counts
      )
      is_current_action_finished : bool = check_finished[0]
      has_action_execution_exceeded_limit : bool = check_finished[1]
      action_finished_counts : int = check_finished[2]

      if is_current_action_finished: 
        # Finished with an action - start the next action
        rospy.loginfo("Drone finished with action: {}".format(self.action_str))
        
        self.__get_action_data()
        relative_movement_ordered = relative_movement_ordered + self.action_movement[:3] # TODO: Bug! Must take yaw into account

        self.__get_next_action_function()()
        start_time = rospy.Time.now()
        action_finished_counts = 0

      elif has_action_execution_exceeded_limit or self.is_ordered_to_cancel_current_action:
        
        # TODO: Move this into own function(s)
        rospy.loginfo("{}. Has exceeded expected time limit: {} [s]".format(self.action_str, self.max_expected_action_time_s))
        
        if self.action_str == "takeoff":
          # Retry takeoff
          rospy.loginfo("Retrying takeoff")
          self.__takeoff()
        
        if self.action_str == "land" and self.__check_current_action_finished(start_time, action_finished_counts)[0]:
          # Retry landing
          rospy.loginfo("Retrying landing")
          self.__land()  
        elif self.drone_state in ["FS_FLYING", "FS_HOVERING"]:
          # Try to track helipad
          self.action_str = "track"
          self.__track()

        if self.action_str == "move_relative" and self.drone_state == "FS_FLYING":
          # Might be stuck in the state "FS_FLYING" if only controlling the altitude (on the sim, atleast)
          rospy.loginfo("Trying to hover")
          self.action_movement = np.zeros((4, 1))
          self.__move_relative()

        # Must determine a method for cancelling actions'
        # Only a small subset of actions are possible to cancel, and 
        # would require some complex logic to get it right

        # self.__cancel_action()

        # self.__get_next_action_function()()
        start_time = rospy.Time.now() # Currently just restart the timer to prevent being spammed with messages
        # action_finished_counts = 0
      
      # else:
      #   rospy.loginfo("Drone performing action: {}. Count achieved: {}".format(self.action_str, action_finished_counts))

      self.rate.sleep()


def main():
  mission_executor_node = MissionExecutorNode()
  mission_executor_node.execute_actions()

if __name__ == "__main__":
  main()
