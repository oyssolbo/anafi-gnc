#!/usr/bin/python3

from locale import currency
from turtle import pos
import rospy
import rospkg

import pymap3d
import numpy as np

from collections import deque
from typing import Callable

import anafi_uav_msgs.msg 
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsRequest, SetPlannedActionsResponse
import olympe_bridge.msg 

import std_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import SetBool

import mission_executor_helpers.utilities as utilities
import simple_missions.lab_test as lab_test

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
    rospy.Subscriber("/anafi/state", std_msgs.msg.String, self._drone_state_cb) 
    rospy.Subscriber("/anafi/ned_pose_from_gnss", geometry_msgs.msg.PointStamped, self._drone_ned_pose_from_gnss_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.PointWithCovarianceStamped, self._ekf_cb)

    # Setup services
    rospy.Service("/mission_executor/service/set_planned_actions", SetPlannedActions, self._set_planned_actions_srv)

    # Setup publishers
    self.takeoff_pub = rospy.Publisher("/anafi/cmd_takeoff", std_msgs.msg.Empty, queue_size=1)
    self.land_pub = rospy.Publisher("/anafi/cmd_land", std_msgs.msg.Empty, queue_size=1)

    self.move_to_pub = rospy.Publisher("/anafi/cmd_moveto", olympe_bridge.msg.MoveToCommand, queue_size=1)
    self.move_to_ned_pos_pub = rospy.Publisher("/anafi/cmd_moveto_ned_position", geometry_msgs.msg.PointStamped, queue_size=1)
    self.move_relative_pub = rospy.Publisher("/anafi/cmd_moveby", olympe_bridge.msg.MoveByCommand, queue_size=1)

    # Services and actions to connect to
    selected_control_method = rospy.get_param("/control_method")
    if selected_control_method == "mpc":
      self.controller_service_name = "/mpc/service/enable_controller"
    else:
      self.controller_service_name = "/velocity_controller/service/enable_controller"

    # Initializing values
    self.pos_ned_gnss : np.ndarray = None 
    self.expected_platform_ned_pos : np.ndarray = None       
    self.pos_relative_to_helipad : np.ndarray = None
    self.desired_ned_pos : np.ndarray = None 
    self.search_points_ned : np.ndarray = np.zeros((0,0))

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


  def _drone_state_cb(
        self, 
        msg: std_msgs.msg.String
      ) -> None:
    # Possible states:  ["FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"]
    #                   ["LANDED", "MOTOR_RAMPING", "TAKINGOFF", "HOVERING", "FLYING", "LANDING", "EMERGENCY"]
    flying_states = ["FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"]
    if msg.data not in flying_states:
      # rospy.logdebug("Unknown status message: {}".format(msg.data))
      return

    if self.prev_drone_state != msg.data:
      self.prev_drone_state = self.drone_state
    self.drone_state = msg.data
    self.new_state_update = True


  def _drone_ned_pose_from_gnss_cb(
        self, 
        msg : geometry_msgs.msg.PointStamped
      ) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.gnss_timestamp, msg_timestamp):
      # Old message
      return
    
    self.gnss_timestamp = msg_timestamp
    self.pos_ned_gnss = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float).reshape((3, 1)) 


  def _ekf_cb(
        self, 
        msg : anafi_uav_msgs.msg.PointWithCovarianceStamped
      ) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.ekf_timestamp, msg_timestamp):
      # Old message
      return
    
    self.ekf_timestamp = msg_timestamp
    self.pos_relative_to_helipad = -np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float).reshape((3, 1)) 


  def _set_planned_actions_srv(
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


  def _get_action_data(self):
    if self.actions_list is None \
      or not self.actions_list:

      self.action_str = "idle"
      self.action_movement = np.zeros((4, 1))
      self.action_desired_time = -1
    
    else:
      # Doing it inefficient, since deque would not remove leftmost element from popleft() 
      action = self.actions_list[0]
      action_list = []
      for idx in range(1, len(self.actions_list)):
        action_list.append(self.actions_list[idx])
      self.actions_list = action_list

      self.action_str = action.get_action_str()
      self.action_movement = action.get_action_movement().reshape((4, 1))
      self.action_desired_time = action.get_action_time()

    self.max_expected_action_time_s = rospy.get_param("~maximum_expected_action_time_s")[self.action_str]


  def _get_next_action_function(self) -> Callable:    
    if self.action_str == "takeoff":
      rospy.loginfo("Next action: Takeoff")
      return self._takeoff
    if self.action_str == "land":
      rospy.loginfo("Next action: Land")
      return self._land
    if self.action_str == "track":
      rospy.loginfo("Next action: Track")
      return self._track 
    if self.action_str == "search":
      rospy.loginfo("Next action: Search")
      self._initialize_search()
      return self._search_positions
    if self.action_str == "communicate":
      rospy.loginfo("Next action: Communicate")
      return self._communicate
    if self.action_str == "travel_to":
      rospy.loginfo("Next action: Travel to")
      return self._travel_to 
    if self.action_str == "move_relative":
      rospy.loginfo("Next action: Move relative")
      return self._move_relative
    if self.action_str == "drop_bouy":
      rospy.loginfo("Next action: Drop bouya")
      return self._drop_bouy
    if self.action_str == "hover":
      rospy.loginfo("Next action: Hover")
      return self._hover
    rospy.loginfo("Next action: Idle")
    return self._idle
    

  def _get_service_interface(
        self,
        service_type,
        service_name  : str,
        timeout       : float = 2.0
    ):
    rospy.wait_for_service(service_name, timeout=timeout)
    return rospy.ServiceProxy(service_name, service_type)


  def _initialize(self) -> bool:
    return True


  def _idle(self):
    rospy.loginfo("Idling")


  def _takeoff(self):
    rospy.loginfo("Trying to take off")

    try:
      service_timeout = 2.0
      
      # Disable the controller to have a predefined state
      service_response = self._get_service_interface(
        service_type=SetBool,
        service_name=self.controller_service_name,
        timeout=service_timeout 
      )(False)
      if not service_response.success:
        rospy.logerr("[_takeoff()] Cannot disable controller")
        raise ValueError()

      # Taking off
      self.takeoff_pub.publish(std_msgs.msg.Empty()) 

    except Exception as e:
      rospy.logerr("[_takeoff()] {} unavailable. Error {}".format(self.controller_service_name, e))


  def _land(self):
    rospy.loginfo("Trying to land")

    try:
      service_timeout = 0.5 # Short timeout might be problem due to power drain when running the perception nodes...
      
      # Disable the controller
      service_response = self._get_service_interface(
        service_type=SetBool,
        service_name=self.controller_service_name, 
        timeout=service_timeout 
      )(False)
      if not service_response.success:
        rospy.logerr("[_land()] Cannot disable controller")
        raise ValueError()

      # Land
      self.land_pub.publish(std_msgs.msg.Empty())

    except Exception as e:
      rospy.logerr("[_land()] {} unavailable. Error: {}".format(self.controller_service_name, e))


  def _track(self) -> None:
    rospy.loginfo("Trying to enable tracking of helipad")

    try:
      service_timeout = 2.0 
      
      # Enable the controller
      service_response = self._get_service_interface(
        service_type=SetBool,
        service_name=self.controller_service_name, 
        timeout=service_timeout 
      )(True)
      if not service_response.success:
        raise ValueError("[_track()] Cannot enable controller")

    except Exception as e:
      rospy.logerr("[_track()] {} unavailable. Error: {}".format(self.controller_service_name, e))
    
  
  def _return_to_expected_home(self) -> None:
    if self.expected_platform_ned_pos is None:
      # No GNSS measurements
      # Assume that the platform error is sufficiently small, such that 
      # a search will detect it
      estimated_platform_ned_pos = np.zeros((3, 1))
    else:
      estimated_platform_ned_pos = self.expected_platform_ned_pos
    self._travel_to_ned_position(estimated_platform_ned_pos, accuracy_required=False)


  def _initialize_search(self, target_str : str = "helipad") -> None: # TODO: Add the initial search_position
    rospy.loginfo("Trying to search an area for target {}".format(target_str))

    if self.search_points_ned.size > 0:
      warn_message_str = "Previous points are marked to be searched. This includes:\n"
      for col in range(self.search_points_ned.shape[1]):
        warn_message_str += str(self.search_points_ned[0, col]) + "," + str(self.search_points_ned[1, col]) + "\n"
      warn_message_str += "\nThese positions will be deleted as the search-initialization is currently implemented!"
      rospy.logwarn(warn_message_str)

    # Hardcoded values
    search_side_length = 0.5 # [m]
    search_altitude = -2.0 # NOTE: I have no clue on the optimal altitude for searching... Discuss this with Simen
    num_sides = 12
    
    search_points_ned : np.ndarray = np.zeros((3, num_sides))
    self.search_target : str = target_str

    if self.pos_ned_gnss is None:
      initial_pos = np.zeros((3, 1))
    else:
      initial_pos = self.pos_ned_gnss

    search_points_ned[:, 0] = initial_pos.ravel()

    length_multiplier = 1.0
    for side_idx in range(1, num_sides):
      current_x_pos = search_points_ned[0, side_idx - 1]
      current_y_pos = search_points_ned[1, side_idx - 1] 
 
      if side_idx % 2 == 1:
        next_x = current_x_pos + length_multiplier * search_side_length
        next_y = current_y_pos
      else:
        next_x = current_x_pos
        next_y = current_y_pos + length_multiplier * search_side_length

        # Multiplication to achieve the expanding square search
        length_multiplier = length_multiplier * (-2.0)

      next_pos = [next_x, next_y, search_altitude]
      search_points_ned[:, side_idx] = np.array(next_pos, dtype=np.float).reshape((3, 1)).ravel()

    self.search_points_ned = search_points_ned
     

  def _search_positions(self) -> bool:
    """
    This is just a very simple expanding squares search. See IAMSAR manual: Vol. 2: Mission co-ordination
    for more details
    """
    if not self.search_points_ned.size:
      rospy.loginfo("No positions to search")
      return True
    
    target_pos = self.search_points_ned[:,0]
    if self.pos_ned_gnss is None:
      current_pos = np.zeros((3, 1))
    else:
      current_pos = self.pos_ned_gnss

    horizontal_distance = np.linalg.norm((target_pos - current_pos.T)[:2])
    vertical_distance = np.abs((target_pos - current_pos.T)[0,2])
    if horizontal_distance <= 0.5 and vertical_distance <= 0.5:
      # Close enough - travel to the next point of interest
      # Mask out the first position
      mask = np.ones(self.search_points_ned.shape[1], dtype=bool)
      mask[0] = False
      self.search_points_ned = self.search_points_ned[:,mask] 

      if not self.search_points_ned.size:
        rospy.loginfo("All areas searched...")
        return True
      
      target_pos = self.search_points_ned[:,0]
        
    target_pos = self.search_points_ned[:,0]
    self._travel_to_ned_position(target_pos, accuracy_required=True)

    return False
  
  
  def _communicate(self) -> None:
    # Might be split into its own thread or similar
    rospy.loginfo("Communicating")
  

  def _travel_to_ned_position(
        self, 
        position_ned      : np.ndarray, 
        accuracy_required : bool        = False
      ) -> None:
    """
    Using the internal controllers as they are more optimized. 
    If the drone is too quick, it might be possible to enable the internal controllers.
    """

    if self.pos_ned_gnss is None:
      rospy.logerr("Global position estimate is currently not received. Will maintain pose")
      return

    if np.array_equal(self.desired_ned_pos, position_ned):
      # Cannot send the command too often. That will cause the Parrot Olympe to cancel the
      # previous command which is sent
      rospy.logdebug_throttle(1, "[_travel_to_ned_position()] Previous desired position is equial to new desired position. Aborting")
      return 

    if position_ned[2] > -1.5: # This might be scary if operating over water, due to high ambiguity in vertical positioning  
      # desired_altitude = -5 # NOTE: This can be really dangerous if flying inside the drone-lab! Must be run outside or in the sim
      desired_altitude = -2 
      rospy.logerr_throttle(1, "Desired position given with an altitude of {} m. Will move in x={} and y={} but keep an altitude of {} m".format(
          -position_ned[2],
          position_ned[0],
          position_ned[1],
          -desired_altitude
        )
      )
    else:
      desired_altitude = position_ned[2]

    if accuracy_required:
      rospy.logwarn_throttle(1, "Must use our own controllers for steering the drone. The builtin functionalities from Parrot are not accurate enough...")
      # TODO: Implement some functionality to use our own controllers

    else:
      rospy.loginfo_throttle(1, "Using the builtin controllers from Parrot to move to position. These might be somewhat inaccurate...")
      ned_point = geometry_msgs.msg.Point()
      ned_point.x = position_ned[0]
      ned_point.y = position_ned[1]
      ned_point.z = desired_altitude

      ned_point_msg = geometry_msgs.msg.PointStamped()
      ned_point_msg.header.stamp = rospy.Time.now()
      ned_point_msg.point = ned_point

      self.move_to_ned_pos_pub.publish(ned_point_msg)


  def _move_relative(self) -> None:
    # TODO: Implement a check to prevent it from crashing into the ground
    move_relative_cmd = olympe_bridge.msg.MoveByCommand()
    move_relative_cmd.header.stamp = rospy.Time.now()
    move_relative_cmd.dx = self.action_movement[0]
    move_relative_cmd.dy = self.action_movement[1]
    move_relative_cmd.dz = self.action_movement[2]
    move_relative_cmd.dyaw = self.action_movement[3] # [rad] 
    
    self.move_relative_pub.publish(move_relative_cmd) 


  def _hover(self) -> None:
    if self.drone_state in ["FS_LANDING", "FS_LANDED"]:
      # Not try to hover if the drone is trying to land
      rospy.logerr("Drone trying to land. Hovering is unavailable")
      return 
    
    rospy.loginfo_throttle(1, "Trying to hover")
    self.action_movement = np.zeros((4, 1))
    self._move_relative()


  def _drop_bouy(self) -> None:
    rospy.logwarn("Dropping bouya not implemented")


  def _check_drone_state(
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

    
  def _load_locations_positions_ned(self, location : str) -> np.ndarray:
    # Load the locations to travel to
    # These should be loaded in NED-frame
    # return rospy.get_param("~locations_ned")[location]
    return np.zeros((3, 1))


  def _check_current_action_finished(
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
      rospy.logerr_throttle(5, "Drone state not set! Check connection to the bridge, and that data is published on topic '/anafi/state'")
      return (False, False, 0)


    if self.action_str == "takeoff":
      return self._check_drone_state(
        desired_state="FS_HOVERING",
        start_time=start_time,
        current_count=current_count,
        min_count=5,
        max_wait_time=self.max_expected_action_time_s
      )


    if self.action_str == "land":
      return self._check_drone_state(
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
        rospy.logerr("EKF does not appear to be running. Exiting tracking-action")
        return (True, False, 0)

      # TODO: Add a position above the landing pad to achieve before landing
      horizontal_pos_error_normed = np.linalg.norm(self.pos_relative_to_helipad[:2])
      vertical_pos_error = self.pos_relative_to_helipad[2]
      
      horizontal_tracking_error_limit = rospy.get_param("~horizontal_tracking_error_limit", default=0.05)
      vertical_tracking_error_limit = rospy.get_param("~vertical_tracking_error_limit", default=0.9) # NOTE: 0.9 for the simulator. Get this from launch-file or something

      # Check whether the drone is close enough to the helipad
      is_drone_close_to_helipad = (
        (horizontal_pos_error_normed < horizontal_tracking_error_limit) 
        and 
        (np.abs(vertical_pos_error) < vertical_tracking_error_limit) 
      )

      # Check whether the horizontal velocity is low enough, such that it does not try
      # to land if it suddenly gets high velocities in one direction
      # One might consider using the attitude, but that might cause problems if the drone 
      # must counteract disturbances from e.g. wind 
      is_velocity_low_enough = True # TODO

      is_drone_ready_to_land = (is_drone_close_to_helipad and is_velocity_low_enough)

      if is_drone_ready_to_land:
        current_count += 1
      else:
        current_count = 0

      return (is_drone_ready_to_land and current_count >= 3, False, current_count) 


    if self.action_str == "search":
      # Must get an update from the perception-module whether the object of interest has been detected
      searched_current_position = self._search_positions()

      is_object_found = False # Unsure how to use this information as of now...
                              # Currently just assuming that the search will stop if found anything, however
                              # it may be better to continue the search afterwards 

      is_search_finished = (searched_current_position and self.search_points_ned.size == 0) or is_object_found

      return (is_search_finished, (rospy.Time.now() - start_time).to_sec() >= self.max_expected_action_time_s, 0)


    if self.action_str == "communicate":
      return (True, False, 0)


    if self.action_str == "travel_to":
      # Check whether the drone is close enough to the desired point of interest
      # May become slightly problematic when travelling to the platform, as the error might be 
      # too large for the tracking to handle

      # This function is currently intended for moving with respect to a larger area
      horizontal_radius_of_acceptance = rospy.get_param("~horizontal_radius_of_acceptance", default=1.0)
      vertical_radius_of_acceptance = rospy.get_param("~vertical_radius_of_acceptance", default=2.0)
      
      if self.desired_ned_pos is None:
        rospy.logerr_throttle(1, "[_check_current_action_finished()] Desired NED position isre None")
        return (False, False, 0)

      pos_error_ned = self.desired_ned_pos - self.pos_ned_gnss
      return np.linalg.norm(pos_error_ned[:2]) <= horizontal_radius_of_acceptance \
              and np.abs(pos_error_ned) <= vertical_radius_of_acceptance


    if self.action_str == "move_relative":
      # This might return to quickly if not checking that it has been flying
      # Edge-case in case relative movement is zero
      # But at the same time, 5 counts may be enough...
      # if self.prev_drone_state != "FS_FLYING":
      #   return (False, False, 0)
      return self._check_drone_state(
        desired_state="FS_HOVERING",
        start_time=start_time,
        current_count=current_count,
        min_count=5,
        max_wait_time=self.max_expected_action_time_s
      )
    

    if self.action_str == "drop_bouy":
      return (True, False, 0)


    if self.action_str == "hover":
      drone_state = self._check_drone_state(
        desired_state="FS_HOVERING",
        start_time=start_time,
        current_count=current_count,
        min_count=5,
        max_wait_time=self.max_expected_action_time_s
      )
      if not drone_state[0]:
        self._hover()
      return (drone_state[0], False, drone_state[2])


    return (False, False, 0)


  def _cancel_action(self) -> None:
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
    3. How to include information about mission position?

    Things that could cause an action to be cancelled:
    Takes too long
    Ordered from the planner that the current action must be aborted

    But what is going to happen if a new set of requested actions are received?
    Should one continue to finish the current action, or should one just start 
    with the new actions? Could be some mediation between the planner and the 
    executor 
    """
    self._get_action_data()
    self._get_next_action_function()()
    start_time = rospy.Time.now()

    relative_movement_ordered : np.ndarray = np.zeros((3, 1))
    ned_position_initialized : bool = False
    action_finished_counts = 0
    
    while not rospy.is_shutdown():
      if not ned_position_initialized:
        # Check if the position has been updated
        if self.pos_ned_gnss is not None:
          # This is only intendended as a rough estimate for missions where the 
          # GNSS-estimate is not received immediately at startup
          self.expected_platform_ned_pos = self.pos_ned_gnss - relative_movement_ordered # TODO: Check that this becomes accurate enough
          ned_position_initialized = True

          ned_origin_info_str = "NED-frame is initialized with respect to the initial GNSS-message.\
          \nPosition of helipad is expected to be at [{}, {}, {}] in the NED-frame.".format(
            self.expected_platform_ned_pos[0],
            self.expected_platform_ned_pos[1],
            self.expected_platform_ned_pos[2]
            )
          rospy.logwarn(ned_origin_info_str)

      check_finished = self._check_current_action_finished(
        start_time=start_time,
        current_count=action_finished_counts
      )
      is_current_action_finished : bool = check_finished[0]
      has_action_execution_exceeded_limit : bool = check_finished[1]
      action_finished_counts : int = check_finished[2]

      if is_current_action_finished: 
        # Finished with an action - start the next action
        rospy.loginfo("Drone finished with action: {}".format(self.action_str))
        
        self._get_action_data()
        relative_movement_ordered = relative_movement_ordered + self.action_movement[:3] # TODO: Bug! Does not account for yaw 

        self._get_next_action_function()()
        start_time = rospy.Time.now()
        action_finished_counts = 0

      elif has_action_execution_exceeded_limit:
        
        # TODO: Move this into own function(s)
        rospy.loginfo("{}. Has exceeded expected time limit: {} [s]".format(self.action_str, self.max_expected_action_time_s))
        
        if self.action_str == "takeoff" and self.drone_state not in ["FS_MOTOR_RAMPING", "FS_TAKINGOFF"]:
          # Retry takeoff
          rospy.loginfo("Retrying takeoff")
          self._takeoff()
        
        elif self.action_str == "land" and self.drone_state not in ["FS_LANDING"]:
          # Retry landing
          rospy.loginfo("Retrying landing")
          self._land()  

        elif self.action_str == "move_relative" and self.drone_state == "FS_FLYING":
          # Might be stuck in the state "FS_FLYING" by some reason
          self._hover()

        start_time = rospy.Time.now() 

      elif self.is_ordered_to_cancel_current_action:
        # May want to have some logic for cancelling actions

        # Must determine a method for cancelling actions'
        # Only a small subset of actions are possible to cancel, and 
        # would require some complex logic to get it right

        # self._cancel_action()

        # self._get_next_action_function()()
        # start_time = rospy.Time.now()
        pass 

      self.rate.sleep()


def main():
  mission_executor_node = MissionExecutorNode()
  # mission_executor_node.test_functions()
  mission_executor_node.execute_actions()
  

if __name__ == "__main__":
  main()
