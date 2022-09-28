#!/usr/bin/python3

import rospy

import numpy as np

import anafi_uav_msgs.msg
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsRequest
import std_msgs.msg

import mission_planner_helpers.utilities as utilities

class MissionPlannerNode():
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default="mission_planner_node")
    node_rate = rospy.get_param("~node_rate", default=1)

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup subscribers
    rospy.Subscriber("/anafi/state", std_msgs.msg.String, self.__drone_state_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.PointWithCovarianceStamped, self.__ekf_cb)

    self.new_drone_state_available : bool = False 
    self.pos : np.ndarray = None 


  def __drone_state_cb(
        self, 
        msg: std_msgs.msg.String
      ) -> None:
    self.drone_state = msg.data
    self.new_drone_state_available = True


  def __ekf_cb(
        self, 
        msg: anafi_uav_msgs.msg.PointWithCovarianceStamped
      ) -> None:
    self.pos = np.array([
      msg.position.x,
      msg.position.y,
      msg.position.z
    ])


  def plan_continously(self):
    # Simple test-case before future development
    service_name = "/mission_executor/service/set_planned_actions"
    action_plan_list = ["takeoff", "track", "land"]
    is_plan_updated = True 

    while not rospy.is_shutdown():
      # Get an update of the current state of the drone TODO

      # Replan or develop plans taking deviations into account TODO

      # Decide if the current plan must be cancelled TODO

      if is_plan_updated:
        try:
          request = SetPlannedActionsRequest()
          request.action_list = action_plan_list
          request.num_actions = len(action_plan_list)
          request.cancel_current_action = False 

          rospy.wait_for_service(service_name, timeout=2.0)
          service = rospy.ServiceProxy(service_name, SetPlannedActions)

          service_response = service(request)
          if not service_response.success:
            rospy.loginfo("[plan_continously()] {} failed to set actions".format(service_name))
          else:
            rospy.loginfo("[plan_continously()] {} updated with new set of actions".format(service_name))
            is_plan_updated = False

        except rospy.ServiceException as e:
          rospy.logerr("[plan_continously()] {} unavailable. Error: {}".format(service_name, e.what()))
        except rospy.ROSException as e:
          rospy.logerr("[plan_continously()] {} ROSExpection occured. Error: {}".format(service_name, e))

      self.rate.sleep()
      

def main():
  mission_planner_node = MissionPlannerNode()
  mission_planner_node.plan_continously()

if __name__ == '__main__':
  main()
