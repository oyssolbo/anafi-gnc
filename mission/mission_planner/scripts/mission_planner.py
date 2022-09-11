#!/usr/bin/python3

import rospy

import numpy as np

import anafi_uav_msgs.msg
from anafi_uav_msgs.srv import SetPlannedActions, SetPlannedActionsResponse

import mission_planner_helpers.utilities as utilities

class MissionPlannerNode():
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default="mission_planner_node")
    node_rate = rospy.get_param("~node_rate", default=1)

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup subscribers
    rospy.Subscriber("/drone/out/telemetry", anafi_uav_msgs.msg.AnafiTelemetry, self.__drone_telemetry_cb)
    rospy.Subscriber("/estimate/ekf", anafi_uav_msgs.msg.PointWithCovarianceStamped, self.__ekf_cb)

    self.last_telemetry_msg : anafi_uav_msgs.msg.AnafiTelemetry = None 
    self.pos : np.ndarray = None 


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


  def plan_continously(self):
    service_name = "/mission_executor/service/set_planned_actions"
    action_plan_list = ["takeoff", "track", "land"]
    is_plan_updated = True 

    while not rospy.is_shutdown():
      # Get an update of the current state of the drone TODO

      # Replan or develop plans taking deviations into account TODO

      # Decide if the current plan must be cancelled TODO




      if is_plan_updated:
        try:
          msg = SetPlannedActions()
          for action in action_plan_list:
            msg.action_list.append(action)
          msg.num_actions = len(action_plan_list)
          msg.cancel_current_action = False 

          rospy.wait_for_service(service_name, timeout=2.0)
          service = rospy.ServiceProxy(service_name, SetPlannedActions)

          service_response = service(msg)
          if not service_response.success:
            rospy.loginfo("[plan_continously()] {} failed to set actions".format(service_name))
          else:
            rospy.logerror("[plan_continously()] {} updated with new set of actions".format(service_name))
            print("fuck python")
            is_plan_updated = False

        except:
          rospy.logerr("[plan_continously()] {} unavailable".format(service_name))

      self.rate.sleep()
      


def main():
  mission_planner_node = MissionPlannerNode()
  mission_planner_node.plan_continously()

if __name__ == '__main__':
  main()
