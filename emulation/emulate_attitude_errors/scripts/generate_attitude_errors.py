#!/usr/bin/python3

import rospy 
import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import QuaternionStamped

class GenerateAttitudeErrors():
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_attitude_error_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.attitude_error_pub = rospy.Publisher("/anafi/attitude", QuaternionStamped, queue_size=1)   
 
    # Set up private variables
    self.time_between_switch : float = 0.25

  def generate_attitude_errors(self) -> None:
    start_time = rospy.Time.now()
    roll_list = [5, 0, -5, 0]
    pitch_list = [0, 5, 0, -5]
    yaw_list = [0, 25, 0, -45]
    i = 0

    while not rospy.is_shutdown():
      if (rospy.Time.now() - start_time).to_sec() > self.time_between_switch:
        roll = 10 #roll_list[i]
        pitch = 5# pitch_list[i]
        yaw = 0 #yaw_list[i]

        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True)
        quaternion = rot.as_quat()

        attitude_msg = QuaternionStamped()
        attitude_msg.header.stamp = rospy.Time.now()
        attitude_msg.quaternion.x = quaternion[0] 
        attitude_msg.quaternion.y = quaternion[1] 
        attitude_msg.quaternion.z = quaternion[2]
        attitude_msg.quaternion.w = quaternion[3] 
        self.attitude_error_pub.publish(attitude_msg)

        i += 1
        if i >= len(roll_list):
          i = 0

        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  generate_position_errors = GenerateAttitudeErrors()
  generate_position_errors.generate_attitude_errors()


if __name__ == "__main__":
  main()
