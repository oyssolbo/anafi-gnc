#!/usr/bin/python3

import rospy 
import numpy as np
from geometry_msgs.msg import TwistStamped


class GenerateVelocityErrors():
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_velocity_error_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.velocity_pub = rospy.Publisher("/anafi/polled_body_velocities", TwistStamped, queue_size=1)   
 
    # Set up private variables
    self.time_between_switch : float = 0.25

  def generate_velocity_errors(self) -> None:
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
      if (rospy.Time.now() - start_time).to_sec() > self.time_between_switch:
        point_msg = TwistStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.twist.linear.x = 2
        point_msg.twist.linear.y = -2
        point_msg.twist.linear.z = 1
        self.velocity_pub.publish(point_msg)

        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  generate_velocity_errors = GenerateVelocityErrors()
  generate_velocity_errors.generate_velocity_errors()


if __name__ == "__main__":
  main()
