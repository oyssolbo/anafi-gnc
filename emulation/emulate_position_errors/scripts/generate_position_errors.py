#!/usr/bin/python3

import rospy 
import numpy as np
from anafi_uav_msgs.msg import PointWithCovarianceStamped


class GeneratePositionErrors():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_position_error_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.position_error_pub = rospy.Publisher("/estimate/ekf", PointWithCovarianceStamped, queue_size=1)   
 
    # Set up private variables
    self.time_between_switch : float = 5

  def generate_position_errors(self) -> None:
    start_time = rospy.Time.now()
    xs = [0, 1, 0, -1, 0]
    ys = [0, 0, 1, 0, -1]
    # xs = [0, -1, 1, -1, 1]
    # ys = [0] * 5
    zs = [0, 1, -1, 1, -1]
    zs = [0, 0.25, -0.25, 0.25, -0.25]
    i = 0

    x = 0
    y = 0
    z = 0
    while not rospy.is_shutdown():
      if (rospy.Time.now() - start_time).to_sec() > self.time_between_switch:
        point_msg = PointWithCovarianceStamped()
        # Testing with pure-pursuit 
        # wrong sign in y

        # Testing with pid guidance
        # wrong sign in y
        # wrong sign in x

        """
        Testing in the simulator shows that the EKF generates position estimates of 
        the platform relative to the drone. 

          x > 0, means the drone is 'behind' the platform. x > 0 should induce a forward movement ()
          y > 0, means the drone is to the left of the platform. y > 0 should induce a movement to the right
          z > 0, means the drone is above the platform. z > 0 should induce a downward movement
        """
        point_msg.header.stamp = rospy.Time.now()
        point_msg.position.x = 1 #xs[i]# np.random.normal(0.0, 0.25)
        point_msg.position.y = 1# ys[i]# np.random.normal(0.0, 0.25)
        point_msg.position.z = 0#zs[i]#zs[i]#np.random.normal(0.0, 0.1)
        self.position_error_pub.publish(point_msg)
        
        # x = x + self.time_between_switch * np.random.normal(0.0, 0.05) 
        # y = y + self.time_between_switch * np.random.normal(0.0, 0.05) 
        # z = z + self.time_between_switch * np.random.normal(0.0, 0.01)

        i += 1
        if i >= len(xs):
          i = 0

        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  generate_position_errors = GeneratePositionErrors()
  generate_position_errors.generate_position_errors()


if __name__ == "__main__":
  main()
