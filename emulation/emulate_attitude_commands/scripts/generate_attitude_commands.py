#!/usr/bin/python3

import rospy 
import numpy as np
from olympe_bridge.msg import AttitudeCommand


class GenerateAttitudeCommands():
  """
  Guidance law generating the desired velocity based on the 
  desired and current position 
  """
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_attitude_commands_node")
    controller_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.attitude_command_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)   
 
    # Set up private variables
    self.time_between_switch : float = 2

    # First order reference model for roll and pitch
    self.tau_attitude = 0.1
    self.Ad = -np.eye(2)
    self.Bd = np.eye((2))

  def get_filtered_reference(
        self, 
        xd_prev : np.ndarray, 
        x_ref   : np.ndarray
      ) -> np.ndarray:
    r = x_ref.reshape((2, 1))

    xd_dot = 1 /self.tau_attitude * (self.Ad @ xd_prev + self.Bd @ r)
    xd_next = xd_prev + self.dt * xd_dot

    return xd_next

  def generate_attitude_commands(self) -> None:
    start_time = rospy.Time.now()
    roll = [10, 0, -10, 0]
    pitch = [0, 10, 0, -10]

    i = 0
    attitude_reference = np.zeros((2, 1))
    attitude_cmd = np.zeros((2, 1))
    while not rospy.is_shutdown():
      attitude_cmd = self.get_filtered_reference(attitude_cmd, attitude_reference)
      # print(attitude_cmd)

      cmd_msg = AttitudeCommand()
      cmd_msg.header.stamp = rospy.Time.now()
      cmd_msg.roll = 0 # attitude_cmd[0] * np.pi / 180.0
      cmd_msg.pitch = 0# -10 * np.pi / 180.0 # attitude_cmd[1] * np.pi / 180.0
      cmd_msg.yaw = 0# -10 * np.pi / 180
      cmd_msg.gaz = 0#1
      self.attitude_command_pub.publish(cmd_msg)
      
      if (rospy.Time.now() - start_time).to_sec() > self.time_between_switch:

        attitude_reference = np.array([roll[i], pitch[i]]).T
        # i += 1
        # if i >= len(roll):
        #   i = 0

        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  generate_position_errors = GenerateAttitudeCommands()
  generate_position_errors.generate_attitude_commands()


if __name__ == "__main__":
  main()
