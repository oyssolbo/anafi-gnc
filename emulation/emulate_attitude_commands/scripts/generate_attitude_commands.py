#!/usr/bin/python3

import rospy 
import numpy as np
from olympe_bridge.msg import AttitudeCommand


class GenerateAttitudeCommands():
  def __init__(self) -> None:
    node_name = rospy.get_param("~node_name", default = "emulate_attitude_commands_node")
    controller_rate = rospy.get_param("~node_rate", default = 50)
    self.dt = 1.0 / controller_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(controller_rate)

    # Set up publishers
    self.attitude_command_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)   
 
    # Set up private variables
    self.estimate_models : bool = rospy.get_param("/estimate_models", default=False)
    if self.estimate_models:
      self.estimate_roll_models : bool = rospy.get_param("/estimate_roll_models", default=False)
      self.estimate_pitch_models : bool = rospy.get_param("/estimate_pitch_models", default=False)

      self.frequency : float = 0.5  

    self.time_to_stabilize : float = 1.0

    # First order reference model for roll and pitch
    self.tau_attitude = 0.05 # 0.05 for step-response, given a frequency of 20 Hz. Above 0.05 for a first-order model
    self.Ad = -np.eye(2)
    self.Bd = np.eye(2)


  def get_filtered_reference(
        self, 
        xd_prev : np.ndarray, 
        x_ref   : np.ndarray
      ) -> np.ndarray:
    r = x_ref.reshape((2, 1))

    xd_dot = 1 / self.tau_attitude * (self.Ad @ xd_prev + self.Bd @ r)
    xd_next = xd_prev + self.dt * xd_dot

    return xd_next

  def generate_attitude_commands(self) -> None:
    test_angle_rad = np.deg2rad(10)

    roll = [0] * 4
    pitch = [0] * 4
    if self.estimate_models:
      if self.estimate_roll_models:
        rospy.loginfo("Generating commands for roll")
        roll = [0, test_angle_rad, 0, -test_angle_rad]
      else:
        rospy.loginfo("Generating commands for pitch")
        pitch = [0, test_angle_rad, 0, -test_angle_rad]

    wait_time = self.time_to_stabilize

    i = 0
    attitude_reference = np.zeros((2, 1))
    attitude_cmd = np.zeros((2, 1))

    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
      attitude_cmd = self.get_filtered_reference(attitude_cmd, attitude_reference)

      cmd_msg = AttitudeCommand()
      cmd_msg.header.stamp = rospy.Time.now()
      cmd_msg.roll = attitude_cmd[0] 
      cmd_msg.pitch = attitude_cmd[1]
      cmd_msg.yaw = 0
      cmd_msg.gaz = 0
      self.attitude_command_pub.publish(cmd_msg)

      if (rospy.Time.now() - start_time).to_sec() > wait_time:

        i += 1
        if i >= len(roll):
          i = 0
          self.frequency += 0.5

        angles_list = [roll[i], pitch[i]]
        attitude_reference = np.array(angles_list).T

        if any(angles_list):
          # Desired roll or pitch is nonzero
          wait_time = 1 / self.frequency
          if wait_time <= 1.5 * self.dt:
            rospy.loginfo("Node too slow for desired rate. Aborting")
            break 

        else:
          wait_time = self.time_to_stabilize

        start_time = rospy.Time.now()

      self.rate.sleep()


def main():
  generate_position_errors = GenerateAttitudeCommands()
  generate_position_errors.generate_attitude_commands()


if __name__ == "__main__":
  main()
