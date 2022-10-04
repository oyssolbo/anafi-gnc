#!/usr/bin/python3

import numpy as np
import casadi
from scipy.spatial.transform import Rotation
import do_mpc

import rospy
import std_msgs.msg

from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped
from olympe_bridge.msg import AttitudeCommand
from std_srvs.srv import SetBool, SetBoolResponse

import mpc_helpers.utilities as utilities

class ModelPredictiveController():

  def __init__(self) -> None:

    # Initializing node
    node_name = rospy.get_param("~node_name", default = "mpc_node")
    node_rate = rospy.get_param("~node_rate", default = 20)
    self.dt = 1.0 / node_rate 

    rospy.init_node(node_name)
    self.rate = rospy.Rate(node_rate)

    # Setup services
    rospy.Service("/mpc/service/enable_controller", SetBool, self.__enable_controller)

    # Setup subscribers 
    rospy.Subscriber("/anafi/optical_flow_velocities", Vector3Stamped, self.__optical_flow_velocities_cb)
    rospy.Subscriber("/anafi/attitude", QuaternionStamped, self.__attitude_cb)
    # Attitude estimate from the anafi
    # Optical flow feedback from the bridge
    # Position estimate from the EKF

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)

    # Initial values
    self.optical_velocities : np.ndarray = np.zeros((3, 1))
    self.attitude_rotation_matrix : np.ndarray = np.eye(3)
    self.attitude_rpy : np.ndarray = np.zeros((3, 1))

    self.optical_velocities_timestamp : std_msgs.msg.Time = None
    self.attitude_timestamp : std_msgs.msg.Time = None 

    self.is_controller_active : bool = False


  def __enable_controller(self, msg : SetBool):
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def __optical_flow_velocities_cb(self, msg : Vector3Stamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.optical_velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.optical_velocities_timestamp = msg_timestamp
    self.optical_velocities = np.array([msg.vector.x, msg.vector.y, msg.vector.z]).T

  
  def __attitude_cb(self, msg : QuaternionStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.attitude_timestamp, msg_timestamp):
      # Old message
      return
    
    self.attitude_timestamp = msg_timestamp
    rotation = Rotation.from_quat([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w])
    self.attitude_rotation_matrix = rotation.as_matrix()
    self.attitude_rpy = rotation.as_euler('xyz')


  def __ode(self, x ,u) -> None:
    drone_mass = 0.320
    gravitational_acceleration = 9.82179
    # Linear drag parameters from Martin Falang
    linear_drag_x = 0.08063504
    linear_drag_y = 0.09929089
    linear_drag_matrix = np.diag([linear_drag_x, linear_drag_y, 0.0])

    g_ned = np.array([[0], [0], [gravitational_acceleration]])

    state_velocity = np.array(
      [
        [x[3]], 
        [x[4]], 
        [x[5]]
      ]
    )
    rotation_matrix = Rotation.from_euler("xyz", np.array([x[6], x[7], x[8]])).as_matrix()

    d_pos = self.attitude_rotation_matrix @ np.array(
      [
        [x[3]], 
        [x[4]], 
        [x[5]]
      ]
    )
    d_vel = (rotation_matrix @ g_ned - linear_drag_matrix @ state_velocity) / (drone_mass)
    d_vel[2] = -u[3] # Change in velocity defined in NED, while the thrust is in ENU

    # Random values - must be estimated later with enough data (preferable from real world experiments)
    k_phi = 1
    k_theta = 1
    tau_phi = 0.01
    tau_theta = 0.01

    d_rpy = np.array(
      [
        [(k_phi * u[0] - x[6]) / tau_phi], 
        [(k_theta * u[1] - x[7]) / tau_theta], 
        [u[3]]
      ]
    )
    dx = np.vstack([d_pos, d_vel, d_rpy])
    return casadi.vertcat(*dx)

  def __initial_states(self) -> tuple:#[np.ndarray, np.ndarray]:
    x0 = [0] * 9 #np.zeros((9, 1)) # Make 9 into a variable
    u0 = [0] * 4 # np.zeros((4, 1)) # Make 4 into a variable
    return x0, u0 

  def __tuning_matrices(self) -> tuple:
    Q = casadi.MX()

  def __ineq_constraints(self) -> tuple:
    x_min = np.array([]) 
    x_max = np.array([])
    u_min = np.array([-10 * np.pi / 180.0, -10 * np.pi / 180.0, -30 * np.pi / 180.0, -0.1]) # Config file
    u_max = -1 * u_min
    return x_min, x_max, u_min, u_max

  def __tuning_parameters(self) -> list:
    q = [2, 2, 0.5, 0.25, 0.25, 0.1, 0.5, 0.5, 0.01]
    r = [1, 1, 1, 1]
    s = [1, 1, 0.01, 1, 1, 0.5, 0.5, 0.5, 1]
    return q, r, s

  def spin(self) -> None:
    while not rospy.is_shutdown():


            
      import os 
      import sys
      sys.exit(0)

      self.rate.sleep()


def main():
  node = ModelPredictiveController()
  node.spin()


if __name__ == "__main__":
  main()
