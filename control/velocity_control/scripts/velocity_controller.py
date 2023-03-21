#!/usr/bin/python3

import numpy as np

import rospy
import std_msgs.msg

import velocity_control_helpers.velocity_reference_model as velocity_reference_model
import velocity_control_helpers.controllers as controllers
import velocity_control_helpers.utilities as utilities

from geometry_msgs.msg import TwistStamped, Vector3Stamped, QuaternionStamped
from olympe_bridge.msg import AttitudeCommand
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension

np.warnings.filterwarnings('ignore', category=np.VisibleDeprecationWarning) 


class VelocityController():

  def __init__(self) -> None:

    # Initializing node
    rospy.init_node("velocity controller_node")
    node_rate = rospy.get_param("~node_rate")
    self.dt = 1.0 / node_rate 
    self.rate = rospy.Rate(node_rate)

    # Initializing reference models
    selected_control_method = rospy.get_param("/controller", default="pid") 
    controller_parameters = rospy.get_param("~" + selected_control_method)
    attitude_limits = rospy.get_param("~attitude_limits")
    selected_controller = controllers.get_controller(selected_control_method)
    self.controller = selected_controller(
      params=controller_parameters,
      limits=attitude_limits 
    )

    velocity_reference_model_parameters = rospy.get_param("~velocity_reference_model")
    velocity_reference_omegas = velocity_reference_model_parameters["omegas"]
    velocity_reference_zetas = velocity_reference_model_parameters["zetas"]
    velocity_reference_T_z = velocity_reference_model_parameters["T_z"]
    velocity_reference_K_z = velocity_reference_model_parameters["K_z"]
    self.velocity_reference_model = velocity_reference_model.VelocityReferenceModel(
      omegas=velocity_reference_omegas,
      zetas=velocity_reference_zetas,
      T_z=velocity_reference_T_z,
      K_z=velocity_reference_K_z
    )

    # Setup services
    rospy.Service("/velocity_controller/service/enable_controller", SetBool, self._enable_controller_srv)

    # Setup subscribers 
    self.use_optical_flow_as_feedback : bool = rospy.get_param("/use_optical_flow_as_feedback")
    if self.use_optical_flow_as_feedback:
      rospy.loginfo("Velocity controller using optical flow velocity estimates as feedback")
      rospy.Subscriber("/anafi/optical_flow_velocities", Vector3Stamped, self._optical_flow_velocities_cb)
    else:
      rospy.loginfo("Velocity controller using polled velocity estimates as feedback")
      rospy.Subscriber("/anafi/polled_body_velocities", TwistStamped, self._polled_velocities_cb)

    # Possibility to use both pure-pursuit and PID-guidance. The project-thesis shows that these are
    # really similar, as long as one is actually able to use the correct formula (which M. Falang was
    # not able to, and therefore got skewed results). The code leaves up to change the guidance module,
    # but the pure-pursuit is used as default. Will currently crash if default-value is removed 
    use_pure_pursuit_guidance : bool = rospy.get_param("/use_pure_pursuit_guidance", default=True)
    if use_pure_pursuit_guidance:
      rospy.loginfo("Velocity controller using pure pursuit guidance as velocity reference")
      rospy.Subscriber("/guidance/pure_pursuit/velocity_reference", TwistStamped, self._reference_velocities_cb)
    else:
      rospy.loginfo("Velocity controller using PID-based guidance as velocity reference")
      rospy.Subscriber("/guidance/pid/velocity_reference", TwistStamped, self._reference_velocities_cb)

    rospy.Subscriber("/anafi/attitude", QuaternionStamped, self._attitude_cb)

    # Setup publishers
    self.attitude_ref_pub = rospy.Publisher("/anafi/cmd_rpyt", AttitudeCommand, queue_size=1)
    self.ipid_delta_hat_pub = rospy.Publisher("/ipid/delta_hat", Float64MultiArray, queue_size=1)

    # Initial values
    self.guidance_reference_velocities : np.ndarray = np.zeros((3, 1))
    self.velocities : np.ndarray = np.zeros((3, 1))
    self.quaternion_body_to_ned : np.ndarray = np.array([1, 0, 0, 0], dtype=np.float)

    self.attitude_timestamp : std_msgs.msg.Time = None
    self.velocities_timestamp : std_msgs.msg.Time = None
    self.guidance_timestamp : std_msgs.msg.Time = None

    self.is_controller_active : bool = False


  def _enable_controller_srv(self, msg : SetBoolRequest) -> SetBoolResponse:
    rospy.loginfo("Action request received: " + str(msg.data))
    self.is_controller_active = msg.data

    res = SetBoolResponse()
    res.success = True
    res.message = "" 
    return res 


  def _reference_velocities_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.guidance_timestamp, msg_timestamp):
      # Old message
      return

    self.guidance_timestamp = msg_timestamp
    self.guidance_reference_velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def _polled_velocities_cb(self, msg : TwistStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.velocities_timestamp = msg_timestamp     
    self.velocities = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]).T


  def _optical_flow_velocities_cb(self, msg : Vector3Stamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.velocities_timestamp, msg_timestamp):
      # Old message
      return
    
    self.velocities_timestamp = msg_timestamp     
    self.velocities = np.array([msg.vector.x, msg.vector.y, msg.vector.z]).T


  def _attitude_cb(self, msg : QuaternionStamped) -> None:
    msg_timestamp = msg.header.stamp

    if not utilities.is_new_msg_timestamp(self.attitude_timestamp, msg_timestamp):
      # Old message
      return
    
    self.attitude_timestamp = msg_timestamp     
    self.quaternion_body_to_ned = np.array([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]).T


  def _convert_body_velocities_to_ned(
        self, 
        velocities_body : np.ndarray
      ) -> np.ndarray:
    """
    Obs: Future improvement to use tf2 instead of manually converting these
    measurements 
    """
    rot_body_to_ned = Rotation(quat=self.quaternion_body_to_ned).as_matrix()
    return rot_body_to_ned.T @ velocities_body


  def _prevent_underflow(
        self, 
        arr       : np.ndarray, 
        min_value : float       = 1e-6
      ) -> np.ndarray:
    """
    Preventing underflow to setting all values with an absolute value below
    @p min_value to 0 
    """
    with np.nditer(arr, op_flags=['readwrite']) as iterator:
      for val in iterator:
        if np.abs(val) < min_value:
          val[...] = 0
    return arr


  def publish_attitude_ref(self) -> None:
    attitude_cmd_msg = AttitudeCommand()

    delta_hat_msg = Float64MultiArray()
    delta_hat_msg.layout = MultiArrayLayout()

    v_ref_body = np.zeros((5, 1))
    while not rospy.is_shutdown():
      if self.is_controller_active:
        v_ref_body = self.velocity_reference_model.get_body_velocity_reference(
          xd_prev=v_ref_body, 
          v_ref_raw=self.guidance_reference_velocities,
          dt=self.dt
        )
        v_ref_ned = self._convert_body_velocities_to_ned(np.array([v_ref_body[0], v_ref_body[1], v_ref_body[4]]))

        att_ref = self.controller.get_attitude_reference(
          v_ref=v_ref_body,
          v=self.velocities, 
          ts=self.velocities_timestamp
        )

        att_ref_3D = np.array([att_ref[0], att_ref[1], 0, v_ref_ned[2]], dtype=np.float64) 
        attitude_cmd_msg.header.stamp = rospy.Time.now()
        attitude_cmd_msg.roll = att_ref_3D[0]   
        attitude_cmd_msg.pitch = att_ref_3D[1]
        attitude_cmd_msg.yaw = att_ref_3D[2]
        attitude_cmd_msg.gaz = att_ref_3D[3]

        self.attitude_ref_pub.publish(attitude_cmd_msg)

        # Publish current adaptive estimates in roll and pitch
        delta_hat = self.controller.get_delta_hat()
        delta_hat_msg.data = [delta_hat[0], delta_hat[1]]
        self.ipid_delta_hat_pub.publish(delta_hat_msg)

      else:
        self.guidance_reference_velocities = np.zeros((3, 1))
        v_ref_body = np.zeros((5, 1))
      
      self.rate.sleep()


def main():
  node = VelocityController()
  node.publish_attitude_ref()


if __name__ == "__main__":
  main()
