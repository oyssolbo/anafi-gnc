
import rospy
import geometry_msgs.msg
import nav_msgs.msg

import numpy as np
from scipy.spatial.transform import Rotation

from generic_output_saver import GenericOutputSaver

from anafi_uav_msgs.msg import PoseStampedEuler

class DronePoseDataSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, PoseStampedEuler, self._drone_gt_pose_cb)

    def _drone_gt_pose_cb(self, msg: PoseStampedEuler):

        output = [
            msg.header.stamp.to_sec(),
            msg.x,
            msg.y,
            msg.z,
            msg.phi,
            msg.theta,
            msg.psi
        ]

        self._save_output(output)

def ssa(angle):
    angle = (angle + 180) % 360 - 180

    return angle

class DroneVelocityDataSaver(GenericOutputSaver):
    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, nav_msgs.msg.Odometry, self._drone_gt_velocity_cb)

    def _drone_gt_velocity_cb(self, msg: nav_msgs.msg.Odometry):
        velocity_body = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

        rot_quat = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]

        euler_angles = Rotation.from_quat(rot_quat).as_euler("xyz", degrees=True)

        # Not sure why this axis was 180 deg wrong (tried using different "xyz", but did not find
        # out what the reason was so just fixed it manually)
        euler_angles[0] -= 180

        euler_angles[0] = ssa(euler_angles[0])
        # For some reason the pitch angle was flipped here. Not sure why, but changed
        # it manually as well
        euler_angles[1] = -ssa(euler_angles[1])
        euler_angles[2] = ssa(euler_angles[2])

        # Apply attitude offsets
        roll_offset = rospy.get_param("/drone/attitude_offsets/roll")
        pitch_offset = rospy.get_param("/drone/attitude_offsets/pitch")
        yaw_offset = rospy.get_param("/drone/attitude_offsets/yaw")

        euler_angles[0] -= roll_offset
        euler_angles[1] -= pitch_offset
        euler_angles[2] -= yaw_offset

        output = [
            rospy.Time.now().to_sec(),
            velocity_body[0],
            velocity_body[1],
            velocity_body[2],
            euler_angles[0],
            euler_angles[1],
            euler_angles[2],
        ]

        self._save_output(output)

# def Rz(radians):
#     c = np.cos(radians)
#     s = np.sin(radians)
#     return np.array([[c, -s, 0],
#                      [s, c, 0],
#                      [0, 0, 1]])


# class GroundTruthDataSaver(GenericOutputSaver):

#     def __init__(self, config, base_dir, output_category, output_type, environment):
#         super().__init__(config, base_dir, output_category, output_type, environment)

#         self.initialized_offsets = False
#         self.offsets = None # Format: numpy array [0,x,y,z,roll,pitch,yaw] (0 for timestamp)

#     def _initialize_offsets(self, output_raw, object_type):

#         self.psi_offset = output_raw[6]

#         R = Rz(-self.psi_offset*math.pi/180)

#         t = -R @ np.array([output_raw[1], output_raw[2], output_raw[3]]).reshape(-1,1)

#         # Create homogeneous transformation matrix
#         self.T_ned_helipad = np.vstack((np.hstack((R, t)), np.array([0,0,0,1])))

#         self.offsets = output_raw
#         self.offsets[0] = 0 # No offset in timestamp
#         print(f"Offsets ({object_type}): " \
#                 f"x: {self.offsets[1]:.3f}m y: {self.offsets[2]:.3f}m " \
#                 f"z: {self.offsets[3]:.3f}m roll: {self.offsets[4]:.3f}deg " \
#                 f"pitch: {self.offsets[5]:.3f}deg yaw: {self.offsets[6]:.3f}deg"
#         )

#         self.initialized_offsets = True

#     def _print_output(self, output, object_type):
#         # Used for setup of motion capture system
#         x, y, z, roll, pitch, yaw = output[1:]
#         print(f"Pose ({object_type}):\tx: {x:.3f} y: {y:.3f} z: {z:.3f}\tRoll: {roll:.3f} Pitch: {pitch:.3f} Yaw: {yaw:.3f}")

#     def _get_output_from_geometry_msg(self, msg):
#         """
#         Modifications made so that coordinate system is aligned with the on used
#         by the pose estimate. Here:
#         Motion capture:
#             - x-axis of mocap is negative y-axis of pose estimate
#             - y-axis of mocap is x-axis of pose estimate
#         Simulator:
#             - All axis are the same
#         """
#         # TODO: Make this work for simulator also
#         quat = [msg.pose.orientation.x,
#             msg.pose.orientation.y,
#             msg.pose.orientation.z,
#             msg.pose.orientation.w
#         ]
#         euler = Rotation.from_quat(quat).as_euler("xyz", degrees=True)

#         if self.environment == "real":
#             # res = np.array([
#             #     msg.header.stamp.to_sec(),
#             #     -msg.pose.position.y, # conversion between frames
#             #     msg.pose.position.x, # conversion between framess
#             #     -msg.pose.position.z,
#             #     euler[0],
#             #     euler[1],
#             #     euler[2]
#             # ])
#             res = np.array([
#                 msg.header.stamp.to_sec(),
#                 -msg.pose.position.x,
#                 -msg.pose.position.y,
#                 -msg.pose.position.z,
#                 euler[0],
#                 euler[1],
#                 euler[2]
#             ])
#         else:
#             res = np.array([
#                 msg.header.stamp.to_sec(),
#                 msg.pose.position.x,
#                 msg.pose.position.y,
#                 msg.pose.position.z,
#                 euler[0],
#                 euler[1],
#                 euler[2]
#             ])

#         return res

# class DronePoseDataSaver(GroundTruthDataSaver):

#     def __init__(self, config, base_dir, output_category, output_type, environment):
#         super().__init__(config, base_dir, output_category, output_type, environment)

#         rospy.Subscriber(self.topic_name, geometry_msgs.msg.PoseStamped, self._drone_gt_pose_cb)


#     def _drone_gt_pose_cb(self, msg):

#         output_raw = self._get_output_from_geometry_msg(msg)

#         # self._print_output(output_raw, "drone")

#         if self.initialized_offsets == False:
#             self._initialize_offsets(output_raw, "drone")

#         # NED position in homogeneous coordinates
#         pos_ned = np.array([output_raw[1], output_raw[2], output_raw[3], 1])
#         # print("Pos ned", pos_ned)

#         pos_helipad = self.T_ned_helipad @ pos_ned
#         # print("Pos helipad", pos_helipad)

#         orientation = output_raw[4:].copy()
#         orientation[2] = (orientation[2] - self.psi_offset + 180) % 360 - 180
#         # print(f"Psi: {orientation[2]}")

#         output = [
#             output_raw[0],
#             pos_helipad[0],
#             pos_helipad[1],
#             pos_helipad[2],
#             orientation[0],
#             orientation[1],
#             orientation[2]
#         ]
#         # print(output)

#         # self._print_output(output, "drone")

#         self._save_output(output)

# class HelipadPoseDataSaver(GroundTruthDataSaver):

#     def __init__(self, config, base_dir, output_category, output_type, environment):
#         super().__init__(config, base_dir, output_category, output_type, environment)

#         rospy.Subscriber(self.topic_name, geometry_msgs.msg.PoseStamped, self._helipad_gt_pose_cb)

#     def _helipad_gt_pose_cb(self, msg):

#         output_raw = self._get_output_from_geometry_msg(msg)

#         # self._print_output(output_raw, "helipad")

#         if self.initialized_offsets == False:
#             self._initialize_offsets(output_raw, "helipad")

#         output = output_raw - self.offsets

#         # self._print_output(output, "helipad")

#         self._save_output(output)

class HelipadPoseDataSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, geometry_msgs.msg.PoseStamped, self._helipad_gt_pose_cb)

    def _helipad_gt_pose_cb(self, msg: geometry_msgs.msg.PoseStamped):

        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        euler = Rotation.from_quat(quat).as_euler("xyz", degrees=True)

        output = np.array([
            msg.header.stamp.to_sec(),
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            euler[0],
            euler[1],
            euler[2]
        ])

        self._save_output(output)

class DronePoseNEDDataSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, geometry_msgs.msg.PoseStamped, self._drone_gt_pose_cb)

    def _drone_gt_pose_cb(self, msg: geometry_msgs.msg.PoseStamped):

        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        euler = Rotation.from_quat(quat).as_euler("xyz", degrees=True)

        output = np.array([
            msg.header.stamp.to_sec(),
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            euler[0],
            euler[1],
            euler[2]
        ])

        self._save_output(output)