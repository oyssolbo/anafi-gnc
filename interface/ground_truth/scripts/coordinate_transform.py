#!/usr/bin/env python3

import os
import sys
import yaml

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
import geometry_msgs.msg
from anafi_uav_msgs.msg import PoseStampedEuler

def Rx(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

def Ry(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def Rz(degrees):
    radians = np.deg2rad(degrees)
    c = np.cos(radians)
    s = np.sin(radians)

    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])

class CoordinateTransform():

    def __init__(self):

        rospy.init_node("coordinate_transform", anonymous=False)

        script_dir = os.path.dirname(os.path.realpath(__file__))

        config_file = rospy.get_param("~config_file")
        self.environment = rospy.get_param("~environment")

        try:
            with open(f"{script_dir}/../config/{config_file}") as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            sys.exit()

        # Set up raw ground truth subscribers
        rospy.Subscriber(self.config["topics"]["drone_pose_ned"][self.environment],
            geometry_msgs.msg.PoseStamped, self._drone_gt_pose_cb
        )

        self.new_drone_gt_pose = False
        self.newest_drone_pose_msg: geometry_msgs.msg.PoseStamped = None

        rospy.Subscriber(self.config["topics"]["helipad_pose_ned"][self.environment],
            geometry_msgs.msg.PoseStamped, self._helipad_gt_pose_cb
        )

        self.new_helipad_gt_pose = False
        self.newest_helipad_pose_msg: geometry_msgs.msg.PoseStamped = None

        # Set up publisher for new drone pose relative to helipad in the helipad frame
        self.drone_pose_helipad_frame_publisher = rospy.Publisher(
            self.config["topics"]["drone_pose_helipad_frame"][self.environment],
            PoseStampedEuler, queue_size=1
        )

        # Set up publisher for new drone pose realtive to the helipad in the body frame
        self.helipad_pose_body_frame_publisher = rospy.Publisher(
            self.config["topics"]["helipad_pose_body_frame"][self.environment],
            PoseStampedEuler, queue_size=1
        )


    def _drone_gt_pose_cb(self, msg: geometry_msgs.msg.PoseStamped):
        # Do the job here, and assume that the previously stored helipad position
        # is correct as the helipad should not have any high speed maneuvers
        if self.newest_helipad_pose_msg != None:
            # print(f"Difference: {(msg.header.stamp - self.newest_helipad_pose_msg.header.stamp).to_sec()*1000}")
            drone_pose_ned_frame = self._geometry_msg_pose_to_array(msg)
            helipad_pose_ned_frame = self._geometry_msg_pose_to_array(self.newest_helipad_pose_msg)
            drone_pose_helipad_frame = self._ned_to_helipad_frame(drone_pose_ned_frame, helipad_pose_ned_frame)
            helipad_pose_body_frame = self._ned_to_body_frame(drone_pose_ned_frame, helipad_pose_ned_frame)
            self._publish_drone_pose(drone_pose_helipad_frame, "helipad")
            self._publish_drone_pose(helipad_pose_body_frame, "drone_body")

    def _helipad_gt_pose_cb(self, msg: geometry_msgs.msg.PoseStamped):
        self.newest_helipad_pose_msg = msg

    def _ned_to_helipad_frame(self, drone_pose_ned: np.ndarray, helipad_pose_ned: np.ndarray):

        # Create rotation matrix between NED and Helipad frame
        helipad_rotation = helipad_pose_ned[5]

        R_ned_to_heli = Rz(helipad_rotation)

        # Translation between Helipad and NED frame (negative since this by definition
        # is the translation Helipad->NED expressed in the Helipad frame, but the
        # simulator and motion capture system gives the distance expressed in the
        # NED frame).
        t_heli_to_ned_in_heli = - R_ned_to_heli @ np.array([
            helipad_pose_ned[0],
            helipad_pose_ned[1],
            helipad_pose_ned[2]
        ])

        # Convert position
        drone_position_ned = np.array([
            drone_pose_ned[0],
            drone_pose_ned[1],
            drone_pose_ned[2]
        ])
        drone_position_helipad = R_ned_to_heli @ drone_position_ned + t_heli_to_ned_in_heli

        # Convert orientation
        drone_orientation_helipad = np.array([
            drone_pose_ned[3],
            drone_pose_ned[4],
            (helipad_rotation - drone_pose_ned[5] + 180) % 360 - 180 # Use smallest signed angle
        ])

        # Store result
        drone_pose_helipad = np.array([
            drone_position_helipad[0],
            drone_position_helipad[1],
            drone_position_helipad[2],
            drone_orientation_helipad[0],
            drone_orientation_helipad[1],
            drone_orientation_helipad[2],
        ])

        return drone_pose_helipad

    def _ned_to_body_frame(self, drone_pose_ned: np.ndarray, helipad_pose_ned: np.ndarray):

        # Create rotation matrix between NED and drone body frame
        R_ned_to_body = Rz(drone_pose_ned[5]) @ Ry(drone_pose_ned[4]) @ Rx(drone_pose_ned[3])

        # Translation between body and NED frame (negative since this by definition
        # is the translation body->NED expressed in the body frame, but the
        # simulator and motion capture system gives the distance expressed in the
        # NED frame).
        t_body_to_ned_in_body = - R_ned_to_body @ np.array([
            drone_pose_ned[0],
            drone_pose_ned[1],
            drone_pose_ned[2]
        ])

        # Convert position
        helipad_position_ned = np.array([
            helipad_pose_ned[0],
            helipad_pose_ned[1],
            helipad_pose_ned[2]
        ])
        helipad_position_body = R_ned_to_body @ helipad_position_ned + t_body_to_ned_in_body

        # Use orientation straight from NED as this is not really that important here
        helipad_orientation_body = np.array([
            helipad_pose_ned[3],
            helipad_pose_ned[4],
            helipad_pose_ned[5]
        ])

        # Store result
        helipad_pose_body = np.array([
            helipad_position_body[0],
            helipad_position_body[1],
            helipad_position_body[2],
            helipad_orientation_body[0],
            helipad_orientation_body[1],
            helipad_orientation_body[2],
        ])

        return helipad_pose_body

    def _geometry_msg_pose_to_array(self, pose: geometry_msgs.msg.PoseStamped):

        quat = [pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ]

        euler = Rotation.from_quat(quat).as_euler("xyz", degrees=True)

        # Redefine heading angle to be relative to North axis instead of East axis
        # as is given by the simulator
        if self.environment == "sim":
            euler[2] = (euler[2] - 90 + 180) % 360 - 180
        # This is not needed for the motion capture system since it defines the angle
        # relative to the North axis
        elif self.environment == "real":
            euler[2] = (euler[2] + 180) % 360 - 180

        # Apply attitude offsets
        # TODO: Find out if these are needed in the simulation as well
        roll_offset = rospy.get_param("/drone/attitude_offsets/roll")
        pitch_offset = rospy.get_param("/drone/attitude_offsets/pitch")
        yaw_offset = rospy.get_param("/drone/attitude_offsets/yaw")

        euler[0] += roll_offset
        euler[1] += pitch_offset
        euler[2] += yaw_offset

        ret = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            euler[0],
            euler[1],
            euler[2]
        ])

        return ret

    def _publish_drone_pose(self, pose: np.ndarray, frame_id: str):

        if frame_id == "helipad":
            publisher = self.drone_pose_helipad_frame_publisher
        elif frame_id == "drone_body":
            publisher = self.helipad_pose_body_frame_publisher
        else:
            rospy.logerr("Invalid frame ID, no such publisher available.")
            return

        msg = PoseStampedEuler()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.x = pose[0]
        msg.y = pose[1]
        msg.z = pose[2]
        msg.phi = pose[3]
        msg.theta = pose[4]
        msg.psi = pose[5]

        publisher.publish(msg)

    def start(self):
        rospy.loginfo("Starting ground truth coordinate frame alignment")

        rospy.spin()

def main():
    coordinate_transformer = CoordinateTransform()
    coordinate_transformer.start()

if __name__ == "__main__":
    main()