#!/usr/bin/env python3

# Simple program which converts received quaternions into Euler angles.
# Used for debugging.

import rospy
import nav_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation

def ssa(angle):
    angle = (angle + 180) % 360 - 180

    return angle

def callback(msg: nav_msgs.msg.Odometry):
    rot_quat = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]

    euler_angles = Rotation.from_quat(rot_quat).as_euler("xyz", degrees=True)
    euler_angles[0] -= 180

    euler_angles[0] = ssa(euler_angles[0])
    euler_angles[1] = ssa(euler_angles[1])
    euler_angles[2] = ssa(euler_angles[2])

    print(f"R: {euler_angles[0]:.2f} \tP: {euler_angles[1]:.2f} \tY: {euler_angles[2]:.2f}")

rospy.init_node("quaternion_live_converter")

topic_name = "/qualisys/anafi/odom"
rospy.Subscriber(topic_name, nav_msgs.msg.Odometry, callback)

rospy.spin()
