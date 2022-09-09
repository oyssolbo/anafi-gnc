#!/usr/bin/env python3

import os
import sys
import yaml

import rospy
import geometry_msgs.msg
import perception.msg
import drone_interface.msg
import numpy as np

from ekf import EKF, EKFState
import utilities.transform as tf
import dynamic_models
import measurement_models

class EKFRosRunner():

    def __init__(self):

        rospy.init_node("perception_ekf", anonymous=False)

        # Load config
        config_file = rospy.get_param("~config_file")
        script_dir = os.path.dirname(os.path.realpath(__file__))

        try:
            with open(f"{script_dir}/../../config/{config_file}") as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            sys.exit()

        # Get dynamic models based on config file
        self.dynamic_model_type = self.config["ekf"]["dynamic_model"]
        self.dynamic_model = dynamic_models.get_dynamic_model_from_type(
            self.dynamic_model_type,
            self.config["dynamic_models"][self.dynamic_model_type]["sigmas"]
        )

        # Get measurement models based on config file
        self.measurement_model_types = self.config["ekf"]["measurement_models"]
        self.measurement_models_dict = measurement_models.get_measurement_models_from_types(
            self.measurement_model_types, self.config["measurements"]
        )

        rospy.loginfo(f"EKF dynamic model: {self.dynamic_model_type}")
        rospy.loginfo(f"EKF measurement model(s): {self.measurement_model_types}")

        # Create estimate publisher based on the states used in the dynamic model
        self.output_states = self.config["dynamic_models"][self.dynamic_model_type]["output_states"]
        self.estimate_publisher = self._get_estimate_publisher(self.output_states)

        # Create ROS subscribers for the input and measurements defined in the config file
        self.has_inputs = True
        self._prev_attitude: np.ndarray = None
        self._setup_subscribers()

        # Set up filter
        self.filter = EKF(self.dynamic_model, self.measurement_models_dict)

        x0 = np.array(self.config["dynamic_models"][self.dynamic_model_type]["init_values"]["x0"])
        P0 = np.diag(np.array(self.config["dynamic_models"][self.dynamic_model_type]["init_values"]["P0"]))**2

        self.ekf_estimate = EKFState(x0, P0)

        self.dt = self.config["ekf"]["dt"]

        # Keep track of when the first measurement arrives and only then start
        # the filter
        self.estimating = False

    def run(self):
        rospy.loginfo("Starting perception EKF")

        # If there are inputs coming into the system, the precition step will be
        # performed each time a new input arrives and this is handled in the respective
        # input callback
        if self.has_inputs:
            rospy.spin()
        else:
            while not rospy.is_shutdown():

                if self.estimating:
                    self.ekf_estimate = self.filter.predict(self.ekf_estimate, None, self.dt)
                    output_msg = self._pack_estimate_msg(self.ekf_estimate, self.output_states)
                    self.estimate_publisher.publish(output_msg)

                rospy.sleep(self.dt)

    def _get_estimate_publisher(self, output_states: str):
        if output_states == "position":
            publisher = rospy.Publisher(
                self.config["ekf"]["output"]["topic_name"],
                perception.msg.PointWithCovarianceStamped, queue_size=10
            )
        else:
            raise NotImplementedError

        return publisher

    def _setup_subscribers(self):

        input_config = self.config["dynamic_models"][self.dynamic_model_type]["input"]

        if input_config["type"] == "control_inputs":
            raise NotImplementedError
        elif input_config["type"] == "none":
            self.has_inputs = False
        else:
            raise NotImplementedError

        for mm in self.measurement_model_types:
            if mm == "dnn_cv_position":
                rospy.Subscriber(
                    self.config["measurements"]["dnn_cv_position"]["topic_name"],
                    geometry_msgs.msg.PointStamped, self._dnn_cv_position_cb
                )
            elif mm == "drone_velocity":
                rospy.Subscriber(
                    self.config["measurements"]["drone_velocity"]["topic_name"],
                    drone_interface.msg.AnafiTelemetry, self._drone_velocity_cb
                )
            elif mm == "tcv_position":
                rospy.Subscriber(
                    self.config["measurements"]["tcv_position"]["topic_name"],
                    perception.msg.EulerPose, self._tcv_position_cb
                )
            elif mm == "dnn_cv_position_xy":
                pass # Uses same callback as dnn_cv_position
            else:
                print(f"Measurement model: {mm} not implemented")
                raise NotImplementedError


    def _pack_estimate_msg(self, ekfstate: EKFState, states_type: str):

        if states_type == "position":
            msg = perception.msg.PointWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "drone_body"
            msg.position.x = ekfstate.mean[0]
            msg.position.y = ekfstate.mean[1]
            msg.position.z = ekfstate.mean[2]
            msg.covariance = ekfstate.cov[:3,:3].flatten().tolist()
        else:
            raise NotImplementedError

        return msg

    def _dnn_cv_position_cb(self, msg: geometry_msgs.msg.PointStamped):
        # Only use dnncv for altitude measurement if above certain altitude (not reliable below)
        if self.ekf_estimate.mean[2] > 0.8:
            z = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "dnn_cv_position")
        else:
            z = np.array([msg.point.x, msg.point.y])
            self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "dnn_cv_position_xy")

    def _tcv_position_cb(self, msg: perception.msg.EulerPose):
        z = np.array([msg.x, msg.y, msg.z])

        self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "tcv_position")

    def _drone_velocity_cb(self, msg: drone_interface.msg.AnafiTelemetry):
        if not self.estimating:
            self.estimating = True

        # Hack to incorporate heading into the model by rotating the position estimate
        if self._prev_attitude is not None:
            xyz = self.ekf_estimate.mean[:3]
            drho = (self._prev_attitude[0] - msg.roll + 180) % 360 - 180 # use SSA
            dtheta = (self._prev_attitude[1] - msg.pitch + 180) % 360 - 180 # use SSA
            dpsi = (self._prev_attitude[2] - msg.yaw + 180) % 360 - 180 # use SSA

            # print(f"ts: {msg.header.stamp.to_sec()}\tdrho: {drho:.3f}\tdtheta: {dtheta:.3f}\tdpsi: {dpsi:.3f}")

            R = tf.Rzyx(-drho, dtheta, dpsi)

            # R = np.array([[np.cos(np.deg2rad(dpsi)), -np.sin(np.deg2rad(dpsi))],
            #             [np.sin(np.deg2rad(dpsi)), np.cos(np.deg2rad(dpsi))]])
            xyz_rot = R @ xyz
            self.ekf_estimate.mean[0] = xyz_rot[0]
            self.ekf_estimate.mean[1] = xyz_rot[1]
            self.ekf_estimate.mean[2] = xyz_rot[2]

        # self._prev_attitude = msg.yaw
        self._prev_attitude = np.array([msg.roll, msg.pitch, msg.yaw])

        z = np.array([msg.vx, msg.vy, msg.vz])

        # Not sure if this is correct or not
        # if self.ekf_estimate.mean[0] < 0:
        #     z[0] *= -1
        # if self.ekf_estimate.mean[0] < 0:
        #     z[1] *= -1

        # # Remove entries that are below a certain threshold as these are not accurate
        z[np.where(np.abs(z) < 0.1)] = 0

        self.ekf_estimate = self.filter.update(z, self.ekf_estimate, "drone_velocity")

def main():
    ekf_runner = EKFRosRunner()
    ekf_runner.run()

if __name__ == "__main__":
    main()