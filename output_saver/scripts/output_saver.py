#!/usr/bin/env python3

import os
import yaml
import sys
import time
import shutil

import rospy

import ground_truths
import estimates
import commands

class OutputSaver():

    def __init__(self):

        rospy.init_node("output_saver", anonymous=False)

        script_dir = os.path.dirname(os.path.realpath(__file__))

        config_file = rospy.get_param("~config_file")
        self.environment = rospy.get_param("~environment")

        try:
            with open(f"{script_dir}/../config/{config_file}") as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            sys.exit()

        today = time.localtime()
        self.output_base_dir = f"{script_dir}/../../out" \
            f"/{today.tm_year}-{today.tm_mon}-{today.tm_mday}" \
            f"/{today.tm_hour}-{today.tm_min}-{today.tm_sec}/"

    def start(self):
        rospy.loginfo(f"Saving output to {self.output_base_dir}")

        estimates.DnnCvPositionSaver(self.config, self.output_base_dir,
            "estimates", "dnn_cv_position", self.environment
        )

        estimates.DnnCvHeadingSaver(self.config, self.output_base_dir,
            "estimates", "dnn_cv_heading", self.environment
        )

        estimates.EkfPositionSaver(self.config, self.output_base_dir,
            "estimates", "ekf_position", self.environment
        )

        estimates.AnafiHeightSaver(self.config, self.output_base_dir,
            "estimates", "anafi_height", self.environment
        )

        estimates.TcvDataSaver(self.config, self.output_base_dir,
            "estimates", "tcv_pose", self.environment
        )

        estimates.AnafiOpticalFlowVelSaver(self.config, self.output_base_dir,
            "estimates", "anafi_optical_flow", self.environment
        )

        estimates.AnafiPolledVelSaver(self.config, self.output_base_dir,
            "estimates", "anafi_polled_vel", self.environment
        )

        estimates.GuidanceSaver(self.config, self.output_base_dir,
            "estimates", "guidance_pid", self.environment
        )

        estimates.GuidanceSaver(self.config, self.output_base_dir,
            "estimates", "guidance_pure_pursuit", self.environment
        )


        ground_truths.DronePoseDataSaver(self.config, self.output_base_dir,
            "ground_truths", "drone_pose_helipad_frame", self.environment
        )

        ground_truths.DronePoseDataSaver(self.config, self.output_base_dir,
            "ground_truths", "helipad_pose_body_frame", self.environment
        )

        # Sphinx simulator does not have velocity output so this is only available
        # when using the motion capture system
        if self.environment == "real":
            ground_truths.DroneVelocityDataSaver(self.config, self.output_base_dir,
                "ground_truths", "drone_velocity_body_frame_and_attitude", self.environment
            )

        ground_truths.HelipadPoseDataSaver(self.config, self.output_base_dir,
            "ground_truths", "helipad_pose", self.environment
        )

        ground_truths.DronePoseNEDDataSaver(self.config, self.output_base_dir,
            "ground_truths", "drone_pose_ned", self.environment
        )

        commands.AnafiAttitudeCMDSaver(self.config, self.output_base_dir,
            "commands", "attitude_cmd", self.environment
        )

        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()

    def _on_shutdown(self):
        ans = input("Save output? [y/n] ")
        if ans.lower() == "n" or ans.lower() == "no":
            rospy.loginfo(f"Deleting directory {self.output_base_dir}")
            shutil.rmtree(self.output_base_dir)
        else:
            rospy.loginfo(f"Saved estimates to {self.output_base_dir}")


def main():
    output_saver = OutputSaver()
    output_saver.start()

if __name__ == "__main__":
    main()