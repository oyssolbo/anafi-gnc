#!/usr/bin/env python3

# Script for interfacing to the Anafi-drone through ROS topics.

import os
import sys
import yaml
import rospy
import olympe
import threading

from anafi_data_publisher import GpsPublisher, TelemetryPublisher, CameraPublisher
from command_listener import CommandListener

olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

class OlympeRosBridge():
    """
    Class which bridges Olympe and ROS, making it possible to receive the data
    from the Anafi drone via ROS topics as well as sending commands to the
    drone via ROS topics.
    """
    def __init__(self):
        rospy.init_node("anafi_interface", anonymous=False)

        config_file = rospy.get_param("~config_file")
        script_dir = os.path.dirname(os.path.realpath(__file__))

        try:
            with open(f"{script_dir}/../../config/{config_file}") as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            sys.exit()

        drone_ip = rospy.get_param("~drone_ip")

        self.drone = olympe.Drone(drone_ip)
        self.drone.logger.setLevel(40)
        self.drone.connect()

        # Options for using SkyController as relay
        self.use_skycontroller = False
        if drone_ip == "192.168.53.1":
            self.use_skycontroller = True
            assert self.drone(olympe.messages.skyctrl.CoPiloting.setPilotingSource(
                source="Controller"
            )).wait().success(), "Failed to set piloting source to Olympe"
            rospy.logwarn("Drone controlled from Olympe, disconnect SkyController to resume control.")

        # Set reject_jitter variable for images based on environment
        if drone_ip == "10.202.0.1": # simulation
            reject_jitter = False
        else: # real life
            reject_jitter = True

        self.command_listener = CommandListener(self.drone)

        self.telemetry_publisher = TelemetryPublisher(
            self.drone, self.config["drone"]["topics"]["output"]["telemetry"]
        )
        self.gps_publisher = GpsPublisher(
            self.drone, self.config["drone"]["topics"]["output"]["gnss"]
        )

        visualize = rospy.get_param("~view_camera_output")
        if not visualize:
            rospy.loginfo("Not showing live camera feed (but images are still published).")

        self.camera_streamer = CameraPublisher(
            self.drone, self.config["drone"]["topics"]["output"]["camera"],
            reject_jitter=reject_jitter, visualize=visualize
        )

    def start(self):
        """
        Start the interface.
        """

        rospy.sleep(1)
        self.command_listener.init(camera_angle=-90)
        rospy.sleep(1)

        if self.use_skycontroller:
            threading.Thread(target=self.switch_piloting_mode, args=(), daemon=True).start()
        threading.Thread(target=self.telemetry_publisher.publish, args=(), daemon=True).start()
        threading.Thread(target=self.gps_publisher.publish, args=(), daemon=True).start()
        threading.Thread(target=self.camera_streamer.publish, args=(), daemon=True).start()

        rospy.spin()

    def switch_piloting_mode(self):
        rospy.logwarn("Press enter to switch to SkyController")
        input()
        # Stop the drone if flying in attitude control
        self.drone(olympe.messages.ardrone3.Piloting.PCMD(
            0, 0, 0, 0, 0, 0
        ))
        assert self.drone(olympe.messages.skyctrl.CoPiloting.setPilotingSource(
                source="SkyController"
        )).wait().success(), "Failed to set piloting source to SkyController"
        rospy.logwarn("SkyController control engaged.")

def main():
    anafi_interface = OlympeRosBridge()
    anafi_interface.start()


if __name__ == "__main__":
    main()
