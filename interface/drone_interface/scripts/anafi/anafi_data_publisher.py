
import os
import csv
import time
import queue
import pathlib
import threading

import rospy
import sensor_msgs.msg
# import anafi_uav_msgs.msg
import anafi_uav_msgs

import olympe
import olympe.messages as olympe_msgs
import olympe.enums as olympe_enums

import cv2 as cv
import numpy as np
import cv_bridge
import numpy as np
from scipy.spatial.transform import Rotation

# TODO: Get this into a utilities file or something
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

class GpsPublisher():
    """
    Class for publishing GPS data from Anafi drone.

    Data published:
    - Latitude
    - Longitude
    - Altitude
    - GPS status (fix/no fix)
    - Latitude, longitude and altitude covariance (something wrong with this as it is always 0)
    - GNSS service (GPS)
    """
    def __init__(self, drone, topic_name) -> None:
        self.drone = drone

        self.publisher = rospy.Publisher(
            topic_name, sensor_msgs.msg.NavSatFix, queue_size=1
        )

        self.prev_gps_data = None

    def publish(self):

        while not rospy.is_shutdown():
            gps_data = self._collect_gps_data()
            if gps_data is not None:
                self.publisher.publish(gps_data)
                rospy.sleep(0.99) # Sleep for 990 ms to be ready for new data after 1 second

    def _collect_gps_data(self):
        """
        Get drone GPS position and other data and store this in a ROS message.

        Returns
        -------
        sensor_msgs.msg.NavSatFix
            ROS message containing the GPS data
        """

        gps_data_msg = sensor_msgs.msg.NavSatFix()
        gps_data_msg.header.stamp = rospy.Time.now()

        try: gps_fix = self.drone.get_state(
                olympe_msgs.ardrone3.GPSSettingsState.GPSFixStateChanged
            )["fixed"]
        except RuntimeError:
            rospy.logwarn("Failed to get GPS Fix state. Assuming no fix.")
            gps_fix = 0


        gps_pos = self.drone.get_state(
            olympe_msgs.ardrone3.PilotingState.GpsLocationChanged
        )

        gps_data_msg.status.status = gps_fix
        gps_data_msg.status.service = sensor_msgs.msg.NavSatStatus.SERVICE_GPS
        gps_data_msg.latitude = gps_pos["latitude"]
        gps_data_msg.longitude = gps_pos["longitude"]
        gps_data_msg.altitude = gps_pos["altitude"]

        gps_data_msg.position_covariance = [
            gps_pos["latitude_accuracy"], 0, 0,
            0, gps_pos["longitude_accuracy"], 0,
            0, 0, gps_pos["altitude_accuracy"]
        ]

        gps_data = [
                gps_pos["latitude"],
                gps_pos["longitude"],
                gps_pos["altitude"],
                gps_pos["latitude_accuracy"],
                gps_pos["longitude_accuracy"],
                gps_pos["altitude_accuracy"]
        ]

        if self.prev_gps_data is None:
            self.prev_gps_data = gps_data
        elif self.prev_gps_data == gps_data:
            return None
        else:
            self.prev_gps_data = gps_data

        gps_data_msg.position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        return gps_data_msg

class TelemetryPublisher():
    """
    Class to publish all telemetry data from Anafi except for GPS. This is
    because the other telemetry data are published at 5 Hz from the drone while
    GPS is published at 1 Hz.

    Telemetry published:
    - Attitude in Euler angles
    - Body velocity
    - Relative altitude to ground
    - Gimbal attitude
    - Battery percentage
    - Battery health
    - Batter serial number
    - Flying state
    """

    def __init__(self, drone, topic_name) -> None:
        self.drone = drone

        self.publisher = rospy.Publisher(
            topic_name, anafi_uav_msgs.msg.AnafiTelemetry, queue_size=1
        )

        # Assumed that all telemetry is update at the same time so that it is
        # enough to check if one has changed
        self.prev_attitude = None

        rospy.loginfo("Initialized telemetry publisher")

    def publish(self):

        while not rospy.is_shutdown():
            telemetry = self._collect_telemetry()
            if telemetry is not None:
                self.publisher.publish(telemetry)
                rospy.sleep(0.19) # Sleep for 190 ms to be ready for new data after 200 ms

    def _collect_telemetry(self):
        """
        Collects telemetry from the drone and packs it into a ROS message.

        Returns
        -------
        anafi_uav_msgs.msg.AnafiTelemetry
            ROS message containing the telemetry for the Anafi drone
        """
        telemetry_msg = anafi_uav_msgs.msg.AnafiTelemetry()

        telemetry_msg.header.stamp = rospy.Time.now()

        # Attitude
        att_euler = self.drone.get_state(
            olympe_msgs.ardrone3.PilotingState.AttitudeChanged
        )

        telemetry_msg.roll = att_euler["roll"] # Originally in deg: np.rad2deg
        telemetry_msg.pitch = att_euler["pitch"] # np.rad2deg
        telemetry_msg.yaw = att_euler["yaw"] # np.rad2deg

        # Check if new 5 Hz data has arrived
        if self.prev_attitude is None:
            self.prev_attitude = [telemetry_msg.roll, telemetry_msg.pitch, telemetry_msg.yaw]
        elif self.prev_attitude == [telemetry_msg.roll, telemetry_msg.pitch, telemetry_msg.yaw]:
            return None
        else:
            self.prev_attitude = [telemetry_msg.roll, telemetry_msg.pitch, telemetry_msg.yaw]

        # Body velocity
        speed = self.drone.get_state(
            olympe_msgs.ardrone3.PilotingState.SpeedChanged
        )

        velocity_ned = np.array([speed["speedX"], speed["speedY"], speed["speedZ"]])

        R_ned_to_body = Rx(telemetry_msg.roll).T @ Ry(telemetry_msg.pitch).T @ Rz(telemetry_msg.yaw).T

        # R_ned_to_body = Rotation.from_euler(
        #     "xyz",
        #     [att_euler["roll"], att_euler["pitch"], att_euler["yaw"]],
        #     degrees=False
        # ).as_matrix().T

        velocity_body = R_ned_to_body @ velocity_ned
        # print("V NED: ", velocity_ned)
        # print("V BOD: ", velocity_body)
        # print()

        [
            telemetry_msg.u_body,
            telemetry_msg.v_body,
            telemetry_msg.w_body
        ] = velocity_body

        # Relative altitude
        relative_altitude = self.drone.get_state(
            olympe_msgs.ardrone3.PilotingState.AltitudeChanged
        )["altitude"]

        # Use negative as this corresponds with NED coordinate system
        telemetry_msg.relative_altitude = -relative_altitude

        # Gimbal attitude
        gimbal_attitude = self.drone.get_state(
            olympe_msgs.gimbal.attitude
        )

        # TODO: Check that these actually work
        telemetry_msg.gimbal_roll = gimbal_attitude[0]["roll_absolute"]
        telemetry_msg.gimbal_pitch = gimbal_attitude[0]["pitch_absolute"]
        telemetry_msg.gimbal_yaw = gimbal_attitude[0]["yaw_absolute"]

        # Battery information
        telemetry_msg.battery_percentage = self.drone.get_state(
            olympe_msgs.common.CommonState.BatteryStateChanged
        )["percent"]

        battery_alert = self.drone.get_state(
            olympe_msgs.battery.alert
        )

        no_alert = olympe_enums.battery.alert_level.none
        battery_warnings = []

        try:
            power_alert_level = battery_alert[olympe_enums.battery.alert.power_level]["level"]
        except KeyError:
            rospy.logerr("Failed to get power alert level. The drone interface was started too soon after the simulator/drone. Restart it.")
            power_alert_level = no_alert
        if power_alert_level != no_alert:
            battery_warnings.append(f"low power: {power_alert_level}")
        try:
            cold_alert_level = battery_alert[olympe_enums.battery.alert.too_hot]["level"]
        except KeyError:
            rospy.logerr("Failed to get cold alert level. The drone interface was started too soon after the simulator/drone. Restart it.")
            cold_alert_level = no_alert
        if cold_alert_level != no_alert:
            battery_warnings.append(f"too cold: {cold_alert_level}")
        try:
            hot_alert_level = battery_alert[olympe_enums.battery.alert.too_cold]["level"]
        except KeyError:
            rospy.logerr("Failed to get hot alert level. The drone interface was started too soon after the simulator/drone. Restart it.")
            hot_alert_level = no_alert
        if hot_alert_level != no_alert:
            battery_warnings.append(f"too hot: {hot_alert_level}")

        if len(battery_warnings) == 0:
            telemetry_msg.battery_health = "good"
        else:
            telemetry_msg.battery_health = ", ".join(battery_warnings)

        telemetry_msg.battery_serial_number = self.drone.get_state(olympe_msgs.battery.serial)["serial"]

        # Flying state
        telemetry_msg.flying_state = self.drone.get_state(
            olympe_msgs.ardrone3.PilotingState.FlyingStateChanged
        )["state"].name

        # Publish message
        return telemetry_msg

class CameraPublisher():
    """
    Class to publish front camera of Anafi drone.

    Publishes:
    - Rectified, color image of Anafi front camera
    """

    def __init__(self, drone, topic_name, reject_jitter=True, visualize=True):
        self.drone = drone

        self.reject_jitter = reject_jitter

        self.visualize = visualize
        if self.visualize:
            self.window_name = "Anafi camera"

        # Make directory for data
        script_dir = os.path.dirname(os.path.realpath(__file__))
        today = time.localtime()
        self.image_dir = f"{script_dir}/../../../../../../out/images" \
            f"/{today.tm_year}-{today.tm_mon}-{today.tm_mday}" \
            f"/{today.tm_hour}-{today.tm_min}-{today.tm_sec}"
        pathlib.Path(self.image_dir).mkdir(parents=True, exist_ok=True)
        rospy.loginfo(f"Video stream to {self.image_dir}")

        # Set up video streaming
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.image_dir, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()
        self.image_queue = queue.Queue()
        self.flush_queue_lock = threading.Lock()

        self.drone.set_streaming_output_files(
            h264_data_file=os.path.join(self.image_dir, 'h264_data.264'),
            h264_meta_file=os.path.join(self.image_dir, 'h264_metadata.json'),
        )

        # Callbacks for live processing of frames
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            flush_raw_cb=self.flush_cb,
        )

        # Channel to publish to
        self.publisher = rospy.Publisher(
            topic_name, sensor_msgs.msg.Image, queue_size=1
        )

        rospy.loginfo("Initialized camera publisher")

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

        Parameters
        ----------
        yuv_frame : olympe.VideoFrame
            Video frame
        """
        yuv_frame.ref()
        self.image_queue.put_nowait(yuv_frame)

    def flush_cb(self):
        """
        Function called by Olympe to flush the queue of frames.

        Returns
        -------
        bool
            True when flushing is finished
        """
        with self.flush_queue_lock:
            while not self.image_queue.empty():
                self.image_queue.get_nowait().unref()
        return True

    def publish(self):
        """
        Publishes video stream from Anafi front camera. Blocks, so this function
        Should be run in a separate thread.
        """

        self.drone.start_video_streaming()

        if self.visualize:
            cv.namedWindow(self.window_name, cv.WINDOW_NORMAL)

        while not rospy.is_shutdown():
            image_msg = self._collect_image()
            if image_msg is not None:
                self.publisher.publish(image_msg)

    def _collect_image(self):
        """
        Gets the current image stored in the image queue. If there is no image,
        then this function returns None.

        Returns
        -------
        sensor_msgs.msg.Image
            ROS message containing the image and associated data
        """
        with self.flush_queue_lock:
            if not self.image_queue.empty():
                yuv_frame = self.image_queue.get()

                # Convert yuv frame to OpenCV compatible image array
                info = yuv_frame.info()

                # Skip if image has errors from transmission. This does not work in simulation
                if self.reject_jitter and (info["has_errors"] or info["is_silent"]):
                    yuv_frame.unref()
                    return None

                cv_cvt_color_flag = {
                    olympe.PDRAW_YUV_FORMAT_I420: cv.COLOR_YUV2BGR_I420,
                    olympe.PDRAW_YUV_FORMAT_NV12: cv.COLOR_YUV2BGR_NV12,
                }[info["yuv"]["format"]]
                cv_frame = cv.cvtColor(yuv_frame.as_ndarray(), cv_cvt_color_flag)

                # Create ros message
                image_msg = cv_bridge.CvBridge().cv2_to_imgmsg(cv_frame)
                image_msg.encoding = "bgr8"
                image_msg.header.stamp = rospy.Time.now()

                if self.visualize:
                    cv.imshow(self.window_name, cv_frame)
                    cv.waitKey(1)

                yuv_frame.unref()
                return image_msg
            else:
                return None


def main():
    drone_ip = "10.202.0.1"
    drone = olympe.Drone(drone_ip)
    drone.logger.setLevel(40)
    drone.connect()

    camera_publisher = CameraPublisher(drone, "/image_test")
    camera_publisher.publish()

if __name__ == "__main__":
    main()