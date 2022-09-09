#!/usr/bin/env python3

from subprocess import PIPE, Popen
import os
import sys
import yaml
import rospy
import geometry_msgs.msg

class SphinxRosBridge():

    def __init__(self):
        """
        Bridge between Sphinx simulator and ROS, which publishes the ground
        truth pose of the drone and the helipad.
        """
        rospy.init_node("sphinx_ros_bridge", anonymous=False)

        script_dir = os.path.dirname(os.path.realpath(__file__))

        config_file = rospy.get_param("~config_file")

        try:
            with open(f"{script_dir}/../config/{config_file}") as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            rospy.logerr(f"Failed to load config: {e}")
            sys.exit()

        self.anafi_pose_publisher = rospy.Publisher(
            self.config["topics"]["drone_pose_ned"]["sim"],
            geometry_msgs.msg.PoseStamped, queue_size=10
        )
        self.helipad_pose_publisher = rospy.Publisher(
            self.config["topics"]["helipad_pose_ned"]["sim"],
            geometry_msgs.msg.PoseStamped, queue_size=10
        )

        self.data = {
        "timestamp": {
            "sec": None,
            "nsec": None
        },
        "anafi": {
            "position": {
                "x": None,
                "y": None,
                "z": None
            },
            "orientation": {
                "x": None,
                "y": None,
                "z": None,
                "w": None
            },
        },
        "helipad": {
            "position": {
                "x": None,
                "y": None,
                "z": None
            },
            "orientation": {
                "x": None,
                "y": None,
                "z": None,
                "w": None
            },
        }
    }

    def start(self):
        """
        This function will block and run infinitely until either the ROS-node is
        killed or the Sphinx simulator is killed.
        """
        rospy.loginfo("Starting Sphinx-Ros-Bridge")

        while not rospy.is_shutdown():

            ON_POSIX = 'posix' in sys.builtin_module_names
            command = "parrot-gz topic -e /gazebo/land/pose/info | grep -E -A 12 " \
                "'time" \
                "|name: \"helipad\"" \
                "|name: \"anafi4k\"'"

            with Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True) as p:

                timestamp_data_gather_done = False

                timestamp_index = 0
                anafi_index = 0
                helipad_index = 0

                # Parse output
                for line in iter(p.stdout.readline, b''):
                    line = line.decode("utf-8")

                    # First find timestamp as this will be the first value
                    if timestamp_index > 0 and timestamp_index <= 3:
                        # Gather data
                        if timestamp_index < 3:
                            line = line.split()
                            self.data["timestamp"][line[0][:-1]] = int(line[1])
                            timestamp_index += 1
                        # Timestamp index is 3 and there is no more data to gather
                        else:
                            timestamp_index = 0
                            timestamp_data_gather_done = True

                    # Found start of a new entry -> pubish poses from previous entry
                    elif "time" in line:
                        self._publish_poses()
                        timestamp_data_gather_done = False
                        timestamp_index = 1

                    # Make sure to only search for data after a timestamp is found to make
                    # sure that the data belong to the correct timestamp
                    if timestamp_data_gather_done == False:
                        continue

                    # Find Anafi data
                    if anafi_index > 0 and anafi_index <= 12:
                        # Skip over two first entries before position data
                        if anafi_index < 3:
                            anafi_index += 1
                        # Parse anafi position
                        elif anafi_index >= 3 and anafi_index <= 5:
                            line = line.split()
                            self.data["anafi"]["position"][line[0][0]] = float(line[1])
                            anafi_index += 1
                        # Skip over two first entries before orientation data
                        elif anafi_index < 8:
                            anafi_index += 1
                        # Parse anafi orientation
                        elif anafi_index >= 8 and anafi_index <= 11:
                            line = line.split()
                            self.data["anafi"]["orientation"][line[0][0]] = float(line[1])
                            anafi_index += 1
                        # Anafi index is 12 and there is no more data to gather
                        else:
                            anafi_index = 0
                    elif "anafi" in line:
                        anafi_index = 1

                    # Find helipad data
                    if helipad_index > 0 and helipad_index <= 12:
                        # Skip over two first entries before position data
                        if helipad_index < 3:
                            helipad_index += 1
                        # Parse helipad position
                        elif helipad_index >= 3 and helipad_index <= 5:
                            line = line.split()
                            self.data["helipad"]["position"][line[0][0]] = float(line[1])
                            helipad_index += 1
                        # Skip over two first entries before orientation data
                        elif helipad_index < 8:
                            helipad_index += 1
                        # Parse helipad orientation
                        elif helipad_index >= 8 and helipad_index <= 11:
                            line = line.split()
                            self.data["helipad"]["orientation"][line[0][0]] = float(line[1])
                            helipad_index += 1
                        # helipad index is 12 and there is no more data to gather
                        else:
                            helipad_index = 0
                    elif "helipad" in line:
                        helipad_index = 1

    def _publish_poses(self):
        anafi_pose = self._pack_message("anafi")
        if anafi_pose is not None:
            self.anafi_pose_publisher.publish(anafi_pose)

        helipad_pose = self._pack_message("helipad")
        if helipad_pose is not None:
            self.helipad_pose_publisher.publish(helipad_pose)

    def _get_helipad_pose_from_model_info(self):
        """
        Function to get helipad pose from the model information from Gazebo.
        This was first created as a workaround since the Gazebo-topic stopped
        publishing after the Anafi did not make contact with the landing pad
        anymore, but this method was much slower. Left in in case it will be
        needed for something in the future.
        """
        command = "parrot-gz model -m helipad -i | head -14 | tail -9"

        p = Popen(command, stdout=PIPE, shell=True)

        self.data["helipad"]["position"]["x"] = float(p.stdout.readline().decode("utf-8").split()[1])
        self.data["helipad"]["position"]["y"] = float(p.stdout.readline().decode("utf-8").split()[1])
        self.data["helipad"]["position"]["z"] = float(p.stdout.readline().decode("utf-8").split()[1])
        p.stdout.readline()
        p.stdout.readline()
        self.data["helipad"]["orientation"]["x"] = float(p.stdout.readline().decode("utf-8").split()[1])
        self.data["helipad"]["orientation"]["y"] = float(p.stdout.readline().decode("utf-8").split()[1])
        self.data["helipad"]["orientation"]["z"] = float(p.stdout.readline().decode("utf-8").split()[1])
        self.data["helipad"]["orientation"]["w"] = float(p.stdout.readline().decode("utf-8").split()[1])

    def _pack_message(self, model):

        # Return None if data has not been initialized yet
        if None in self.data["timestamp"].values() or None in self.data[model]["position"].values():
            return None

        model_pose = geometry_msgs.msg.PoseStamped()

        # model_pose.header.stamp.secs = self.data["timestamp"]["sec"]
        # model_pose.header.stamp.nsecs = self.data["timestamp"]["nsec"]
        # Using rospy.Time.now() will give a timestamp consistent with the
        # timestamps from all the other packages. The timestamp from the
        # Gazebo-topic gives only the simulation time. The timestamp is left in
        # in case it will be useful in the future
        model_pose.header.stamp = rospy.Time.now()

        # Convert East-North-Up (ENU) coordinates to North-East-Down (NED)
        model_pose.pose.position.x = self.data[model]["position"]["y"]
        model_pose.pose.position.y = self.data[model]["position"]["x"]
        model_pose.pose.position.z = -self.data[model]["position"]["z"]

        model_pose.pose.orientation.x = self.data[model]["orientation"]["x"]
        model_pose.pose.orientation.y = self.data[model]["orientation"]["y"]
        model_pose.pose.orientation.z = self.data[model]["orientation"]["z"]
        model_pose.pose.orientation.w = self.data[model]["orientation"]["w"]

        return model_pose

def main():
    bridge = SphinxRosBridge()
    bridge.start()

if __name__ == "__main__":
    main()
