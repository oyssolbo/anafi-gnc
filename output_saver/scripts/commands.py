import rospy
import geometry_msgs.msg
from olympe_bridge.msg import AttitudeCommand, CameraCommand, MoveByCommand, MoveToCommand

from generic_output_saver import GenericOutputSaver

import numpy as np

class AnafiAttitudeCMDSaver(GenericOutputSaver):
    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, AttitudeCommand, self.attitude_cmd_cb)

    def attitude_cmd_cb(self, msg: AttitudeCommand):

        output = [
            msg.header.stamp.to_sec(),
            msg.roll * 180 / np.pi,
            msg.pitch * 180 / np.pi,
            msg.yaw * 180 / np.pi,
            msg.gaz
        ]

        self._save_output(output)

