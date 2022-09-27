
import rospy
import geometry_msgs.msg
from anafi_uav_msgs.msg import AnafiTelemetry, Heading, PointWithCovarianceStamped, EulerPose, EkfOutput, PoseStampedEuler

from generic_output_saver import GenericOutputSaver

class AnafiRawDataSaver(GenericOutputSaver):
    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, AnafiTelemetry, self._anafi_raw_data_cb)

    def _anafi_raw_data_cb(self, msg: AnafiTelemetry):

        output = [
            msg.header.stamp.to_sec(),
            msg.vx,
            msg.vy,
            msg.vz,
            msg.roll,
            msg.pitch,
            msg.yaw
        ]

        self._save_output(output)

class DnnCvPositionSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, geometry_msgs.msg.PointStamped, self._dnn_cv_position_cb)

    def _dnn_cv_position_cb(self, msg: geometry_msgs.msg.PointStamped):

        self._save_output([
            msg.header.stamp.to_sec(),
            msg.point.x,
            msg.point.y,
            msg.point.z
        ])

class DnnCvHeadingSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, Heading, self._dnn_cv_heading_cb)

    def _dnn_cv_heading_cb(self, msg: Heading):

        self._save_output([
            msg.header.stamp.to_sec(),
            msg.heading
        ])

class EkfPositionSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, PointWithCovarianceStamped, self._ekf_position_estimate_cb)

    def _ekf_position_estimate_cb(self, msg: PointWithCovarianceStamped):

        output = [
            msg.header.stamp.to_sec(),
            msg.position.x,
            msg.position.y,
            msg.position.z,
        ]

        output.extend(msg.covariance)

        self._save_output(output)



class TcvDataSaver(GenericOutputSaver):
    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, EulerPose, self._tcv_pose_cb)

    def _tcv_pose_cb(self, msg):

        self._save_output([
            msg.header.stamp.to_sec(),
            msg.x,
            msg.y,
            msg.z,
            msg.phi,
            msg.theta,
            msg.psi
        ])

class EkfDataSaver(GenericOutputSaver):

    def __init__(self, config, base_dir, output_category, output_type, environment):
        super().__init__(config, base_dir, output_category, output_type, environment)

        rospy.Subscriber(self.topic_name, EkfOutput,
            self._ekf_output_cb
        )

    def _ekf_output_cb(self, msg):
        output = [
            msg.header.stamp.to_sec(),
            msg.x,
            msg.y,
            msg.z,
            msg.psi,
            msg.v_x,
            msg.v_y,
            msg.v_z
        ]

        output.extend(msg.covariance)

        self._save_output(output)
