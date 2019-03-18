"""Convert apriltags2_ros/AprilTagDetectionArrays to
apriltag_ros/AprilTagDetectionArrays.
"""
from __future__ import print_function

import rospy

from apriltags2_ros.msg import AprilTagDetection as AprilTag2Detection
from apriltags2_ros.msg import AprilTagDetectionArray as AprilTag2DetectionArray

from apriltags2to1.msg import AprilTagDetection, AprilTagDetectionArray


class AprilTagConverter:

    def __init__(self):
        self._camera_frame_id = rospy.get_param('~camera_frame_id', 'camera_link')

        self._targets_pub = rospy.Publisher('targets/out',
                                            AprilTagDetectionArray,
                                            queue_size=10)

        self._targets_sub = rospy.Subscriber('targets/in',
                                             AprilTag2DetectionArray,
                                             self._targets_cb, queue_size=10)

    def _targets_cb(self, msg):
        # type: (AprilTag2DetectionArray) -> None
        """Re-publish each message in the format of an
        apriltags_ros/AprilTagDetectionArray.
        """
        msg_out = AprilTagDetectionArray()

        for detection_in in msg.detections:  # type: AprilTag2Detection
            detection_out = AprilTagDetection()

            detection_out.id = detection_in.id[0]
            detection_out.size = detection_in.size[0]

            detection_out.pose.header = msg.header
            detection_out.pose.header.frame_id = self._camera_frame_id
            detection_out.pose.pose = detection_in.pose.pose.pose

            msg_out.detections.append(detection_out)

        self._targets_pub.publish(msg_out)
