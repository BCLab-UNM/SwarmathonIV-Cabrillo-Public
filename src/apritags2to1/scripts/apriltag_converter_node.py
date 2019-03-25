#! /usr/bin/env python
"""ROS node converting apriltags2_ros/AprilTagDetectionArrays to
apriltag_ros/AprilTagDetectionArrays.
"""
from __future__ import print_function

import rospy

from apriltags2to1 import AprilTagConverter


def main():
    rospy.init_node('apriltag2to1_converter')

    converter = AprilTagConverter()
    rospy.spin()


if __name__ == '__main__':
    main()
