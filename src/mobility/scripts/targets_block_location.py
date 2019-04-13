#! /usr/bin/env python
"""Get the targets buffer from swarmie and publish a PoseArray of the estimated
block locations (tags translated ~2.5cm toward the center of a block).
"""
from __future__ import print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List
except ImportError:
    pass

import rospy
import tf
import tf.transformations

from apriltags2to1.msg import AprilTagDetection
from geometry_msgs.msg import Pose, PoseArray

from mobility.swarmie import swarmie
from mobility.utils import block_pose


def publish(pub, nearest_block_pub, detections):
    # type: (rospy.Publisher, rospy.Publisher, List[AprilTagDetection]) -> None
    """Publish PoseArrays of block poses and the nearest block location."""
    now = rospy.Time.now()

    array = PoseArray()
    array.header.frame_id = swarmie.rover_name + '/camera_link'
    array.header.stamp = now
    array.poses = []

    for detection in detections:
        try:
            ps = block_pose(detection)
            array.poses.append(ps.pose)

        except tf.Exception:
            pass

    nearest = PoseArray()
    nearest.header.frame_id = swarmie.rover_name + '/odom'
    nearest.header.stamp = now

    block = swarmie.get_nearest_block_location(targets_buffer_age=0.5)
    if block is not None:
        nearest.poses = [Pose(position=block)]

    nearest_block_pub.publish(nearest)
    pub.publish(array)


def main():
    block_pub = rospy.Publisher('targets/blocks',
                                PoseArray, queue_size=10)
    nearest_block_pub = rospy.Publisher('targets/nearest_block',
                                        PoseArray, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        target_tags = swarmie.get_targets_buffer(age=0.5, id=0)  # type: List[AprilTagDetection]
        publish(block_pub, nearest_block_pub, target_tags)
        rate.sleep()


if __name__ == '__main__':
    swarmie.start(node_name='targets_block_location')
    main()
