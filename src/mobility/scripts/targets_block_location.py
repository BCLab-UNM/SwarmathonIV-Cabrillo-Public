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
from mobility.utils import block_detection, filter_detections


def publish(blocks_pub,  # type: rospy.Publisher
            blocks_filtered_pub,  # type: rospy.Publisher
            nearest_block_pub,  # type: rospy.Publisher
            detections  # type: List[AprilTagDetection]
            ):
    # type: (...) -> None
    """Publish PoseArrays of block poses and the nearest block location."""
    now = rospy.Time.now()

    blocks = PoseArray()
    blocks.header.frame_id = swarmie.rover_name + '/camera_link'
    blocks.header.stamp = now
    blocks.poses = []

    blocks_filtered = PoseArray()
    blocks_filtered.header.frame_id = swarmie.rover_name + '/camera_link'
    blocks_filtered.header.stamp = now
    blocks_filtered.poses = []

    block_detections = []  # type: List[AprilTagDetection]

    for detection in detections:
        try:
            block_det = block_detection(detection, swarmie.block_size)
            block_detections.append(block_det)
            blocks.poses.append(block_det.pose.pose)

        except tf.Exception:
            pass

    block_detections = filter_detections(
        block_detections,
        dist=swarmie.block_size - 0.01
    )
    blocks_filtered.poses = [d.pose.pose for d in block_detections]

    nearest = PoseArray()
    nearest.header.frame_id = swarmie.rover_name + '/odom'
    nearest.header.stamp = now

    block = swarmie.get_nearest_block_location(targets_buffer_age=0.5)
    if block is not None:
        nearest.poses = [Pose(position=block)]

    nearest_block_pub.publish(nearest)
    blocks_pub.publish(blocks)
    blocks_filtered_pub.publish(blocks_filtered)


def main():
    blocks_pub = rospy.Publisher('targets/blocks',
                                 PoseArray, queue_size=10)
    blocks_filtered_pub = rospy.Publisher('targets/blocks/filtered',
                                          PoseArray, queue_size=10)
    nearest_block_pub = rospy.Publisher('targets/nearest_block',
                                        PoseArray, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        target_tags = swarmie.get_targets_buffer(age=0.5, id=0)  # type: List[AprilTagDetection]
        publish(blocks_pub, blocks_filtered_pub, nearest_block_pub, target_tags)
        rate.sleep()


if __name__ == '__main__':
    swarmie.start(node_name='targets_block_location')
    main()
