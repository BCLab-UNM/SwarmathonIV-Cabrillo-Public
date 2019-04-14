"""Common utility functions.

Please document any functions added to this file. Google-style docstrings are
preferred. See below link for examples:

https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html
"""
from __future__ import division, print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List
except ImportError:
    pass

import tf.transformations

from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry

from apriltags2to1.msg import AprilTagDetection


def sort_tags_left_to_right(detections):
    # type: (List[AprilTagDetection]) -> List[AprilTagDetection]
    """Sort tags in view from left to right (by their x position in the camera
    frame). Removes/ignores tags close enough in the camera to likely be a block
    in the claw.

    Args:
        detections: The list of detections. This should only contain the type of
            tag you care about sorting. (i.e. filter unwanted tag id's
            out first).

    Returns:
        The sorted list of AprilTagDetections in view. This will be empty if no
            tags are in view.
    """
    BLOCK_IN_CLAW_DIST = 0.22  # meters

    return sorted(
        filter(lambda x: x.pose.pose.position.z > BLOCK_IN_CLAW_DIST,
               detections),
        key=lambda x: x.pose.pose.position.x
    )

def is_moving(odom, lin_x_thld=0.1, ang_z_thld=0.2):
    # type: (Odometry, float, float) -> bool
    """Given an Odometry message, return True if the rover is moving.

    Args:
        lin_x_thld: The threshold linear x velocity.
        ang_z_thld: The threshold angular z velocity.
    """
    return (abs(odom.twist.twist.linear.x) > 0.1
            or abs(odom.twist.twist.angular.z) > 0.2)


def block_pose(detection, block_size=0.05):
    # type: (AprilTagDetection, float) -> PoseStamped
    """Given a tag detection (id == 0), return the block's pose. The block pose
    has the same orientation as the tag detection, but it's position is
    translated to be at the cube's center.

    Args:
        detection: The AprilTagDetection.
        block_size: The block's side length in meters.
    """
    transform = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix(
            [detection.pose.pose.position.x,
             detection.pose.pose.position.y,
             detection.pose.pose.position.z]
        ),
        tf.transformations.quaternion_matrix(
            [detection.pose.pose.orientation.x,
             detection.pose.pose.orientation.y,
             detection.pose.pose.orientation.z,
             detection.pose.pose.orientation.w]
        ),
        tf.transformations.translation_matrix(
            [0, 0, -block_size / 2]
        )
    )

    t = tf.transformations.translation_from_matrix(transform)
    q = tf.transformations.quaternion_from_matrix(transform)

    ps = PoseStamped()
    ps.header.frame_id = detection.pose.header.frame_id
    ps.header.stamp = detection.pose.header.stamp
    ps.pose.position = Point(*t)
    ps.pose.orientation = Quaternion(*q)

    return ps


def block_detection(detection, block_size=0.5):
    # type: (AprilTagDetection, float) -> AprilTagDetection
    """Given a tag detection (id == 0), return a new AprilTagDetection whose
    pose is the block's pose. The block pose has the same orientation as the tag
    detection, but it's position is translated to be at the cube's center.

    Args:
        detection: The AprilTagDetection.
        block_size: The block's side length in meters.
    """
    det = AprilTagDetection()
    det.id = detection.id
    det.size = detection.size
    det.pose = block_pose(detection, block_size)

    return det
