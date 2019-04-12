"""Common utility functions.

Please document any functions added to this file. Google-style docstrings are
preferred. See below link for examples:

https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html
"""
from __future__ import print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List
except ImportError:
    pass

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
