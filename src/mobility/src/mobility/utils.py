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

from apriltags_ros.msg import AprilTagDetection


def sort_tags_left_to_right(detections, id=0):
    # type: (List[AprilTagDetection], int) -> List[AprilTagDetection]
    """Sort tags in view from left to right (by their x position in the camera
    frame). Removes/ignores tags close enough in the camera to likely be a block
    in the claw.

    Args:
        detections: The list of detections.
        id: The tag id to filter by before sorting. (0 - target, 256 - home)

    Returns:
        The sorted list of AprilTagDetections in view. This will be empty if no
            tags are in view.
    """
    BLOCK_IN_CLAW_DIST = 0.22  # meters
    sorted_detections = []

    for detection in detections:
        if (detection.id == id and
                detection.pose.pose.position.z > BLOCK_IN_CLAW_DIST):
            sorted_detections.append(detection)

    return sorted(sorted_detections, key=lambda x: x.pose.pose.position.x)



