#! /usr/bin/env python
"""Drop off a cube inside of home."""
from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D, PoseStamped
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import swarmie
from mobility.behavior.find_home_corner import find_home_corner


def find_home_pose():
    # type: () -> PoseStamped
    """Find home's pose in the odom frame.

    Returns:
        The home plate's pose.

    Raises:
        tf.Exception if the transform fails.
    """
    home_origin = PoseStamped()
    home_origin.header.frame_id = 'home'

    # TODO: what do we do if this raises a tf exception?
    return swarmie.transform_pose('odom', home_origin)


def dropoff():
    """Drive in and drop off a cube."""
    home_odom = find_home_pose()

    # Move the wrist up so the resource won't hit the home plate.
    swarmie.set_wrist_angle(.3)
    swarmie.drive_to(home_odom.pose.position, claw_offset=0.5,
                     ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)

    swarmie.set_wrist_angle(.7)
    # Wait a moment for the wrist to move down before opening the fingers next.
    rospy.sleep(.4)

    if swarmie.simulator_running():
        swarmie.fingers_open()
    else:
        # Open the fingers just enough to release the block. This only works in
        # the real world because the simulator's gripper plugin makes a physical
        # attachment between the finger and the block, and it doesn't remove the
        # attachment unless the fingers open wider than this angle.
        swarmie.set_finger_angle(1)

    # Wait a moment for the fingers to open before moving the wrist back up.
    rospy.sleep(.4)
    swarmie.wrist_up()


def exit_home():
    """After we drop of a cube, back up out of home and turn away from home."""
    # TODO: Is this simple function call reliable enough during congested rounds?
    #  it's very bad if the rover don't make it fully back out of the home ring.
    swarmie.drive(-.5, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(-8 * math.pi / 9, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)


def main(**kwargs):
    """Do dropoff behavior."""
    # Move the wrist down, but not so far down that the resource hits the
    # ground. This is useful so the cube and claw don't obscure the camera's
    # field of view while looking for the home corner.
    swarmie.wrist_middle()

    # TODO: What should find_home_corner() do when it's called with no home tags
    #  in view? Does this happen very often?
    find_home_corner()

    dropoff()

    exit_home()

    return 0


if __name__ == '__main__' :
    swarmie.start(node_name='dropoff')
    sys.exit(main())
