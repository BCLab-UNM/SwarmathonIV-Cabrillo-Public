#! /usr/bin/env python
"""Drop off a cube inside of home."""
from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D, PoseStamped
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import swarmie, PathException
from mobility.behavior.find_home_corner import find_home_corner


def dropoff(approx_home=False):
    """Drive in and drop off a cube.

    Args:
        approx_home: Whether we should use the approximate home location as the
        goal point to drive toward.
    """
    if approx_home:
        rospy.logwarn("Performing dropoff using approximate home location.")

    home_odom = swarmie.get_home_odom_location(approx=approx_home)

    # Move the wrist up so the resource won't hit the home plate.
    swarmie.set_wrist_angle(.3)

    if swarmie.simulator_running():
        claw_offset = 0.5
    else:
        claw_offset = 0.6

    swarmie.drive_to(home_odom, claw_offset=claw_offset,
                     ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

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
    use_approx_home = True
    try:
        if find_home_corner():
            use_approx_home = False
    except PathException as e:
        swarmie.print_infoLog('<font color="red">{}</font>'.format(e.status))
        rospy.logwarn(e.status)

    dropoff(use_approx_home)

    exit_home()

    return 0


if __name__ == '__main__' :
    swarmie.start(node_name='dropoff')
    sys.exit(main())
