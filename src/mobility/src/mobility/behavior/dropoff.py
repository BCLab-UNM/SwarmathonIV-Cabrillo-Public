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


def main(**kwargs):
    """Do dropoff behavior."""
    # Move the wrist down, but not so far down that the resource hits the ground.
    swarmie.wrist_middle()

    # TODO: What should find_home_corner() do when it's called with no home tags
    #  in view? Does this happen very often?
    find_home_corner()

    home_origin = PoseStamped()
    home_origin.header.frame_id = 'home'

    # TODO: what do we do if this raises a tf exception?
    home_odom = swarmie.transform_pose('odom', home_origin)  # type: PoseStamed

    # Move the wrist up so the resource won't hit the home plate.
    swarmie.set_wrist_angle(.3)
    swarmie.drive_to(home_odom.pose.position, claw_offset=0.5,
                     ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)

    swarmie.set_wrist_angle(.7)
    # TODO: Are the sleep statements here and a few lines below useful for
    #  giving the wrist and fingers time to move?
    # rospy.sleep(.4)

    if swarmie.simulator_running():
        swarmie.fingers_open()
    else:
        swarmie.set_finger_angle(1)

    # rospy.sleep(.4)
    swarmie.set_wrist_angle(0)
    swarmie.drive(-.5, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    return 0


if __name__ == '__main__' :
    swarmie.start(node_name='dropoff')
    sys.exit(main())
