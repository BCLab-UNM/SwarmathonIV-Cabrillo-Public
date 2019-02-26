#! /usr/bin/env python

from __future__ import print_function

import sys
import math 
import rospy 

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult
from mobility.srv import QueueRemove, QueueRemoveRequest

from mobility.swarmie import swarmie, Location

def main(**kwargs):
    remove_from_queue = rospy.ServiceProxy('start_queue/remove', QueueRemove)

    # During a normal startup the rover will be facing the center and
    # close to the nest. But there's no guarantee where we will be if 
    # mobility crashes and is forced to restart. This checks to see 
    # if we've set home location prviously.
    
    # Assume the starting position is facing the center.
    # This should be valid by contest rules. 
    #
    # Drive closer until we can see the center. 
    try:
        swarmie.drive(1) 
    except: 
        # This could happen if we bump into another rover. 
        # Let's just call it good. 
        pass

    current_location = swarmie.get_odom_location()
    current_pose = current_location.get_pose()
    home_odom = Location(current_location.Odometry)

    # project home_odom location 50cm in front of rover's current location
    home_odom.Odometry.pose.pose.position.x = (
        current_pose.x + 0.5 * math.cos(current_pose.theta)
    )
    home_odom.Odometry.pose.pose.position.y = (
        current_pose.y + 0.5 * math.sin(current_pose.theta)
    )
    swarmie.set_home_odom_location(home_odom)

    swarmie.turn(
        math.pi,
        ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR
    )

    remove_from_queue(QueueRemoveRequest(rover_name=swarmie.rover_name,
                                         notify_others=True))

    return 0 

if __name__ == '__main__' : 
    swarmie.start(node_name='init')
    sys.exit(main())
