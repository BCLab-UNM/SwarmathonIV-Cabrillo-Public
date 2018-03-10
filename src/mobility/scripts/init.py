#! /usr/bin/env python

from __future__ import print_function

import sys 
import math 
import rospy 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, Location

def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)

    # During a normal startup the rover will be facing the center and
    # close to the nest. But there's no guarantee where we will be if 
    # mobility crashes and is forced to restart. This checks to see 
    # if we've set home location prviously.
    
    if swarmie.has_home_gps_location() : 
        # Oops. This must be a restart. Don't assume we're near the 
        # nest. Just exit and hope... 
        exit(0)

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
    
    # Wait up to two minutes for an good GPS fix. 
    # TODO: Can we ever get a fix this good in real life? 
    home = swarmie.wait_for_fix(distance=3, time=120) 

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

    if home is None : 
        swarmie.print_infoLog(rovername + ' failed to get a GPS fix!')        
    else:
        swarmie.set_home_gps_location(home)

    swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    
if __name__ == '__main__' : 
    main()
