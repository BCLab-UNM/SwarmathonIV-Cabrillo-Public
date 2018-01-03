#! /usr/bin/env python

from __future__ import print_function

import sys 
import math 
import rospy 
import angles 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, HomeException

def drive_straight_home() :
    global swarmie 
    global rovername 
    
    # Use GPS to figure out about where we are. 
    # FIXME: We need to hanlde poor GPS fix. 
    loc = swarmie.wait_for_fix(distance=4, time=60).get_pose()
    home = swarmie.get_home_gps_location()

    
    dist = math.hypot(loc.y - home['y'], 
                      loc.x - home['x'])
    
    angle = angles.shortest_angular_distance(loc.theta, 
                                             math.atan2(home['y'] - loc.y,
                                                        home['x'] - loc.x))
    
    swarmie.turn(angle, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    swarmie.drive(dist, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    
def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)

    try : 
        while not rospy.is_shutdown() : 
            drive_straight_home()
    except HomeException as e: 
        # Found home!
        exit(0)
    
if __name__ == '__main__' : 
    main()
