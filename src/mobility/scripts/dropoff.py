#! /usr/bin/env python

from __future__ import print_function

import sys 
import math 
import rospy 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie

def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)

    swarmie.putdown() 
    swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(math.pi, ignore=Obstacle.IS_SONAR)
    
    # Recalibrate the home location because we're here. 
    odom_location = swarmie.get_odom_location()
    swarmie.set_home_odom_location(odom_location)

if __name__ == '__main__' : 
    main()
