#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from mobility.srv import Core
from mobility.msg import MoveResult
from obstacle_detection.srv import DetectionMask 
from obstacle_detection.msg import Obstacle 
from std_msgs.msg import String

from mobility.swarmie import Swarmie 

'''Searcher node.''' 

def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)

    swarmie.set_obstacle_mask(Obstacle.IS_SONAR | Obstacle.IS_VISION)
    while not rospy.is_shutdown() : 
        rval = swarmie.drive(2, random.gauss(0, math.pi/8))
        print ("Drive returns:", rval)
        if rval & MoveResult.OBSTACLE_SONAR != 0 :
            swarmie.set_obstacle_mask(0)
            swarmie.drive(1, math.pi)
            swarmie.set_obstacle_mask(Obstacle.IS_VISION | Obstacle.IS_SONAR)
        elif rval & MoveResult.OBSTACLE_VISION != 0 : 
            return 0
        
    return 0

if __name__ == '__main__' : 
    main()

