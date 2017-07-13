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

    # Enable stop on all obstacles.
    swarmie.all_obstacles() 
    
    while not rospy.is_shutdown() : 
        rospy.loginfo("Wandering...")
        
        # Drive to a random location.   
        rval = swarmie.drive(2, random.gauss(0, math.pi/8))
        
        # Failed because we saw an obstacle with sonar.
        if rval == MoveResult.OBSTACLE_SONAR :
            rospy.loginfo("I see a wall!")
            swarmie.set_obstacles(Obstacle.IS_VISION)
            rval = swarmie.drive(0, random.gauss(math.pi, math.pi/8))
            swarmie.all_obstacles()

        # Failed because we saw a tag.        
        elif rval == MoveResult.OBSTACLE_VISION : 
            rospy.loginfo("I see a tag!")
            pass 
        
    return 0

if __name__ == '__main__' : 
    main()

