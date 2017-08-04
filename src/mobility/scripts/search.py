#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from std_msgs.msg import String

from obstacle_detection.msg import Obstacle 
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Searcher node.''' 

def turnaround(swarmie): 
    swarmie.drive(0, random.gauss(math.pi/2, math.pi/4), Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def wander(swarmie):
    try :
        rospy.loginfo("Wandering...")
        swarmie.drive(2, random.gauss(0, math.pi/8))

        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround(swarmie)


def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
            
    try: 
        while not rospy.is_shutdown() : 
            try:
                wander(swarmie)
            
            except HomeException : 
                # TODO: Recalibrate the map.
                print ("I saw home!")
                turnaround(swarmie)
                
    except TagException : 
        print("I found a tag!")
    
    return 0

if __name__ == '__main__' : 
    main()

