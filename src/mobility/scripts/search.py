#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from std_msgs.msg import String

from mobility.msg import MoveResult, Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def wander():
    global swarmie
    try :
        rospy.loginfo("Wandering...")
        swarmie.turn(random.gauss(0, math.pi/8))
        swarmie.drive(2)

        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround()


def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
            
    try: 
        for move in range(10) :
            if rospy.is_shutdown : 
                exit(-1)
            try:
                wander()
            
            except HomeException : 
                # TODO: Recalibrate the map.
                print ("I saw home!")
                turnaround()
                
    except TagException : 
        print("I found a tag!")
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

