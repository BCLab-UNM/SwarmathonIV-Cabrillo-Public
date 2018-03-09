#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from swarmie_msgs.msg import Obstacle

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
    swarmie.fingers_open()
    swarmie.wrist_middle()

    try: 
        for move in range(30) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                wander()
            
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location()
                swarmie.set_home_odom_location(odom_location)
                turnaround()
                
    except TagException : 
        print("I found a tag!")
        # Let's drive there to be helpful.
        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

