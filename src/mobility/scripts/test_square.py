#! /usr/bin/env python

from __future__ import print_function

import sys 
import math 
import rospy 

from swarmie_msgs.msg import Obstacle
from geometry_msgs.msg import Pose2D, Point

from mobility.swarmie import Swarmie

def dumb_square(swarmie, distance):
    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

def smart_square(swarmie, distance):    
    # Compute a square based on the current heading. 
    start_pose = swarmie.get_odom_location().get_pose()
    start = Point()
    start.x = start_pose.x 
    start.y = start_pose.y 
    
    sq1 = Point()
    sq1.x = start.x + distance * math.cos(start_pose.theta)
    sq1.y = start.y + distance * math.sin(start_pose.theta)
    
    sq2 = Point()
    sq2.x = sq1.x + distance * math.cos(start_pose.theta + math.pi/2)
    sq2.y = sq1.y + distance * math.sin(start_pose.theta + math.pi/2)
    
    sq3 = Point()
    sq3.x = sq2.x + distance * math.cos(start_pose.theta + math.pi)
    sq3.y = sq2.y + distance * math.sin(start_pose.theta + math.pi)

    swarmie.drive_to(sq1, ignore=Obstacle.IS_VISION)
    swarmie.drive_to(sq2, ignore=Obstacle.IS_VISION)
    swarmie.drive_to(sq3, ignore=Obstacle.IS_VISION)
    swarmie.drive_to(start, ignore=Obstacle.IS_VISION)
    
    swarmie.set_heading(start_pose.theta)
    
    
def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 3 :
        print ('usage:', sys.argv[0], '<rovername> <distance>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    distance = float(sys.argv[2])

    smart_square(swarmie, distance)
    
if __name__ == '__main__' : 
    main()
