#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException
from OpenGL.raw.GL.ATI.pn_triangles import glInitPnTrianglesATI

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

def get_sonar_directional():
    """Returns 3 boolean values that indicate which sonar are going off."""
    global swarmie
    unmasked = swarmie.get_obstacle_condition()
    left = unmasked & 1 == Obstacle.SONAR_LEFT
    right = unmasked & 2 == Obstacle.SONAR_RIGHT
    center = unmasked & 4 == Obstacle.SONAR_CENTER
    return left, center, right

# def random_and_return():
#     "Goes in a random direction between 160 and 200 degrees (away from home) and comes back."
#     try:
#         swarmie.turn(random.gauss((math.pi * 8 / 9)/(math.pi * 10 / 9)))
#         swarmie.drive(9999); # go till you hit an obstacle
#     except ObstacleException:
#         swarmie.turn(math.pi, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
#         swarmie.

def zigzag(distance_per_triangle, triangles):
    """Makes the rover zig-zag back and forth for the entered distance * triangles."""
    global swarmie
    triangle_leg_distance = distance_per_triangle/math.sqrt(2)
    on_left_triangle = True
    if (triangles % 2 == 1):
        on_left_triangle = False
    try:
        if (triangles <= 0):
            print("triangles <= 0, the rover shall not move.")
        else:
         #make the first turn
            if on_left_triangle:
                swarmie.turn(math.pi / 4)
            else:
                swarmie.turn((-1 * math.pi) / 4)
        #while we still have triangles to move
        while (triangles > 0):
            #or else, drive a (45, 45, 90) triangle
            print(triangles)
            triangles = triangles - 1
            swarmie.drive(triangle_leg_distance)
            if (on_left_triangle):
                swarmie.turn((-1 * math.pi) / 2)
            else:
                swarmie.turn(math.pi / 2)
            swarmie.drive(triangle_leg_distance)
            on_left_triangle = not on_left_triangle
    except ObstacleException:
        print("I found an obstacle.")
        #avoid()
        get_sonar_directional()
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
        for move in range(1) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                zigzag(3, 5)
            
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location()
                swarmie.set_home_odom_location(odom_location)
                turnaround()
                
    except TagException : 
        print("I found a tag!")
        # Let's drive there to be helpful.
        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.3, ignore=Obstacle.IS_VISION)
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()