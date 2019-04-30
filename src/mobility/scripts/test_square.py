#! /usr/bin/env python

from __future__ import print_function

import argparse
import sys 
import math 
import rospy 

from swarmie_msgs.msg import Obstacle
from geometry_msgs.msg import Pose2D, Point

from mobility.swarmie import swarmie

def dumb_square(distance):
    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)   
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION)   

def smart_square(distance, ignore_sonar=False):
    ignore = Obstacle.IS_VISION
    if ignore_sonar:
        ignore |= Obstacle.IS_SONAR

    # Compute a square based on the current heading. 
    start_pose = swarmie.get_odom_location().get_pose()
    start = Point()
    start.x = start_pose.x 
    start.y = start_pose.y
    print('Start point: ({:.2f}, {:.2f})'.format(start.x, start.y))
    
    sq1 = Point()
    sq1.x = start.x + distance * math.cos(start_pose.theta)
    sq1.y = start.y + distance * math.sin(start_pose.theta)
    
    sq2 = Point()
    sq2.x = sq1.x + distance * math.cos(start_pose.theta + math.pi/2)
    sq2.y = sq1.y + distance * math.sin(start_pose.theta + math.pi/2)
    
    sq3 = Point()
    sq3.x = sq2.x + distance * math.cos(start_pose.theta + math.pi)
    sq3.y = sq2.y + distance * math.sin(start_pose.theta + math.pi)

    swarmie.drive_to(sq1, ignore=ignore)
    swarmie.drive_to(sq2, ignore=ignore)
    swarmie.drive_to(sq3, ignore=ignore)
    swarmie.drive_to(start, ignore=ignore)

    swarmie.set_heading(start_pose.theta)

    end_pose = swarmie.get_odom_location().get_pose()
    print('Start point: ({:.2f}, {:.2f})'.format(end_pose.x, end_pose.y))

def abs_square(distance):
    
    start_pose = swarmie.get_odom_location().get_pose()
    
    swarmie.drive(distance, ignore=Obstacle.IS_VISION)
    swarmie.set_heading(start_pose.theta + math.pi/2, ignore=-1)

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)
    swarmie.set_heading(start_pose.theta + math.pi, ignore=-1)

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)
    swarmie.set_heading(start_pose.theta + (3 * math.pi)/2, ignore=-1)

    swarmie.drive(distance, ignore=Obstacle.IS_VISION)
    swarmie.set_heading(start_pose.theta, ignore=-1)

def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        'distance',
        type=float,
        help='The length (m) of each side of the square.'
    )
    parser.add_argument(
        '--ignore-sonar',
        action='store_true',
        help='Whether the rover should ignore sonar obstacles on its journey.'
    )
    args = parser.parse_args()

    smart_square(args.distance, args.ignore_sonar)

if __name__ == '__main__' :
    swarmie.start(node_name='square')
    main()
