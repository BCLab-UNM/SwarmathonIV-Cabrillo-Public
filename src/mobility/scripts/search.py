#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

import dynamic_reconfigure.client
from swarmie_msgs.msg import Obstacle

from planner import Planner
from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException, MoveResult

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


def handle_exit():
    reset_speeds()
    set_search_exit_poses()


def reset_speeds():
    global initial_drive_speed, initial_turn_speed, param_client
    param_client.update_configuration(
        {'DRIVE_SPEED': initial_drive_speed,
         'TURN_SPEED': initial_turn_speed}
    )


def set_search_exit_poses():
    global swarmie
    swarmie.set_search_exit_poses()


def main():
    global swarmie 
    global rovername
    global initial_drive_speed, initial_turn_speed, param_client

    search_drive_speed = 0.25
    search_turn_speed = 0.7
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    planner = Planner(swarmie)

    swarmie.fingers_open()
    swarmie.wrist_middle()

    # Change drive and turn speeds for this behavior, and register shutdown
    # hook to reset them at exit.
    param_client = dynamic_reconfigure.client.Client(rovername + '_MOBILITY')
    initial_config = param_client.get_configuration()
    initial_drive_speed = initial_config['DRIVE_SPEED']
    initial_turn_speed = initial_config['TURN_SPEED']
    param_client.update_configuration(
        {'DRIVE_SPEED': search_drive_speed,
         'TURN_SPEED': search_turn_speed}
    )
    rospy.on_shutdown(handle_exit)

    # try:
    #     for move in range(30) :
    #         if rospy.is_shutdown() :
    #             exit(-1)
    #         try:
    #             wander()
    #
    #         except HomeException :
    #             print ("I saw home!")
    #             odom_location = swarmie.get_odom_location()
    #             swarmie.set_home_odom_location(odom_location)
    #             turnaround()
    #
    # except TagException :
    #     print("I found a tag!")
    #     # Let's drive there to be helpful.
    #     swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)
    #     exit(0)

    try:
        swarmie.drive(0.5, ignore=Obstacle.IS_SONAR)
    except HomeException:
        swarmie.turn(math.pi, ignore=Obstacle.IS_VISION)


    # Return to our last search exit pose if possible
    if swarmie.has_search_exit_poses():
        last_pose, _gps = swarmie.get_search_exit_poses()
        try:
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True)
        except rospy.ServiceException:
            # try again without map waypoints
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True,
                             use_waypoints=False)
        except PathException:
            # good enough
            pass

        swarmie.set_heading(last_pose.theta,
                            ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    # do search
    try:
        drive_result = planner.spiral_search(
            0.5,
            0.75,
            num_legs=15,
            tolerance=0.0,
            tolerance_step=0.5,
            avoid_targets=False,
            avoid_home=True
        )
    except rospy.ServiceException:
        # try again with no map waypoints
        drive_result = planner.spiral_search(
            0.5,
            0.75,
            num_legs=15,
            tolerance=0.0,
            tolerance_step=0.5,
            avoid_targets=False,
            avoid_home=True,
            use_waypoints=False
        )

    if drive_result == MoveResult.OBSTACLE_TAG:
        swarmie.drive_to(swarmie.get_nearest_block_location(),
                          claw_offset=0.6, ignore=Obstacle.IS_VISION)
        exit(0)

    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

