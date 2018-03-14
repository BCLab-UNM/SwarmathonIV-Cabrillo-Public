#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy
import tf
import math
import random 

import dynamic_reconfigure.client
from geometry_msgs.msg import Point
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
    global swarmie, found_tag

    reset_speeds()

    if found_tag:
        print('Found a tag! Trying to get a little closer.')
        try:
            block = swarmie.get_nearest_block_location(
                use_targets_buffer=True
            )
        except tf.Exception:
            pass  # good enough
        if block is not None:
            swarmie.drive_to(
                block,
                claw_offset=0.5,
                ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR
            )

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
    global found_tag
    global initial_drive_speed, initial_turn_speed, param_client

    found_tag = False
    SEARCH_DRIVE_SPEED = 0.2
    SEARCH_TURN_SPEED = 0.6
    
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
    drive_speed = rospy.get_param(
        '/' + rovername + '/search/drive_speed',
        default=SEARCH_DRIVE_SPEED
    )
    turn_speed = rospy.get_param(
        '/' + rovername + '/search/turn_speed',
        default=SEARCH_TURN_SPEED
    )
    param_client = dynamic_reconfigure.client.Client(rovername + '_MOBILITY')
    initial_config = param_client.get_configuration()
    initial_drive_speed = initial_config['DRIVE_SPEED']
    initial_turn_speed = initial_config['TURN_SPEED']
    param_client.update_configuration(
        {'DRIVE_SPEED': drive_speed,
         'TURN_SPEED': turn_speed}
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

    if not planner.sees_home_tag():
        try:
            swarmie.drive(0.5, ignore=Obstacle.IS_SONAR)
        except HomeException:
            swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        except TagException:
            rospy.sleep(0.3)  # build the buffer a little
            try:
                if swarmie.get_nearest_block_location() is not None:
                    found_tag = True
                    exit(0)  # found a tag?
            except tf.Exception:
                pass
    else:
        swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    # Return to our last search exit pose if possible
    dist = 0
    cur_pose = swarmie.get_odom_location().get_pose()

    if swarmie.has_search_exit_poses():
        last_pose, _gps = swarmie.get_search_exit_poses()
        dist = math.sqrt((last_pose.x - cur_pose.x) ** 2
                         + (last_pose.y - cur_pose.y) ** 2)

    if dist > 1.5:  # only bother if it was reasonably far away
        print('Driving to last search exit position.')
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
            print('PathException on our way to last search exit location.')
            # good enough
            pass

        try:
            swarmie.set_heading(
                last_pose.theta,
                ignore=Obstacle.TAG_HOME | Obstacle.IS_SONAR
            )
        except TagException:
            rospy.sleep(0.3)  # build buffer a little
            # too risky to stop for targets if home is in view too
            if not planner.sees_home_tag():
                # success!
                found_tag = True
                exit(0)
            pass

    else:
        # drive somewhere before starting spiral
        print('Driving somewhere before starting search pattern.')
        angle = random.gauss(0, math.pi/8)
        point = Point()
        point.x = cur_pose.x + 2 * math.cos(cur_pose.theta)
        point.y = cur_pose.y + 2 * math.sin(cur_pose.theta)
        try:
            planner.drive_to(point, avoid_targets=False, avoid_home=True)
        except rospy.ServiceException:
            pass  # just start the search

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
        print('Nav plan ServiceException, trying search without waypoints.')
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
        rospy.sleep(0.3)
        if not planner.sees_home_tag():
            found_tag = True
            exit(0)

    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

