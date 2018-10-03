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
        swarmie.turn(random.gauss(0, math.pi/6))
        swarmie.drive(random.gauss(2.5, 1))

        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround()


def escape_home(detections):
    global swarmie, planner

    print('\nGetting out of the home ring!!')
    angle, dist = planner.get_angle_and_dist_to_escape_home(detections)
    swarmie.turn(
        angle,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )
    swarmie.drive(
        dist,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )


def handle_exit():
    global planner, swarmie, found_tag

    reset_speeds()

    if found_tag:
        print('Found a tag! Trying to get a little closer.')
        planner.face_nearest_block()

    swarmie.print_infoLog('Setting search exit poses.')
    set_search_exit_poses()


def reset_speeds():
    global initial_config, param_client
    param_client.update_configuration(initial_config)


def set_search_exit_poses():
    global swarmie
    swarmie.set_search_exit_poses()


def main(s, **kwargs):
    global swarmie, planner, found_tag
    global initial_config, param_client

    swarmie = s
    found_tag = False
    SEARCH_SPEEDS = {
         'DRIVE_SPEED': 0.25,
         'TURN_SPEED': 0.7
    }

    planner = Planner(swarmie)

    swarmie.fingers_open()
    swarmie.wrist_middle()

    # Change drive and turn speeds for this behavior, and register shutdown
    # hook to reset them at exit.
    if not rospy.has_param('search/speeds'):
        speeds = SEARCH_SPEEDS
        rospy.set_param('search/speeds', speeds)
    else:
        speeds = rospy.get_param('search/speeds',
                                 default=SEARCH_SPEEDS)

    param_client = dynamic_reconfigure.client.Client('mobility')
    config = param_client.get_configuration()
    initial_config = {
        'DRIVE_SPEED': config['DRIVE_SPEED'],
        'TURN_SPEED': config['TURN_SPEED']
    }
    param_client.update_configuration(speeds)
    rospy.on_shutdown(handle_exit)

    if not planner.sees_home_tag():
        try:
            swarmie.drive(0.5, ignore=Obstacle.IS_SONAR)
        except HomeException:
            # get out of from inside home if it ever happens
            detections = swarmie.get_latest_targets().detections
            if planner.is_inside_home_ring(detections):
                escape_home(detections)
            else:
                swarmie.turn(math.pi,
                             ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        except TagException:
            rospy.sleep(0.3)  # build the buffer a little
            try:
                if swarmie.get_nearest_block_location() is not None:
                    found_tag = True
                    # print('Found a tag! Turning to face.')
                    # planner.face_nearest_block()
                    sys.exit(0)  # found a tag?
            except tf.Exception:
                pass
    else:
        # get out of from inside home if it ever happens
        detections = swarmie.get_latest_targets().detections
        if planner.is_inside_home_ring(detections):
            escape_home(detections)
        else:
            swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    # Return to our last search exit pose if possible
    dist = 0
    cur_pose = swarmie.get_odom_location().get_pose()

    if swarmie.has_search_exit_poses():
        last_pose = swarmie.get_search_exit_poses()
        dist = math.sqrt((last_pose.x - cur_pose.x) ** 2
                         + (last_pose.y - cur_pose.y) ** 2)

    if dist > 1:  # only bother if it was reasonably far away
        print('Driving to last search exit position.')
        swarmie.print_infoLog('Driving to last search exit position.')
        try:
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True)

            cur_loc = swarmie.get_odom_location()
            if not cur_loc.at_goal(last_pose, 0.3):
                print('Getting a little closer to last exit position.')
                swarmie.drive_to(last_pose, throw=False)

        except rospy.ServiceException:
            # try again without map waypoints
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True,
                             use_waypoints=False)

            cur_loc = swarmie.get_odom_location()
            if not cur_loc.at_goal(last_pose, 0.3):
                print('Getting a little closer to last exit position.')
                swarmie.drive_to(last_pose, throw=False)

        except PathException:
            print('PathException on our way to last search exit location.')
            # good enough
            pass

        try:
            # planner.clear(math.pi / 4, ignore=Obstacle.TAG_HOME, throw=True)
            # swarmie.drive(0.2, throw=False)
            # planner.sweep(throw=True)
            swarmie.circle()
            swarmie.set_heading(
                last_pose.theta,
                ignore=Obstacle.TAG_HOME
            )
        except TagException:
            rospy.sleep(0.3)  # build buffer a little
            # too risky to stop for targets if home is in view too
            if not planner.sees_home_tag():
                # success!
                found_tag = True
                # print('Found a tag! Turning to face.')
                # planner.face_nearest_block()
                sys.exit(0)
        except HomeException:
            # Just move onto random search, but this shouldn't really happen
            # either.
            pass
        except ObstacleException:
            print('ObstacleException while finishing return to last search exit location.')
            pass # good enough

    try:
        for move in range(30) :
            if rospy.is_shutdown() :
                sys.exit(-1)
            try:
                wander()

            except HomeException :
                print ("I saw home!")
                planner.set_home_locations()

                # get out of from inside home if it ever happens
                detections = swarmie.get_latest_targets().detections
                if planner.is_inside_home_ring(detections):
                    escape_home(detections)

                turnaround()

    except TagException :
        print("I found a tag!")
        # Let's drive there to be helpful.
        rospy.sleep(0.3)
        if not planner.sees_home_tag():
            found_tag = True
            # print('Found a tag! Turning to face.')
            # planner.face_nearest_block()
            # swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)
            sys.exit(0)

    print ("I'm homesick!")
    return 1 

if __name__ == '__main__' : 
    sys.exit(main(Swarmie()))
