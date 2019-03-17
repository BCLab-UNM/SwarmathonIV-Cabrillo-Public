#! /usr/bin/env python
"""Searcher node."""

from __future__ import print_function

import sys
import rospy
import tf
import math
import random 

import dynamic_reconfigure.client
from geometry_msgs.msg import Point
from swarmie_msgs.msg import Obstacle

from mobility.planner import Planner
from mobility.swarmie import swarmie, TagException, HomeException, ObstacleException, PathException, AbortException, MoveResult


def turnaround(ignore=Obstacle.IS_SONAR | Obstacle.VISION_SAFE):
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=ignore)


def wander():
    try:
        rospy.loginfo("Wandering...")
        swarmie.drive(random.gauss(2.5, 1))
        prev_heading = swarmie.get_odom_location().get_pose().theta

        rospy.loginfo("Circling...")
        swarmie.circle()
        swarmie.set_heading(prev_heading + random.gauss(0, math.pi/6))

    except HomeException:
        print ("I saw home!")
        # TODO: We used to set the home odom location here, while we had
        #  the chance. If finding home during gohome becomes difficult,
        #  it may be useful to have home_transform publish a less
        #  accurate, but easier to update, home position estimate.
        turnaround()

    except ObstacleException:
        print ("I saw an obstacle!")
        turnaround(ignore=Obstacle.IS_SONAR)


def random_walk(num_moves):
    """Do random walk `num_moves` times."""
    global planner, found_tag

    try:
        for move in range(num_moves):
            if rospy.is_shutdown():
                search_exit(-1)

            wander()

    except TagException:
        print("I found a tag!")
        # Let's drive there to be helpful.
        rospy.sleep(0.3)
        if not planner.sees_home_tag():
            found_tag = True
            search_exit(0)


def search_exit(code):
    global planner, found_tag
    
    reset_speeds()
    
    if found_tag:
        print('Found a tag! Trying to get a little closer.')
        planner.face_nearest_block()
    
    if code == 0:
        swarmie.print_infoLog('Setting search exit poses.')
        set_search_exit_poses()
    sys.exit(code)


def reset_speeds():
    global initial_config, param_client
    param_client.update_configuration(initial_config)


def set_search_exit_poses():
    swarmie.set_search_exit_poses()


def drive_to(pose, use_waypoints):
    planner.drive_to(pose,
                     tolerance=0.5,
                     tolerance_step=0.5,
                     avoid_targets=False,
                     avoid_home=True,
                     use_waypoints=use_waypoints)

    cur_loc = swarmie.get_odom_location()
    if not cur_loc.at_goal(pose, 0.3):
        print('Getting a little closer to last exit position.')
        swarmie.drive_to(pose, throw=False)


def return_to_last_exit_position(last_pose):
    global planner, found_tag

    print('Driving to last search exit position.')
    swarmie.print_infoLog('Driving to last search exit position.')

    count = 0
    use_waypoints = True

    while count < 2:
        try:
            drive_to(last_pose, use_waypoints)

        except rospy.ServiceException:
            # try again without map waypoints
            use_waypoints = False

        except PathException:
            print('PathException on our way to last search exit location.')
            # good enough
            break

        count += 1

    try:
        swarmie.circle()
        swarmie.set_heading(
            last_pose.theta,
            ignore=Obstacle.VISION_HOME
        )

    except TagException:
        rospy.sleep(0.3)  # build buffer a little
        # too risky to stop for targets if home is in view too
        if not planner.sees_home_tag():
            # success!
            found_tag = True
            search_exit(0)

    except HomeException:
        # Just move onto random search, but this shouldn't really happen either.
        pass

    except ObstacleException:
        print('ObstacleException while finishing return to last search exit location.')
        pass  # good enough


def main(**kwargs):
    global planner, found_tag
    global initial_config, param_client

    found_tag = False
    SEARCH_SPEEDS = {
         'DRIVE_SPEED': 0.25,
         'TURN_SPEED': 0.7
    }

    planner = Planner()

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

    # Return to our last search exit pose if possible
    if swarmie.has_search_exit_poses():
        cur_pose = swarmie.get_odom_location().get_pose()

        last_pose = swarmie.get_search_exit_poses()
        dist = math.sqrt((last_pose.x - cur_pose.x) ** 2
                         + (last_pose.y - cur_pose.y) ** 2)

        if dist > 1:  # only bother if it was reasonably far away
            return_to_last_exit_position(last_pose)

    random_walk(num_moves=30)

    print ("I'm homesick!")
    search_exit(1)


if __name__ == '__main__' : 
    swarmie.start(node_name='search')
    sys.exit(main())
