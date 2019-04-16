#! /usr/bin/env python
"""gohome.py
Tries to get the rover back to the home nest, while avoiding sonar and cube
obstacles, and hopefully not dropping the cube in its claw.
"""
from __future__ import print_function

import sys
import math 
import rospy
import tf
import angles
import argparse

from geometry_msgs.msg import Point

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import swarmie, Location, PathException, HomeException, TagException, ObstacleException
from mobility.planner import Planner


GOHOME_FOUND_TAG = 1
GOHOME_FAIL = -1


def face_home_tag():
    # todo: is it necessary to check that we can still see a home tag? or does dropoff handle it ok?
    rospy.sleep(0.25)  # improve target detection chances?
    if planner.sees_home_tag():
        try:
            planner.face_home_tag()
        except tf.Exception:
            pass  # good enough


def drive_home(has_block, home_loc):
    global planner, use_waypoints, GOHOME_FAIL

    drive_result = None
    counter = 0

    while (counter < 2 and
           drive_result != MoveResult.SUCCESS and
           drive_result != MoveResult.OBSTACLE_HOME and
           drive_result != MoveResult.OBSTACLE_TAG):
        try:
            drive_result = planner.drive_to(
                home_loc,
                tolerance=0.5+counter,
                tolerance_step=0.5+counter,
                avoid_targets=has_block,
                use_waypoints=use_waypoints,
                **swarmie.speed_fast
            )
        except rospy.ServiceException:
            use_waypoints = False  # fallback if map service fails
        except PathException as e:
            if counter < 2:
                pass
            else:
                sys.exit(GOHOME_FAIL)

        counter += 1

    if drive_result == MoveResult.OBSTACLE_HOME:
        face_home_tag()

    elif drive_result == MoveResult.OBSTACLE_TAG:
        # This can happen if we're going home without a block.
        planner.face_nearest_block()

    return drive_result


def spiral_search(has_block):
    global planner

    # no map waypoints
    try:
        drive_result = planner.spiral_search(
            0.5,
            0.75,
            tolerance=0.0,
            tolerance_step=0.5,
            avoid_targets=has_block,
            avoid_home=False,
            use_waypoints=False,
            **swarmie.speed_fast
        )
    except PathException:
        raise

    if drive_result == MoveResult.OBSTACLE_HOME:
        face_home_tag()

    elif drive_result == MoveResult.OBSTACLE_TAG:
        # This can happen if we're going home without a block.
        planner.face_nearest_block()

    return drive_result


def get_best_home_location(dist_threshold=0.7):
    """Return the home location most likely to be up to date. This will
    be either:

        - The accurate home odom location, if it's close to the approximate
          home location.
        - The approximate home odom location, if the accurate home odom location
          appears not to have been updated recently enough.

    Args:
        dist_threshold: Distance (m). If the accurate home odom location is
            further than this distance away from the approximate home odom
            location, then it won't be used.
    """
    home_accurate = swarmie.get_home_odom_location()
    home_approx = swarmie.get_home_odom_location(approx=True)

    dist = math.hypot(home_accurate.x - home_approx.x,
                      home_accurate.y - home_approx.y)

    if dist > dist_threshold:
        return home_approx

    return home_accurate


def main(**kwargs):
    global planner, use_waypoints

    has_block = False
    if 'has_block' in kwargs : 
        has_block = kwargs['has_block']
    
    # Whether to use waypoints from searching the map. Can be set to False if
    # the map service fails.
    use_waypoints = True

    if not has_block:
        swarmie.print_infoLog("I don't have a block. Not avoiding targets.")

    planner = Planner()

    swarmie.fingers_close()  # make sure we keep a firm grip
    swarmie.wrist_middle()  # get block mostly out of camera view
    home = get_best_home_location()

    drive_result = drive_home(has_block, home)
    if drive_result == MoveResult.OBSTACLE_HOME:
        sys.exit(0)
    elif drive_result == MoveResult.OBSTACLE_TAG:
        sys.exit(GOHOME_FOUND_TAG)

    # Look to the right and left before starting spiral search, which goes
    # left:
    ignore = Obstacle.PATH_IS_CLEAR
    if has_block:
        ignore |= Obstacle.TAG_TARGET
    try:
        cur_loc = swarmie.get_odom_location()
        if not cur_loc.at_goal(home, 0.3):
            print('Getting a little closer to home position.')
            swarmie.drive_to(home, ignore=ignore, **swarmie.speed_fast)

        planner.clear(2 * math.pi / 5, ignore=ignore, throw=True)
    except HomeException:
        face_home_tag()
        sys.exit(0)
    except TagException:
        sys.exit(GOHOME_FOUND_TAG)
    except ObstacleException:
        pass # do spiral search

    swarmie.print_infoLog(swarmie.rover_name + " Starting spiral search")

    print('Starting spiral search with location')
    try:
        drive_result = spiral_search(has_block)
        if drive_result == MoveResult.OBSTACLE_HOME:
            sys.exit(0)
        elif drive_result == MoveResult.OBSTACLE_TAG:
            sys.exit(GOHOME_FOUND_TAG)
    except PathException:
        sys.exit(GOHOME_FAIL)

    # didn't find anything
    return GOHOME_FAIL

if __name__ == '__main__' :
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
        )
    parser.add_argument(
        '--has-block',
        action='store_true',
        help=('whether the rover currently has a block, and should ' +
              'accordingly either avoid cubes or stop for them')
    )
    args = parser.parse_args()
    swarmie.start(node_name='gohome')
    sys.exit(main(has_block=args.has_block))
