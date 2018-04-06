#! /usr/bin/env python
"""gohome.py
Tries to get the rover back to the home nest, while avoiding sonar and cube
obstacles, and hopefully not dropping the cube in its claw.
todo: test backup gps gohome functionality
"""
from __future__ import print_function

import sys
import math 
import rospy
import tf
import angles
import dynamic_reconfigure.client

from geometry_msgs.msg import Point

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, Location, PathException, HomeException, TagException, ObstacleException
from planner import Planner


GOHOME_FOUND_TAG = 1
GOHOME_FAIL = -1


def get_gps_angle_and_dist():
    global swarmie

    # Use GPS to figure out about where we are.
    # FIXME: We need to hanlde poor GPS fix.
    loc = swarmie.wait_for_fix(distance=4, time=60).get_pose()
    home = swarmie.get_home_gps_location()


    dist = math.hypot(loc.y - home.y,
                      loc.x - home.x)

    angle = angles.shortest_angular_distance(loc.theta,
                                             math.atan2(home.y - loc.y,
                                                        home.y - loc.x))

    # swarmie.turn(angle, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    # swarmie.drive(dist, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    return angle, dist


def drive_straight_home_odom() :
    global swarmie

    # We remember home in the Odom frame when we see it. Unlike GPS
    # there's no need to translate the location into r and theta. The
    # swarmie's drive_to function takes a point in odometry space.

    home = swarmie.get_home_odom_location()
    swarmie.drive_to(home, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)

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
                use_waypoints=use_waypoints
            )
        except rospy.ServiceException:
            use_waypoints = False  # fallback if map service fails
        except PathException as e:
            if counter < 2:
                pass
            else:
                exit(GOHOME_FAIL)


def spiral_search(has_block):
    global planner, swarmie

    # no map waypoints
    try:
        drive_result = planner.spiral_search(
            0.5,
            0.75,
            tolerance=0.0,
            tolerance_step=0.5,
            avoid_targets=has_block,
            avoid_home=False,
            use_waypoints=False
        )
    except PathException:
        raise

    if drive_result == MoveResult.OBSTACLE_HOME:
        rospy.sleep(0.25)  # improve target detection chances?
        if planner.sees_home_tag():
            try:
                planner.face_home_tag()
            except tf.Exception:
                pass  # good enough
            if has_block is False:
                planner.set_home_locations()

    elif drive_result == MoveResult.OBSTACLE_TAG:
        # This can happen if we're going home without a block.
        planner.face_nearest_block()

    return drive_result


def reset_speeds():
    global initial_config, param_client
    param_client.update_configuration(initial_config)


def main():
    global planner, swarmie, rovername, use_waypoints
    global initial_config, param_client

    has_block = False
    # Whether to use waypoints from searching the map. Can be set to False if
    # the map service fails.
    use_waypoints = True

    GOHOME_SPEEDS = {
         'DRIVE_SPEED': 0.25,
         'TURN_SPEED': 0.7
    }

    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    if len(sys.argv) >= 3 and sys.argv[2] == '--has-block':
        has_block = True

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    if not has_block:
        swarmie.print_infoLog(rovername +
                              ": I don't have a block. Not avoiding targets.")

    planner = Planner(swarmie)

    # Change drive and turn speeds for this behavior, and register shutdown
    # hook to reset them at exit.
    if not rospy.has_param('/' + rovername + '/gohome/speeds'):
        speeds = GOHOME_SPEEDS
        rospy.set_param('/' + rovername + '/gohome/speeds', speeds)
    else:
        speeds = rospy.get_param('/' + rovername + '/gohome/speeds',
                                 default=GOHOME_SPEEDS)

    param_client = dynamic_reconfigure.client.Client(rovername + '_MOBILITY')
    config = param_client.get_configuration()
    initial_config = {
        'DRIVE_SPEED': config['DRIVE_SPEED'],
        'TURN_SPEED': config['TURN_SPEED']
    }
    param_client.update_configuration(speeds)
    rospy.on_shutdown(reset_speeds)

    swarmie.fingers_close()  # make sure we keep a firm grip
    swarmie.wrist_middle()  # get block mostly out of camera view
    home = swarmie.get_home_odom_location()

    drive_home(has_block, home)

    # todo: is it necessary to check that we can still see a home tag? or does dropoff handle it ok?
    rospy.sleep(0.25)  # improve target detection chances?
    if planner.sees_home_tag():
        # victory!
        planner.face_home_tag()
        exit(0)

    # Look to the right and left before starting spiral search, which goes
    # left:
    ignore = Obstacle.PATH_IS_CLEAR
    if has_block:
        ignore |= Obstacle.TAG_TARGET
    try:
        cur_loc = swarmie.get_odom_location()
        if not cur_loc.at_goal(home, 0.3):
            print('Getting a little closer to home position.')
            swarmie.drive_to(home, ignore=ignore)

        planner.clear(2 * math.pi / 5, ignore=ignore, throw=True)
    except HomeException:
        planner.face_home_tag()
        exit(0)
    except TagException:
        exit(GOHOME_FOUND_TAG)
    except ObstacleException:
        pass # do spiral search

    print('Starting spiral search')
    try:
        drive_result = spiral_search(has_block)
        if drive_result == MoveResult.OBSTACLE_HOME:
            exit(0)
        elif drive_result == MoveResult.OBSTACLE_TAG:
            exit(GOHOME_FOUND_TAG)
    except PathException:
        pass  # try gps backup

    # gps backup attempt
    current_loc = swarmie.get_odom_location().get_pose()
    angle, dist = get_gps_angle_and_dist()

    goal = Point()
    goal.x = current_loc.x + dist * math.cos(current_loc.theta + angle)
    goal.y = current_loc.y + dist * math.sin(current_loc.theta + angle)

    drive_home(has_block, goal)

    print('Starting spiral search with gps location')
    try:
        drive_result = spiral_search(has_block)
        if drive_result == MoveResult.OBSTACLE_HOME:
            exit(0)
        elif drive_result == MoveResult.OBSTACLE_TAG:
            exit(GOHOME_FOUND_TAG)
    except PathException:
        exit(GOHOME_FAIL)

    # didn't find anything
    exit(GOHOME_FAIL)

if __name__ == '__main__' : 
    main()
