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
import angles

from geometry_msgs.msg import Point

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, Location, PathException
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
    global planner, GOHOME_FAIL

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
                avoid_targets=has_block
            )
        except PathException as e:
            if counter < 2:
                pass
            else:
                exit(GOHOME_FAIL)


def set_home_locations():
    global swarmie

    swarmie.set_home_gps_location(swarmie.get_gps_location())

    current_location = swarmie.get_odom_location()
    current_pose = current_location.get_pose()
    home_odom = Location(current_location.Odometry)

    # project home_odom location 50cm in front of rover's current location
    home_odom.Odometry.pose.pose.position.x = (
        current_pose.x + 0.5 * math.cos(current_pose.theta)
    )
    home_odom.Odometry.pose.pose.position.y = (
        current_pose.y + 0.5 * math.sin(current_pose.theta)
    )
    swarmie.set_home_odom_location(home_odom)


def main():
    global planner
    global swarmie 
    global rovername
    has_block = False

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

    print('Starting spiral search')
    drive_result = planner.spiral_home_search(
        0.5,
        0.75,
        tolerance=0.0,
        tolerance_step=0.5,
        avoid_targets=has_block
    )
    if drive_result == MoveResult.OBSTACLE_HOME:
        rospy.sleep(0.25)  # improve target detection chances?
        if planner.sees_home_tag():
            planner.face_home_tag()
            if has_block is False:
                set_home_locations()
            exit(0)
    elif drive_result == MoveResult.OBSTACLE_TAG:
        exit(GOHOME_FOUND_TAG)

    # gps backup attempt
    current_loc = swarmie.get_odom_location().get_pose()
    angle, dist = get_gps_angle_and_dist()

    goal = Point()
    goal.x = current_loc.x + dist * math.cos(current_loc.theta + angle)
    goal.y = current_loc.y + dist * math.sin(current_loc.theta + angle)

    drive_home(has_block, goal)

    print('Starting spiral search')
    drive_result = planner.spiral_home_search(
        0.5,
        0.75,
        tolerance=0.0,
        tolerance_step=0.5,
        avoid_targets=has_block
    )
    if drive_result == MoveResult.OBSTACLE_HOME:
        rospy.sleep(0.25)  # improve target detection chances?
        if planner.sees_home_tag():
            planner.face_home_tag()
            if has_block is False:
                set_home_locations()
            exit(0)
    elif drive_result == MoveResult.OBSTACLE_TAG:
        exit(GOHOME_FOUND_TAG)

    # didn't find anything
    exit(GOHOME_FAIL)

if __name__ == '__main__' : 
    main()
