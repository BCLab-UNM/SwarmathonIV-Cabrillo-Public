#! /usr/bin/env python
"""gohome.py
Tries to get the rover back to the home nest, while avoiding sonar and cube
obstacles, and hopefully not dropping the cube in its claw.
"""
from __future__ import print_function

import sys
import math 
import rospy 
import angles

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, PathException
from planner import Planner


def drive_straight_home_gps() :
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

    swarmie.turn(angle, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    swarmie.drive(dist, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)


def drive_straight_home_odom() :
    global swarmie

    # We remember home in the Odom frame when we see it. Unlike GPS
    # there's no need to translate the location into r and theta. The
    # swarmie's drive_to function takes a point in odometry space.

    home = swarmie.get_home_odom_location()
    swarmie.drive_to(home, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)


def main():
    global swarmie 
    global rovername

    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.print_infoLog(rovername + ": gohome started.")

    planner = Planner(swarmie)
    swarmie.fingers_close()  # make sure we keep a firm grip
    swarmie.wrist_middle()  # get block mostly out of camera view
    home = swarmie.get_home_odom_location()
    drive_result = None
    counter = 0

    while (counter < 2 and
           drive_result != MoveResult.SUCCESS and
           drive_result != MoveResult.OBSTACLE_HOME):
        try:
            drive_result = planner.drive_to(
                home,
                tolerance=0.5+counter,
                tolerance_step=0.5+counter
            )
        except PathException as e:
            if counter < 2:
                pass
            else:
                exit(1)

    # todo: is it necessary to check that we can still see a home tag? or does dropoff handle it ok?
    rospy.sleep(0.25)  # improve target detection chances?
    if planner.sees_home_tag():
        # victory!
        planner.face_home_tag()
        exit(0)

    print('Starting spiral search')
    planner.spiral_home_search(0.5, 0.75, tolerance=0.0, tolerance_step=0.5)
    rospy.sleep(0.25)  # improve target detection chances?
    if planner.sees_home_tag():
       planner.face_home_tag()


if __name__ == '__main__' : 
    main()
