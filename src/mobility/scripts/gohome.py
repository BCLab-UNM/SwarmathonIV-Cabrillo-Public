#! /usr/bin/env python

from __future__ import print_function

import argparse
import sys 
import math 
import rospy 
import angles

from geometry_msgs.msg import PoseStamped, Pose2D, Point
from nav_msgs.srv import GetPlanResponse

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, HomeException, Location

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

def clear(angle):
    global swarmie

    start_heading = swarmie.get_odom_location().get_pose().theta
    swarmie.set_heading(start_heading - angle, ignore=Obstacle.IS_SONAR)
    swarmie.set_heading(start_heading + angle, ignore=Obstacle.IS_SONAR)
    swarmie.set_heading(start_heading, ignore=Obstacle.IS_SONAR)

def go_around(angle, dist):
    global swarmie
    cur_heading = swarmie.get_odom_location().get_pose().theta
    swarmie.set_heading(
        cur_heading + angle,
        ignore=Obstacle.IS_SONAR,
        throw=False
    )
    swarmie.drive(dist, throw=False)

def rviz_nav_goal_cb(msg):
    global swarmie
    print('Request received')
    DISTANCE_OK = 0.5
    FAIL_COUNT_LIMIT = 3
    fail_count = 0

    goal = Pose2D()
    goal.x = msg.pose.position.x
    goal.y = msg.pose.position.y

    cur_loc = swarmie.get_odom_location()

    while (not cur_loc.at_goal(goal, DISTANCE_OK)
           and fail_count < FAIL_COUNT_LIMIT):
        print('Getting new nav plan.')
        plan = swarmie.get_plan(goal)
        print('Received nav plan.')

        # drive to 1st-3rd pts in plan, then get new plan
        # for index, pose in enumerate(plan.plan.poses[0:3]):
        index = 1
        pose = plan.plan.poses[0]
        point = Point(
            x=pose.pose.position.x,
            y=pose.pose.position.y
        )
        result = swarmie.drive_to(point, throw=False)

        if result == MoveResult.SUCCESS:
            fail_count = 0
            print('Successfully drove to point {} in nav plan.'.format(
                index
            ))
        else:
            fail_count += 1

            if result == MoveResult.OBSTACLE_HOME:
                print('Obstacle: Found Home.')
                return
            elif result == MoveResult.OBSTACLE_TAG:
                print('Obstacle: Found a Tag.')
                return
            elif result == MoveResult.OBSTACLE_SONAR:
                print('Obstacle: Sonar.')
                obstacle = swarmie.get_obstacle_condition()

                if obstacle & Obstacle.SONAR_LEFT == 0:
                    print('Left looks clear, turning left.')
                    go_around(math.pi/4, 0.75)
                    swarmie.drive_to(point, throw=False)
                elif obstacle & Obstacle.SONAR_RIGHT == 0:
                    print('Right looks clear, turning right.')
                    go_around(-math.pi/4, 0.75)
                    swarmie.drive_to(point, throw=False)
                else:
                    swarmie.drive(
                        -0.2,
                        ignore=Obstacle.IS_SONAR,
                        throw=False
                    )
                    clear(math.pi/8)

                # if fail_count == 1:
                #     clear(math.pi/8)
                # elif fail_count == 2:
                #     clear(math.pi/4)
                # elif fail_count == 3:
                #     clear(math.pi/2)

            elif result == MoveResult.PATH_FAIL:
                print('Path Failure. Backing up.')
                swarmie.drive(
                    -0.2,
                    ignore=Obstacle.IS_SONAR,
                    throw=False
                )
            # break  # get new plan now, current one isn't working

        cur_loc = swarmie.get_odom_location()

    if fail_count >= FAIL_COUNT_LIMIT:
        print('Failed to drive to goal {} times.'.format(FAIL_COUNT_LIMIT))
        return

    print('Successfully executed nav plan.')


def main():
    global swarmie 
    global rovername

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'rovername',
        help='required, name of the rover to connect to',
        nargs='?',
        default=None
    )
    parser.add_argument(
        '-r',
        '--use_rviz',
        action='store_true',
        help='Use RViz 2D Nav Goal to request goals from path planner.'
    )
    args = parser.parse_args()
    
    if args.rovername is None:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = args.rovername
    swarmie = Swarmie(rovername)

    if args.use_rviz:
        print ('Using RViz 2D Nav Goals')
        nav_goal_sub = rospy.Subscriber(
            rovername + '/goal',
            PoseStamped,
            rviz_nav_goal_cb,
            queue_size=1
        )
        rospy.spin()
    else:
        try :
            # Try driving home with odometry before GPS
            drive_straight_home_odom()
            while not rospy.is_shutdown() :
                drive_straight_home_gps()
        except HomeException as e:
            # Found home!
            exit(0)
    
if __name__ == '__main__' : 
    main()
