#! /usr/bin/env python

from __future__ import print_function

import sys
import math 
import rospy 

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import swarmie, Location

def main(**kwargs):

    # During a normal startup the rover will be facing the center and
    # close to the nest. But there's no guarantee where we will be if 
    # mobility crashes and is forced to restart. This checks to see 
    # if we've set home location prviously.
    
    while not swarmie.imu_is_finished_validating():
        pass  # wait till extended cal file has been loaded in IMU node

    if not swarmie.simulator_running():
        swarmie.start_gyro_bias_calibration()
        rospy.sleep(3)
        swarmie.store_imu_calibration()

    # Assume the starting position is facing the center. 
    # This should be valid by contest rules. 
    #
    # Drive closer until we can see the center. 
    try:
        swarmie.drive(1) 
    except: 
        # This could happen if we bump into another rover. 
        # Let's just call it good. 
        pass


    wait_duration = 3;
    start_time = rospy.get_rostime().secs
    current_location = swarmie.get_odom_location()
    print("init.py: Waiting ", wait_duration, "secs")
    while (not current_location) and (rospy.get_rostime().secs < (start_time + wait_duration)):
        current_location = swarmie.get_odom_location()
    if not current_location:
        print("init.py: could not get odom location")  # will crash after this
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

    if swarmie.imu_needs_calibration():
        swarmie.drive(
            -0.5,
            ignore=Obstacle.TAG_HOME | Obstacle.TAG_TARGET | Obstacle.IS_SONAR
        )
        start_heading = swarmie.get_odom_location().get_pose().theta

        swarmie.start_imu_calibration()
        drive_result = None
        while drive_result != MoveResult.SUCCESS:
            drive_result = swarmie.timed_drive(
                25,
                0,
                0.6,
                ignore=Obstacle.TAG_HOME|Obstacle.TAG_TARGET|Obstacle.IS_SONAR,
                throw=False
            )
        swarmie.store_imu_calibration()

        swarmie.set_heading(
            start_heading,
            ignore=Obstacle.TAG_HOME | Obstacle.TAG_TARGET | Obstacle.IS_SONAR
        )

    swarmie.turn(
        math.pi,
        ignore=Obstacle.TAG_HOME | Obstacle.TAG_TARGET | Obstacle.IS_SONAR
    )
    return 0

if __name__ == '__main__' : 
    swarmie.start(node_name='init')
    sys.exit(main())
