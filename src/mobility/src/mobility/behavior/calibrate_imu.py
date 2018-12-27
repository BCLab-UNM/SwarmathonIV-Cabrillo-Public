#! /usr/bin/env python
"""Calibrate the IMU using the 2D method, if necessary."""
from __future__ import print_function

import sys
import rospy

from mobility.msg import MoveResult

from mobility.swarmie import swarmie, Obstacle


def main(**kwargs):
    # Wait until the extended calibration file has been loaded in IMU node.
    while not swarmie.imu_is_finished_validating():
        pass

    if not swarmie.simulator_running():
        # TODO: would it make sense to calibrate the simulated IMU as well?
        swarmie.start_gyro_bias_calibration()
        rospy.sleep(3)
        swarmie.store_imu_calibration()

    if swarmie.imu_needs_calibration():
        start_heading = swarmie.get_odom_location().get_pose().theta

        swarmie.start_imu_calibration()
        drive_result = None
        while drive_result != MoveResult.SUCCESS:
            drive_result = swarmie.timed_drive(
                25,
                0,
                0.6,
                ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR,
                throw=False
            )
        swarmie.store_imu_calibration()

        swarmie.set_heading(start_heading,
                            ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    return 0


if __name__ == '__main__':
    swarmie.start(node_name='calibrate_imu')
    sys.exit(main())