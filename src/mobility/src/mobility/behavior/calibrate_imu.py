#! /usr/bin/env python
"""Calibrate the IMU using the 2D method, if necessary."""
from __future__ import print_function

import sys
import rospy

from mobility.swarmie import swarmie


def main(**kwargs):
    # Wait until the extended calibration file has been loaded in IMU node.
    while not swarmie.imu_is_finished_validating():
        pass

    if not swarmie.simulator_running():
        # TODO: would it make sense to calibrate the simulated IMU as well?
        swarmie.start_gyro_bias_calibration()
        rospy.sleep(3)
        swarmie.store_imu_calibration()

    return 0


if __name__ == '__main__':
    swarmie.start(node_name='calibrate_imu')
    sys.exit(main())