#! /usr/bin/env python
"""
Listen to the /imu topic while the rover is not moving.

This will help experimentally determine the variance of each sensor.
"""
from __future__ import print_function
from datetime import datetime
import signal
import sys
import os
import math
import numpy as np

import rospy
import rosnode
import tf
from sensor_msgs.msg import Imu

def handle_exit(signal, frame) :
    global imu_sub, orientations, angular_vels, linear_accs
    imu_sub.unregister()

    filename = 'imu_variances_' + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + '.txt'
    fullpath = os.environ['HOME'] + '/' + filename

    roll_avg, pitch_avg, yaw_avg = np.average(orientations, axis=0)
    roll_var, pitch_var, yaw_var = np.var(orientations, axis=0)

    angx_avg, angy_avg, angz_avg = np.average(angular_vels, axis=0)
    angx_var, angy_var, angz_var = np.var(angular_vels, axis=0)

    accx_avg, accy_avg, accz_avg = np.average(linear_accs, axis=0)
    accx_var, accy_var, accz_var = np.var(linear_accs, axis=0)

    vals = [[roll_avg, pitch_avg, yaw_avg],
            [roll_var, pitch_var, yaw_var],
            [angx_avg, angy_avg, angz_avg],
            [angx_var, angy_var, angz_var],
            [accx_avg, accy_avg, accz_avg],
            [accx_var, accy_var, accz_var]]

    print('\nSaving values to', fullpath, '\n')
    np.savetxt(fullpath,
               vals,
               fmt='%g',
               header=string_of_vals())
    sys.exit(0)

def imu_callback(msg):
    global orientations, angular_vels, linear_accs
    quat = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]
    orientation = tf.transformations.euler_from_quaternion(quat)
    orientations = np.append(orientations, [orientation], axis=0)
    angular_vels = np.append(
        angular_vels,
        [[msg.angular_velocity.x,
          msg.angular_velocity.y,
          msg.angular_velocity.z]],
        axis=0
    )
    linear_accs = np.append(
        linear_accs,
        [[msg.linear_acceleration.x,
          msg.linear_acceleration.y,
          msg.linear_acceleration.z]],
        axis=0
    )

    print(string_of_vals())


def string_of_vals():
    global orientations, angular_vels, linear_accs

    roll_avg, pitch_avg, yaw_avg = np.average(orientations, axis=0)
    roll_var, pitch_var, yaw_var = np.var(orientations, axis=0)

    angx_avg, angy_avg, angz_avg = np.average(angular_vels, axis=0)
    angx_var, angy_var, angz_var = np.var(angular_vels, axis=0)

    accx_avg, accy_avg, accz_avg = np.average(linear_accs, axis=0)
    accx_var, accy_var, accz_var = np.var(linear_accs, axis=0)

    s = '''
    Orientation:
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}
    
    Angular Velocity:
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}
    
    Linear Acceleration:
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}
    {:<10}: {: 6.7f} | {:<10}: {: 6.7f} | {:<10}: {: 6.7f}'''

    return s.format(
        'Roll Avg', roll_avg, 'Pitch Avg', pitch_avg, 'Yaw Avg', yaw_avg,
        'Roll Var', roll_var, 'Pitch Var', pitch_var, 'Yaw Var', yaw_var,
        'Ang-x Avg', angx_avg, 'Ang-y Avg', angy_avg, 'Ang-z Avg', angz_avg,
        'Ang-x Var', angx_var, 'Ang-y Var', angy_var, 'Ang-z Var', angz_var,
        'Acc-x Avg', accx_avg, 'Acc-y Avg', accy_avg, 'Acc-z Avg', accz_avg,
        'Acc-x Var', accx_var, 'Acc-y Var', accy_var, 'Acc-z Var', accz_var,

    )

if __name__ == '__main__' :
    global imu_sub, orientations, angular_vels, linear_accs

    if len(sys.argv) < 2:
        rovers = set()
        nodes = rosnode.get_node_names()
        for node in nodes:
            if 'MOBILITY' in node:
                node = node.lstrip('/')
                rovername = node.split('_')[0]
                rovers.add(rovername)
        if len(rovers) == 0:
            print('\033[91m',"No Rovers Detected",'\033[0m')
            print('usage:', sys.argv[0], '<rovername>')
            exit(-1)
        elif len(rovers) == 1:
            rovername = rovers.pop()
            print('Detected rovers: ', rovername)
            print('\033[92m',"Auto selected:",rovername,'\033[0m')
        else:
            print('Detected rovers:')
            for rover in rovers:
                print(rover)
            rovername = ''
            while rovername not in rovers:
                rovername = raw_input('Which rover would you like to connect to? ')
    else:
        rovername = sys.argv[1]

    collecting_data = True
    orientations = np.empty((0, 3))
    angular_vels = np.empty((0, 3))
    linear_accs = np.empty((0, 3))

    signal.signal(signal.SIGINT, handle_exit)
    rospy.init_node('COVARIANCE')

    imu_sub = rospy.Subscriber(
        rovername + '/imu',
        Imu,
        imu_callback
    )

    print('Leave the rover still and wait a few minutes.')
    rospy.spin()