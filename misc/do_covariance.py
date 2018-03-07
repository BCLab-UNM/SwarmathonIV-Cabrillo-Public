"""
Listen to the accel, mag and gyro sensors while the rover is not moving. 

This will help experimentally determine the variance of each sensor. 
"""
from __future__ import print_function
import signal
import sys
import os
import math
import numpy as np

import rospy
from std_msgs.msg import Int32MultiArray

def handle_exit(signal, frame) :
    print_mats()
    sys.exit(0)


def callback_lsm303(sample):
    global collecting_data, lsm303_data
    lsm303_data = np.append(lsm303_data, sample.data)

def callback_l3g(sample):
    global collecting_data, l3g_data
    l3g_data = np.append(l3g_data, sample.data)

def print_mats() :
    global lsm303_data, l3g_data

    lsm303 = lsm303_data.reshape((-1, 6))
    l3g = l3g_data.reshape((-1, 3))
    
    magx_var = np.var(lsm303[:,0])
    magy_var = np.var(lsm303[:,1])
    magz_var = np.var(lsm303[:,2])
    accx_var = np.var(lsm303[:,3])
    accy_var = np.var(lsm303[:,4])
    accz_var = np.var(lsm303[:,5])
    
    gyrx_var = np.var(l3g[:,0])
    gyry_var = np.var(l3g[:,1])
    gyrz_var = np.var(l3g[:,2])
    
    lsm303_cov = np.diag((magx_var, magy_var, magz_var, accx_var, accy_var, accz_var))
    l3g_cov = np.diag((gyrx_var, gyry_var, gyrz_var))

    print ('\n\n==== lsm303 covariance matrix ===\n')    
    print(lsm303_cov)
    
    print ('\n\n==== l3g covariance matrix ===\n')    
    print(l3g_cov)

    
if __name__ == '__main__' :
    global collecting_data, lsm303_data, l3g_data

    collecting_data = True
    lsm303_data = np.array([])
    l3g_data = np.array([])
    
    signal.signal(signal.SIGINT, handle_exit)
    rospy.init_node('COVARIANCE')
    lsm_sub = rospy.Subscriber("/lsm303", Int32MultiArray, callback_lsm303)
    l3g_sub = rospy.Subscriber("/l3g", Int32MultiArray, callback_l3g)

    print('Leave the rover still and wait a few minutes.')
    raw_input('When finished press enter.')
    lsm_sub.unregister()
    l3g_sub.unregister()
    print_mats()
