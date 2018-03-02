"""
Write calibration files and test calibration quality.
Test calculate's rover's current pitch and roll, and the sum of squared
errors of calibrated magnetometer and accelerometer data from the surface
of the unit sphere.

Pitch and roll should be below a set threshold, given that the rover is
placed on level ground after calibration procedure is complete just before
terminating this program.

v[E]'s should also be below a set threshold.

todo: pick SSE and roll/pitch thresholds
todo: delete files if not satisfactory?
"""
from __future__ import print_function
import signal
import sys
import os
import math
import numpy as np
import atexit
import traceback

import rospy
from std_msgs.msg import Int32MultiArray

CAL_MINMAX_FILE = os.environ['HOME'] + '/KSC.cal'
CAL_DATA_FILE = os.environ['HOME'] + '/KSC_extended_calibration.csv'

def ellipsoid_fit(x, y, z):
    """
    Fit the data points contained in numpy arrays x, y and z to a unit sphere
    centered at the origin.
    Returns a list containing the offset matrix to center the data, and
    a list containing the transformation matrix, to map each data point to
    its position on the sphere.
    Modified from:
    http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
    """
    D = np.array([x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z, 2*x, 2*y, 2*z])
    DT = D.conj().T
    v = np.linalg.solve(D.dot(DT), D.dot(np.ones(np.size(x))))
    A = np.array([[v[0], v[3], v[4], v[6]],
                  [v[3], v[1], v[5], v[7]],
                  [v[4], v[5], v[2], v[8]],
                  [v[6], v[7], v[8], -1]])
    center = np.linalg.solve(-A[:3,:3], [[v[6]], [v[7]], [v[8]]])
    T = np.eye(4)
    T[3,:3] = center.T
    R = T.dot(A).dot(T.conj().T)
    evals, evecs = np.linalg.eig(R[:3,:3] / -R[3,3])
    radii = np.sqrt(1. / evals)
    offset = center

    a, b, c = radii
    D = np.array([[1/a, 0., 0.], [0., 1/b, 0.], [0., 0., 1/c]])
    transform = evecs.dot(D).dot(evecs.T)

    return offset.tolist(), transform.tolist()


def compute_calibrated_data(x, y, z, offset, transform):
    """
    Map the raw x, y, z accelerometer or magnetometer vector onto the
    calibrated unit sphere.
    """
    v = np.array([[x], [y], [z]])
    offset = np.array(offset)
    transform = np.array(transform)
    v = transform.dot(v - offset)

    return v.item(0), v.item(1), v.item(2)


def handle_exit(signal, frame) :
    """Close files if necessary."""
    global logfile, calfile

    if not logfile.closed:
        print ('Closing', CAL_DATA_FILE)
        logfile.close()
    if not calfile.closed:
        print ('Closing', CAL_MINMAX_FILE)
        calfile.close()

    sys.exit(0)


def callback(data):
    global logfile, x_min, y_min, z_min, x_max, y_max, z_max, roll, pitch
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global collecting_data, calculating_data

    mag = data.data[0:3]
    if collecting_data:
        if mag[0] < x_min :
            x_min = mag[0]
        if mag[0] > x_max :
            x_max = mag[0]

        if mag[1] < y_min :
            y_min = mag[1]
        if mag[1] > y_max :
            y_max = mag[1]

        if mag[2] < z_min :
            z_min = mag[2]
        if mag[2] > z_max :
            z_max = mag[2]

        logfile.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(*data.data))
    elif calculating_data:
        acc = data.data[3:6]
        (acc_x, acc_y, acc_z) = compute_calibrated_data(
            acc[0],
            acc[1],
            acc[2],
            acc_offsets,
            acc_transform
        )
        tmp = -acc_x
        acc_x = acc_y
        acc_y = tmp

        # From: Computing tilt measurement and tilt-compensated e-compass
        # www.st.com/resource/en/design_tip/dm00269987.pdf
        roll = math.atan2(acc_y, acc_z)
        Gz2 = (acc_y * math.sin(roll) + acc_z * math.cos(roll))
        pitch = math.atan(-acc_x / Gz2)

    return


def error(x, y, z, offsets, transform):
    """
    Compute the sum of squared errors, SSE, of data in numpy arrays x, y, z.
    Errors are the distances of the calibrated points from the surface of the
    unit sphere.
    """
    v = np.array([x, y, z])
    offsets = np.array(offsets)
    transform = np.array(transform)
    v = transform.dot(v - offsets)
    # sse = np.sum(np.square(np.sqrt(np.sum(np.square(v), 0)) - 1))
    var_err = np.var(np.sqrt(np.sum(np.square(v), 0)) - 1)
    return var_err


def deg(rad):
    return rad * 180 / math.pi

def check_calibration() :
    try:
        print('Loading', CAL_DATA_FILE, 'for examination')
        data = np.loadtxt(CAL_DATA_FILE, delimiter=',')
        mag_x = data[:,0]
        mag_y = data[:,1]
        mag_z = data[:,2]
        acc_x = data[:,3]
        acc_y = data[:,4]
        acc_z = data[:,5]
        (mag_offsets, mag_transform) = ellipsoid_fit(mag_x, mag_y, mag_z)
        (acc_offsets, acc_transform) = ellipsoid_fit(acc_x, acc_y, acc_z)
        calculating_data = True

        mag_var_err = error(mag_x, mag_y, mag_z, mag_offsets, mag_transform)
        acc_var_err = error(acc_x, acc_y, acc_z, acc_offsets, acc_transform)
        num_pts = len(mag_x)
        minutes = num_pts // 50 // 60  # 50 hz, 60 sec/min
        seconds = num_pts // 50 % 60  # 50 hz, 60 sec/min

        print('Num data points in file:', num_pts)
        print('Approx time spent calibrating: {}:{:02}'.format(minutes, seconds))
        print('Magnetometer v[Err]:', mag_var_err)
        print('Accelerometer v[Err]:', acc_var_err)

        if math.isnan(mag_var_err) or abs(mag_var_err) >= 1e-3 :
            raise ValueError("The magnetometer fit is too poor to use.")
        
        if math.isnan(acc_var_err) or abs(acc_var_err) >= 3e-3 :
            raise ValueError("The accelerometer fit is too poor to use.")

        print('Checking roll and pitch...')
        rolls = []
        pitches = []
        rate = rospy.Rate(50)
        for i in range(20):
            rolls.append(roll)
            pitches.append(pitch)
            rate.sleep()
        avg_roll = deg(np.average(rolls))
        avg_pitch = deg(np.average(pitches))

        print('Average roll: {:6.3f} deg'.format(avg_roll))
        print('Average pitch: {:6.3f} deg'.format(avg_pitch))

        if abs(avg_roll) > ROLL_PITCH_TOLERANCE:
            raise ValueError('Roll exceeds tolerance threshold of {:.1f} deg.'.format(
                ROLL_PITCH_TOLERANCE)
            )
        if abs(avg_pitch) > ROLL_PITCH_TOLERANCE:
            raise ValueError('Pitch exceeds tolerance threshold of {:.1f} deg.'.format(
                ROLL_PITCH_TOLERANCE)
            )

    except Exception as e:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        
        os.remove(CAL_DATA_FILE)
        os.remove(CAL_MINMAX_FILE)
        
        print ('''
*******************************************************************************
** 
** 
** ERROR: CALIBRATION FAILED 
**
** 
*******************************************************************************

The calibration data that was gathered failed to result in a good calibration
fit. This is most likely because the calibration procedure was not done slowly
or completely enough. 

****** THE CALIBRATION FILES HAVE BEEN REMOVED *******

Using a poor calibration will cause poor performance and hurt teams. Please
recalibrate. 

''')

if __name__ == '__main__' :
    global logfile, calfile
    global x_min, y_min, z_min, x_max, y_max, z_max, roll, pitch
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global collecting_data, calculating_data

    atexit.register(check_calibration)
    signal.signal(signal.SIGINT, handle_exit)
    
    logfile = open(CAL_DATA_FILE, 'w')
    calfile = open(CAL_MINMAX_FILE, 'w')

    collecting_data = True
    calculating_data = False

    x_min = sys.maxint
    y_min = x_min
    z_min = x_min
    x_max = -sys.maxint - 1
    y_max = x_max
    z_max = x_max
    acc_offsets = [[0], [0], [0]]
    acc_transform = [[1., 0, 0],
                     [0, 1., 0],
                     [0, 0, 1.]]
    mag_offsets = [[0], [0], [0]]
    mag_transform = [[1., 0, 0],
                     [0, 1., 0],
                     [0, 0, 1.]]
    roll = 0
    pitch = 0

    ROLL_PITCH_TOLERANCE = 1.5  # degrees

    rospy.init_node('CALIBRATE')
    rospy.Subscriber("/imu_raw", Int32MultiArray, callback)

    try:
        rospy.wait_for_message('/imu_raw', Int32MultiArray, timeout=5)
    except rospy.ROSException as e:
        print('Timeout exceeded for /imu_raw topic. Closing files and exiting.')
        calfile.close()
        logfile.close()
        sys.exit(1)

    print('Writing IMU data to file. Start calibration procedure now.')
    raw_input('When finished, put rover down on level ground and press enter.')
    collecting_data = False

    print ('Saving', CAL_MINMAX_FILE)
    calfile.write('min: {{ {}, {}, {} }} max: {{ {}, {}, {} }}\n'.format(x_min, y_min, z_min, x_max, y_max, z_max))
    calfile.close()

    print ('Closing', CAL_DATA_FILE)
    logfile.close()

