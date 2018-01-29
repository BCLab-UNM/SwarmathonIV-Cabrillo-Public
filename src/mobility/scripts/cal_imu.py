#! /usr/bin/env python
"""
Ellipsoid fit, from:
https://github.com/aleksandrbazhin/ellipsoid_fit_python

Apdapted to work in ROS.

The MIT License (MIT)

Copyright (c) 2016 aleksandrbazhin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

todo: is storing the globals as lists and building numpy arrays every time ok?
It looks like numpy.append doens't occur in-place, so it might not be
practical to store the data in a numpy array either
"""
from __future__ import print_function
import json
import math
import numpy
import sys
import tf
import message_filters
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3, Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse

def ellipsoid_fit(x, y, z):
    """
    Modified from:
    http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
    """
    D = numpy.array([x*x,
                     y*y,
                     z*z,
                     2 * x*y,
                     2 * x*z,
                     2 * y*z,
                     2 * x,
                     2 * y,
                     2 * z])
    DT = D.conj().T
    v = numpy.linalg.solve(D.dot(DT), D.dot(numpy.ones(numpy.size(x))))
    A = numpy.array([[v[0], v[3], v[4], v[6]],
                     [v[3], v[1], v[5], v[7]],
                     [v[4], v[5], v[2], v[8]],
                     [v[6], v[7], v[8], -1]])
    center = numpy.linalg.solve(-A[:3,:3], [[v[6]], [v[7]], [v[8]]])
    T = numpy.eye(4)
    T[3,:3] = center.T
    R = T.dot(A).dot(T.conj().T)
    evals, evecs = numpy.linalg.eig(R[:3,:3] / -R[3,3])
    radii = numpy.sqrt(1. / evals)
    offset = center

    a, b, c = radii
    D = numpy.array([[1/a, 0., 0.], [0., 1/b, 0.], [0., 0., 1/c]])
    transform = evecs.dot(D).dot(evecs.T)

    return offset.tolist(), transform.tolist()


def compute_calibrated_data(x, y, z, offset, transform):
    """
    Map the raw x, y, z accelerometer or magnetometer vector onto the
    calibrated unit sphere. Skips misalignment transformation if we are in
    the misalignment calibration state.
    """
    global calibrating, misalignment
    v = numpy.array([[x],
                     [y],
                     [z]])
    offset = numpy.array(offset)

    # Misalignment calibration needs to get only hard-iron and soft-iron
    # calibrated data.
    if calibrating == 'misalignment':
        M_m = numpy.eye(3)
    else:
        M_m = numpy.array(misalignment)
    transform = numpy.array(transform)
    T = M_m.dot(transform)
    v = T.dot(v - offset)

    return v.item(0), v.item(1), v.item(2)


def imu_callback(imu_msg, acc_raw_msg, mag_raw_msg):
    global calibrating, acc_offsets, acc_transform, mag_offsets, mag_transform
    global misalignment
    acc_cal = Vector3Stamped()
    acc_cal.header = acc_raw_msg.header
    mag_cal = Vector3Stamped()
    mag_cal.header = mag_raw_msg.header
    imu_cal = Imu()
    imu_cal.header = imu_msg.header
    imu_cal.angular_velocity = imu_msg.angular_velocity


    if calibrating == 'imu':
        acc_data[0].append(acc_raw_msg.vector.x)
        acc_data[1].append(acc_raw_msg.vector.y)
        acc_data[2].append(acc_raw_msg.vector.z)
        mag_data[0].append(mag_raw_msg.vector.x)
        mag_data[1].append(mag_raw_msg.vector.y)
        mag_data[2].append(mag_raw_msg.vector.z)

        # Don't fit until some data has been collected.
        # May still get a runtime warning until enough data in all directions
        # has been collected.
        if len(acc_data[0]) > 10:
            (acc_offsets, acc_transform) = ellipsoid_fit(
                numpy.array(acc_data[0]),
                numpy.array(acc_data[1]),
                numpy.array(acc_data[2]),
            )
            (mag_offsets, mag_transform) = ellipsoid_fit(
                numpy.array(mag_data[0]),
                numpy.array(mag_data[1]),
                numpy.array(mag_data[2]),
            )
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = imu_msg.header.stamp
        diag_msg.status = [
            DiagnosticStatus(
                level = DiagnosticStatus.OK,
                name = 'IMU Calibration Info',
                values = [
                    KeyValue(key='Accel Offsets', value=str(acc_offsets)),
                    KeyValue(key='Accel Transform', value=str(acc_transform)),
                    KeyValue(key='Mag Offsets', value=str(mag_offsets)),
                    KeyValue(key='Mag Transform', value=str(mag_transform))
                ]
            )
        ]
        imu_diag_pub.publish(diag_msg)

    (acc_x, acc_y, acc_z) = compute_calibrated_data(
        acc_raw_msg.vector.x,
        acc_raw_msg.vector.y,
        acc_raw_msg.vector.z,
        acc_offsets,
        acc_transform
    )
    acc_cal.vector.x = acc_x
    acc_cal.vector.y = acc_y
    acc_cal.vector.z = acc_z

    # Convert accelerometer digits to milligravities, then to gravities, and
    # finally to meters per second squared.
    # mismatched x, y as in arduino code
    # todo is this calc still accurate after fitting to a sphere?
    # imu_cal.linear_acceleration.x = acc_y * 0.061 / 1000 * 9.81
    # imu_cal.linear_acceleration.y = -acc_x * 0.061 / 1000 * 9.81
    # imu_cal.linear_acceleration.z = acc_z * 0.061 / 1000 * 9.81

    imu_cal.linear_acceleration.x = acc_y * 9.81
    imu_cal.linear_acceleration.y = -acc_x * 9.81
    imu_cal.linear_acceleration.z = acc_z * 9.81

    (mag_x, mag_y, mag_z) = compute_calibrated_data(
        mag_raw_msg.vector.x,
        mag_raw_msg.vector.y,
        mag_raw_msg.vector.z,
        mag_offsets,
        mag_transform
    )
    mag_cal.vector.x = mag_x
    mag_cal.vector.y = mag_y
    mag_cal.vector.z = mag_z

    if calibrating == 'misalignment':
        mag_data[0].append(mag_x)
        mag_data[1].append(mag_y)
        mag_data[2].append(mag_z)

        # Wait until some data has been collected.
        if len(mag_data[0]) > 50:
            data = numpy.array(mag_data)
            H = data.T
            w = numpy.sqrt(numpy.sum(numpy.square(H), axis=1)).reshape(-1, 1)
            (X, residuals, rank, shape) = numpy.linalg.lstsq(H, w)
            R = X / numpy.sqrt((numpy.sum(X**2)))
            misalignment = numpy.array(misalignment)
            misalignment[:,2] = R.T
            misalignment = misalignment.tolist()

            # Z-position of column-z in misalignment matrix should be positive.
            # It's calculated as a negative value because spinning the rover in
            # place on level ground actually is a z-up rotation, and the
            # calibration calculation assumes a z-down rotation, so the sign
            # gets reversed.
            misalignment[2][2] = abs(misalignment[2][2])

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = imu_msg.header.stamp
            diag_msg.status = [
                DiagnosticStatus(
                    level = DiagnosticStatus.OK,
                    name = 'IMU Calibration Info',
                    values = [
                        KeyValue(key='Misalignment', value=str(misalignment))
                    ]
                )
            ]
            imu_diag_pub.publish(diag_msg)


    # roll = math.atan2(acc_y, acc_z)
    # Gz2 = acc_y * math.sin(roll) + acc_z * math.cos(roll)
    # if abs(Gz2) < 0.01:
    #     if acc_x > 0:
    #         pitch = -math.pi / 2
    #     else:
    #         pitch = math.pi / 2
    # else:
    #     pitch = math.atan(-acc_x / Gz2)
    # yaw = math.atan2(
    #     mag_z*math.sin(roll) - mag_y*math.cos(roll),
    #     mag_x*math.cos(pitch)
    #     + (mag_y*math.sin(roll) + mag_z*math.cos(roll)) * math.sin(pitch)
    # )
    roll = math.atan2(
        imu_cal.linear_acceleration.y,
        math.sqrt(imu_cal.linear_acceleration.x**2
                  + imu_cal.linear_acceleration.z**2)
    )
    pitch = -math.atan2(
        imu_cal.linear_acceleration.x,
        math.sqrt(imu_cal.linear_acceleration.y**2
                  + imu_cal.linear_acceleration.z**2)
    )
    yaw = math.pi + math.atan2(
        -mag_y*math.cos(roll) + mag_z*math.sin(roll),
        mag_x*math.cos(pitch)
        + mag_y*math.sin(pitch)*math.sin(roll)
        + mag_z*math.sin(pitch)*math.cos(roll)
    )
    imu_cal.orientation = Quaternion(
        *tf.transformations.quaternion_from_euler(
            roll,
            pitch,
            yaw
        )
    )

    imu_acc_cal_pub.publish(acc_cal)
    imu_mag_cal_pub.publish(mag_cal)
    imu_pub.publish(imu_cal)

    return


def start_imu_calibration(req):
    """
    Reset accelerometer and magnetometer offset and transform matrices, and
    enter accelerometer and magnetometer calibration state.

    This calibration should be performed before the rover starts operating
    in a new environment.

    Raw accelerometer and magnetometer data is collected and fit to an
    ellipsoid. During this time, the rover should perform three full round
    rotations with its body axis x up or down, y up or down, and z up or
    down. These are slow 2D rotations. Then the rover should also perform
    3D random rotations to put it in as many orientations as possible.

    Calculated calibration matrices can be viewed on the
    /rover/imu/cal_diag topic while calibration is in progress.
    """
    global calibrating, acc_data, mag_data
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    calibrating = 'imu'
    acc_data = [[], [], []]
    mag_data = [[], [], []]
    acc_offsets = [[0], [0], [0]]
    acc_transform = [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]
    mag_offsets = [[0], [0], [0]]
    mag_transform = [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]
    return EmptyResponse()


def start_misalignment_calibration(req):
    """
    Reset misalignment matrix, and enter misalignment calibration state. This
    should be performed after the ellipsoid fit IMU calibration.

    I think this calibration should only need to be performed once for a
    given rover.

    Raw magnetometer data is collected while the rover performs at least one
    slow 2D rotation with its z axis up. This can be performed by having the
    rover spin slowly in place on level ground. This is only one third of
    a typical misalignment calibration procedure, but since the rover only
    operates in two dimensions on relatively level ground, it should be ok
    to skip the x-down and y-down rotations.
    """
    global calibrating, mag_data, misalignment
    calibrating = 'misalignment'
    mag_data = [[], [], []]
    misalignment = [[1., 0, 0],
                    [0, 1., 0],
                    [0, 0, 1.]]
    return EmptyResponse()


def store_calibration(req):
    """
    Stores all current calibration matrices.
    """
    global calibrating, cal, rover
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global misalignment
    calibrating = None

    cal['acc_offsets'] = acc_offsets
    cal['acc_transform'] = acc_transform
    cal['mag_offsets'] = mag_offsets
    cal['mag_transform'] = mag_transform
    cal['misalignment'] = misalignment
    with open('/home/robot/'+rover+'_calibration_alt.json', 'w') as f:
        f.write(json.dumps(cal, sort_keys=True, indent=2))
    return EmptyResponse()


if __name__ == "__main__":
    global rover, calibrating, cal, acc_data, mag_data
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global misalignment

    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rover = sys.argv[1]
    rospy.init_node(rover + '_IMUCAL')
    calibrating = None
    cal = {}
    # Data is stored in a list of lists, which is converted to a numpy array
    # when needed.
    acc_data = [[], [], []]
    mag_data = [[], [], []]

    try:
        with open('/home/robot/'+rover+'_calibration_alt.json', 'r') as f:
            cal = json.loads(f.read())
    except IOError as e:
        rospy.loginfo('No IMU calibration file found.')
    except ValueError as e:
        rospy.loginfo('Invalid IMU calibration file. Starting from scratch.')

    # Calibration matrices are stored as lists and converted to numpy arrays
    # when needed.
    if not cal:
        acc_offsets = [[0], [0], [0]]
        acc_transform = [[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]]
        mag_offsets = [[0], [0], [0]]
        mag_transform = [[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]]
        misalignment = [[1., 0, 0],
                        [0, 1., 0],
                        [0, 0, 1.]]
    else:
        acc_offsets = cal['acc_offsets']
        acc_transform = cal['acc_transform']
        mag_offsets = cal['mag_offsets']
        mag_transform = cal['mag_transform']
        misalignment = cal['misalignment']


    # Subscribers
    imu_sub = message_filters.Subscriber(
        rover + '/imu',
        Imu
    )
    imu_acc_raw_sub = message_filters.Subscriber(
        rover + '/imu/accel/raw',
        Vector3Stamped
    )
    imu_mag_raw_sub = message_filters.Subscriber(
        rover + '/imu/mag/raw',
        Vector3Stamped
    )

    # Publishers
    imu_pub = rospy.Publisher(
        rover + '/imu/calibrated',
        Imu,
        queue_size=10
    )
    imu_diag_pub = rospy.Publisher(
        rover + '/imu/cal_diag',
        DiagnosticArray,
        queue_size=10
    )
    imu_acc_cal_pub = rospy.Publisher(
        rover + '/imu/accel/calibrated',
        Vector3Stamped,
        queue_size=10
    )
    imu_mag_cal_pub = rospy.Publisher(
        rover + '/imu/mag/calibrated',
        Vector3Stamped,
        queue_size=10
    )

    # Synchronizer
    ts = message_filters.TimeSynchronizer(
        [imu_sub, imu_acc_raw_sub, imu_mag_raw_sub],
        10
    )
    ts.registerCallback(imu_callback)

    # Services
    start_imu_cal = rospy.Service(
        rover + '/start_imu_calibration',
        Empty,
        start_imu_calibration
    )
    store_cal = rospy.Service(
        rover + '/store_imu_calibration',
        Empty,
        store_calibration
    )
    start_misalign_cal = rospy.Service(
        rover + '/start_misalignment_calibration',
        Empty,
        start_misalignment_calibration
    )

    rospy.spin()
