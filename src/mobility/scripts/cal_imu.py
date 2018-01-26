#! /usr/bin/env python
"""
cal_lib.py - Ellipsoid into Sphere calibration library based upon numpy and linalg
Copyright (C) 2012 Fabio Varesano <fabio at varesano dot net>
Development of this code has been supported by the Department of Computer Science,
Universita' degli Studi di Torino, Italy within the Piemonte Project
http://www.piemonte.di.unito.it/
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
from __future__ import print_function
import json
import math
import numpy
from numpy import linalg
import sys
import tf
import message_filters
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Vector3, Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty, EmptyResponse

def calibrate(x, y, z):
    # H = numpy.array([x, y, z, -y**2, -z**2, numpy.ones([len(x), 1])])
    H = numpy.array([x, y, z, -y**2, -z**2, numpy.ones([len(x)])])
    H = numpy.transpose(H)
    w = x**2

    (X, residues, rank, shape) = linalg.lstsq(H, w)

    OSx = X[0] / 2
    OSy = X[1] / (2 * X[3])
    OSz = X[2] / (2 * X[4])

    A = X[5] + OSx**2 + X[3] * OSy**2 + X[4] * OSz**2
    B = A / X[3]
    C = A / X[4]

    SCx = numpy.sqrt(A)
    SCy = numpy.sqrt(B)
    SCz = numpy.sqrt(C)

    # type conversion from numpy.float64 to standard python floats
    offsets = [OSx, OSy, OSz]
    scale = [SCx, SCy, SCz]

    offsets = map(numpy.asscalar, offsets)
    scale = map(numpy.asscalar, scale)

    return (offsets, scale)


def calibrate_from_file(file_name):
    samples_f = open(file_name, 'r')
    samples_x = []
    samples_y = []
    samples_z = []
    for line in samples_f:
        reading = line.split()
        if len(reading) == 3:
            samples_x.append(int(reading[0]))
            samples_y.append(int(reading[1]))
            samples_z.append(int(reading[2]))

    return calibrate(
        numpy.array(samples_x),
        numpy.array(samples_y),
        numpy.array(samples_z)
    )


def compute_calibrated_data(x, y, z, offsets, scale):
    output = []
    output.append((x - offsets[0]) / scale[0])
    output.append((y - offsets[1]) / scale[1])
    output.append((z - offsets[2]) / scale[2])
    return output


def imu_callback(imu_msg, acc_raw_msg, mag_raw_msg):
    global calibrating, acc_offsets, acc_scales, mag_offsets, mag_scales
    acc_cal = Vector3Stamped()
    acc_cal.header = acc_raw_msg.header
    mag_cal = Vector3Stamped()
    mag_cal.header = mag_raw_msg.header
    imu_cal = Imu()
    imu_cal.header = imu_msg.header
    imu_cal.angular_velocity = imu_msg.angular_velocity


    if calibrating:
        acc_data[0].append(acc_raw_msg.vector.x)
        acc_data[1].append(acc_raw_msg.vector.y)
        acc_data[2].append(acc_raw_msg.vector.z)
        mag_data[0].append(mag_raw_msg.vector.x)
        mag_data[1].append(mag_raw_msg.vector.y)
        mag_data[2].append(mag_raw_msg.vector.z)

        (acc_offsets, acc_scales) = calibrate(
            numpy.array(acc_data[0]),
            numpy.array(acc_data[1]),
            numpy.array(acc_data[2]),
        )
        (mag_offsets, mag_scales) = calibrate(
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
                    KeyValue(key='Accel Scales', value=str(acc_scales)),
                    KeyValue(key='Mag Offsets', value=str(mag_offsets)),
                    KeyValue(key='Mag Scales', value=str(mag_scales))
                ]
            )
        ]
        imu_diag_pub.publish(diag_msg)

    # TODO calc calibrated data
    (acc_x, acc_y, acc_z) = compute_calibrated_data(
        acc_raw_msg.vector.x,
        acc_raw_msg.vector.y,
        acc_raw_msg.vector.z,
        acc_offsets,
        acc_scales
    )
    acc_cal.vector.x = acc_x
    acc_cal.vector.y = acc_y
    acc_cal.vector.z = acc_z

    # Convert accelerometer digits to milligravities, then to gravities, and finally to meters per second squared
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
        mag_scales
    )
    mag_cal.vector.x = mag_x
    mag_cal.vector.y = mag_y
    mag_cal.vector.z = mag_z

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


def start_callback(req):
    global calibrating, acc_data, mag_data
    global acc_offsets, acc_scales, mag_offsets, mag_scales
    calibrating = True
    acc_data = [[], [], []]
    mag_data = [[], [], []]
    acc_offsets = []
    acc_scales = []
    mag_offsets = []
    mag_scales = []
    return EmptyResponse()


def store_callback(req):
    global calibrating, cal, rover
    global acc_offsets, acc_scales, mag_offsets, mag_scales
    calibrating = False
    cal['acc_offsets'] = acc_offsets
    cal['acc_scales'] = acc_scales
    cal['mag_offsets'] = mag_offsets
    cal['mag_scales'] = mag_scales
    with open('/home/robot/'+rover+'_calibration.json', 'w') as f:
        f.write(json.dumps(cal, sort_keys=True, indent=2))
    return EmptyResponse()


if __name__ == "__main__":
    global rover, calibrating, cal, acc_data, mag_data
    global acc_offsets, acc_scales, mag_offsets, mag_scales

    # print "Calibrating from acc.txt"
    # (offsets, scale) = calibrate_from_file("acc.txt")
    # print "Offsets:"
    # print offsets
    # print "Scales:"
    # print scale
    #
    # print "Calibrating from magn.txt"
    # (offsets, scale) = calibrate_from_file("magn.txt")
    # print "Offsets:"
    # print offsets
    # print "Scales:"
    # print scale

    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rover = sys.argv[1]
    rospy.init_node(rover + '_IMUCAL')
    calibrating = False
    cal = {}
    acc_data = [[], [], []]
    mag_data = [[], [], []]

    try:
        with open('/home/robot/'+rover+'_calibration.json', 'r') as f:
            cal = json.loads(f.read())
    except IOError as e:
        rospy.loginfo('No IMU calibration file found.')
    except ValueError as e:
        rospy.loginfo('Invalid IMU calibration file. Starting from scratch.')

    if not cal:
        acc_offsets = [0, 0, 0]
        acc_scales = [1, 1, 1]
        mag_offsets = [0, 0, 0]
        mag_scales = [1, 1, 1]
    else:
        acc_offsets = cal['acc_offsets']
        acc_scales = cal['acc_scales']
        mag_offsets = cal['mag_offsets']
        mag_scales = cal['mag_scales']


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
    start_server = rospy.Service(
        rover + '/start_imu_calibration',
        Empty,
        start_callback
    )
    store_server = rospy.Service(
        rover + '/store_imu_calibration',
        Empty,
        store_callback
    )

    rospy.spin()

