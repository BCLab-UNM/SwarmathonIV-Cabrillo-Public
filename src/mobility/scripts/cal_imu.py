#! /usr/bin/env python
"""
Ellipsoid fit, from:
https://github.com/aleksandrbazhin/ellipsoid_fit_python

Adapted for ROS by Darren Churchill, Cabrillo College.

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

todo: Put paths to competition files when UNM finalizes location and filename
todo: ROS params for calibration matrices? at least misalignment and gyro in case node dies for some reason
"""
from __future__ import print_function
import json
import math
import numpy
import sys
import tf
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from std_srvs.srv import Trigger, TriggerResponse

from swarmie_msgs.msg import SwarmieIMU


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


def calc_misalignment(H, current_misalign):
    """
    Misalignment calibration.
    From: https://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf

    We will only perform calibration for the rotation around the z-axis.

    Calculates compensation to align the IMU sensor axis to the rover's body
    axis using numpy array, H, the data from a 2D rotation around one axis.
    """
    w = numpy.sqrt(numpy.sum(numpy.square(H), axis=1)).reshape(-1, 1)
    try:
        (X, residuals, rank, shape) = numpy.linalg.lstsq(H, w)
        R = X / numpy.sqrt((numpy.sum(X**2)))
        misalignment = numpy.array(current_misalign)
        misalignment[:,2] = R.T
        misalignment = misalignment.tolist()
    except ValueError as e:
        rospy.logwarn("Misalignment data can't be fit yet.")

    # Z-position of column-z in misalignment matrix should be positive.
    # It's calculated as a negative value because spinning the rover in
    # place on level ground actually is a z-up rotation, and the
    # calibration calculation assumes a z-down rotation, so the sign
    # gets reversed.
    misalignment[2][2] = abs(misalignment[2][2])

    return misalignment


def compute_calibrated_data(x, y, z, offset, transform, use_misalignment=True):
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
    if calibrating == 'misalignment' or use_misalignment is False:
        M_m = numpy.eye(3)
    else:
        M_m = numpy.array(misalignment)

    transform = numpy.array(transform)
    T = M_m.dot(transform)
    v = T.dot(v - offset)

    return v.item(0), v.item(1), v.item(2)


def publish_diagnostic_msg():
    """
    Helper to imu_callback. Publishes a DiagnosticArray containing calibration
    information.
    """
    global acc_offsets, acc_transform, mag_offsets, mag_transform, misalignment
    global gyro_bias, gyro_scale
    global imu_diag_pub

    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = rospy.Time.now()
    diag_msg.status = [
        DiagnosticStatus(
            level = DiagnosticStatus.OK,
            name = 'IMU Calibration Info',
            values = [
                KeyValue(key='Gyro Bias', value=str(gyro_bias)),
                KeyValue(key='Gyro Scale', value=str(gyro_scale)),
                KeyValue(key='Accel Offsets', value=str(acc_offsets)),
                KeyValue(key='Accel Transform', value=str(acc_transform)),
                KeyValue(key='Mag Offsets', value=str(mag_offsets)),
                KeyValue(key='Mag Transform', value=str(mag_transform)),
                KeyValue(key='Misalignment', value=str(misalignment))
            ]
        )
    ]
    imu_diag_pub.publish(diag_msg)
    return


def imu_callback(imu_raw_msg):
    """
    Callback for the SwarmieIMU message containing raw accelerometer and
    magnetometer data.

    Calibrates IMU by fitting an ellipsoid, or calculating the misalignment
    matrix if we are in either of those states.

    Computes calibrated accelerometer and magnetometer data, transformed from
    the IMU's frame into the rover's frame, calculates roll, pitch, yaw, and
    publishes a calibrated IMU message.
    """
    global calibrating, acc_data, mag_data, gyro_data
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global gyro_bias, gyro_scale, gyro_start_time, gyro_status_msg
    global misalignment

    # In case someone forgets to exit either calibration state.
    DATA_SIZE_LIMIT = 3000  # 5 min worth of data at 10 Hz
    MIN_DATA_SIZE = 50

    # Message for raw, calibrated data
    imu_cal_data = SwarmieIMU()
    imu_cal_data.header = imu_raw_msg.header
    imu_cal_data.angular_velocity = imu_raw_msg.angular_velocity

    # IMU Message
    imu_cal = Imu()
    imu_cal.header = imu_raw_msg.header

    if calibrating == 'gyro_bias':
        gyro_data[0].append(imu_raw_msg.angular_velocity.x)
        gyro_data[1].append(imu_raw_msg.angular_velocity.y)
        gyro_data[2].append(imu_raw_msg.angular_velocity.z)
        gyro_bias[0][0] = numpy.mean(gyro_data[0])
        gyro_bias[1][0] = numpy.mean(gyro_data[1])
        gyro_bias[2][0] = numpy.mean(gyro_data[2])

    (gyro_x, gyro_y, gyro_z) = compute_calibrated_data(
        imu_raw_msg.angular_velocity.x,
        imu_raw_msg.angular_velocity.y,
        imu_raw_msg.angular_velocity.z,
        gyro_bias,
        gyro_scale,
        use_misalignment=False
    )
    # Convert gyroscope digits to millidegrees per second, then to degrees per
    # second, and finally to radians per second.
    # axes mismatched as in arduino code
    imu_cal.angular_velocity.x = gyro_y*8.75/1000*(math.pi/180)
    imu_cal.angular_velocity.y = -gyro_x*8.75/1000*(math.pi/180)
    imu_cal.angular_velocity.z = gyro_z*8.75/1000*(math.pi/180)

    if calibrating == 'gyro_scale':
        current_time = rospy.Time.now().to_sec()
        if current_time - gyro_start_time < 10:
            gyro_status_msg = (
                    rover + ': Collecting data from first gyro rotation.'
            )
            gyro_data[0].append(imu_raw_msg.header.stamp.to_sec())
            gyro_data[1].append(imu_cal.angular_velocity.z)
        elif current_time - gyro_start_time < 20:
            gyro_status_msg = (
                    rover + ': Collecting data from second gyro rotation.'
            )
            gyro_data[2].append(imu_raw_msg.header.stamp.to_sec())
            gyro_data[3].append(imu_cal.angular_velocity.z)
        else:
            angle_1 = numpy.trapz(gyro_data[1], x=gyro_data[0])
            angle_2 = numpy.trapz(gyro_data[3], x=gyro_data[2])
            z_scale = (abs(math.pi/angle_1) + abs(math.pi/angle_2)) / 2
            gyro_scale[2][2] = z_scale
            msg = (rover + ': Finished collecting gyro rotation data. ' +
                   'Z-Scale: ' + str(z_scale))
            info_log.publish(msg)
            store_calibration(EmptyRequest())

    if calibrating == 'imu':
        acc_data[0].append(imu_raw_msg.accelerometer.x)
        acc_data[1].append(imu_raw_msg.accelerometer.y)
        acc_data[2].append(imu_raw_msg.accelerometer.z)
        mag_data[0].append(imu_raw_msg.magnetometer.x)
        mag_data[1].append(imu_raw_msg.magnetometer.y)
        mag_data[2].append(imu_raw_msg.magnetometer.z)

        if len(acc_data[0]) > DATA_SIZE_LIMIT:
            rospy.logwarn(
                'IMU calibration timeout exceeded. ' +
                'Saving current calculated matrix.'
            )
            store_calibration(EmptyRequest())

        # Don't fit until some data has been collected.
        # May still get a runtime warning until enough data in all directions
        # has been collected.
        elif len(acc_data[0]) > MIN_DATA_SIZE:
            (acc_offsets, acc_transform) = ellipsoid_fit(
                numpy.array(acc_data[0]),
                numpy.array(acc_data[1]),
                numpy.array(acc_data[2])
            )
            (mag_offsets, mag_transform) = ellipsoid_fit(
                numpy.array(mag_data[0]),
                numpy.array(mag_data[1]),
                numpy.array(mag_data[2])
            )
            publish_diagnostic_msg()

    (acc_x, acc_y, acc_z) = compute_calibrated_data(
        imu_raw_msg.accelerometer.x,
        imu_raw_msg.accelerometer.y,
        imu_raw_msg.accelerometer.z,
        acc_offsets,
        acc_transform
    )
    imu_cal_data.accelerometer.x = acc_x
    imu_cal_data.accelerometer.y = acc_y
    imu_cal_data.accelerometer.z = acc_z

    # swap x and y to orient measurements in the rover's frame
    tmp = -acc_x
    acc_x = acc_y
    acc_y = tmp

    # Scale accelerations back to m/s**2 after being fit to the unit sphere.
    imu_cal.linear_acceleration.x = acc_x * 9.81
    imu_cal.linear_acceleration.y = acc_y * 9.81
    imu_cal.linear_acceleration.z = acc_z * 9.81

    (mag_x, mag_y, mag_z) = compute_calibrated_data(
        imu_raw_msg.magnetometer.x,
        imu_raw_msg.magnetometer.y,
        imu_raw_msg.magnetometer.z,
        mag_offsets,
        mag_transform
    )
    imu_cal_data.magnetometer.x = mag_x
    imu_cal_data.magnetometer.y = mag_y
    imu_cal_data.magnetometer.z = mag_z

    if calibrating == 'misalignment':
        mag_data[0].append(mag_x)
        mag_data[1].append(mag_y)
        mag_data[2].append(mag_z)

        if len(mag_data[0]) > DATA_SIZE_LIMIT:
            rospy.logwarn(
                'Misalignment calibration timeout exceeded. ' +
                'Saving current calculated matrix.'
            )
            store_calibration(EmptyRequest())

        # Wait until some data has been collected.
        elif len(mag_data[0]) > MIN_DATA_SIZE:
            data = numpy.array(mag_data)
            misalignment = calc_misalignment(data.T, misalignment)
            publish_diagnostic_msg()

    # swap x and y to orient measurements in the rover's frame
    # Done here so the misalignment calibration can use data in the original
    # IMU frame
    tmp = -mag_x
    mag_x = mag_y
    mag_y = tmp

    # From: Computing tilt measurement and tilt-compensated e-compass
    # www.st.com/resource/en/design_tip/dm00269987.pdf
    roll = math.atan2(acc_y, acc_z)
    Gz2 = (acc_y * math.sin(roll) + acc_z * math.cos(roll))

    # Gimbal-lock. Special case when pitched + or - 90 deg.
    # Heading is unreliable here, but plenty of other bad things will be
    # happening if the rover is ever in this position.
    if abs(Gz2) < 0.01:
        if acc_x > 0:
            rospy.loginfo('Special compass case: pitch is -90 deg')
            pitch = -math.pi / 2
        else:
            rospy.loginfo('Special compass case: pitch is +90 deg')
            pitch = math.pi / 2
        alpha = .01  # Can be set from [0.01 - 0.05]
        roll = math.atan2(acc_y, acc_z + acc_x * alpha)
    else:
        pitch = math.atan(-acc_x / Gz2)

    By2 = mag_z * math.sin(roll) - mag_y * math.cos(roll)
    Bz2 = mag_y * math.sin(roll) + mag_z * math.cos(roll)
    Bx3 = mag_x * math.cos(pitch) + Bz2 * math.sin(pitch)
    yaw = math.pi / 2 + math.atan2(By2, Bx3)

    imu_cal.orientation = Quaternion(
        *tf.transformations.quaternion_from_euler(
            roll,
            pitch,
            yaw
        )
    )

    imu_cal_data_pub.publish(imu_cal_data)
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


def start_gyro_bias_calibration(req):
    """Start gyro bias calibration. Rover should remain static for this."""
    global calibrating, gyro_data, gyro_bias
    calibrating = 'gyro_bias'
    gyro_data = [[], [], []]
    gyro_bias = [[0], [0], [0]]
    return EmptyResponse()


def start_gyro_scale_calibration(req):
    """
    Start gyro scale calibration. Rover should rotate 180 degrees in one
    direction during first 10 seconds, and 180 degress in opposite direction
    during second 10 seconds.
    """
    global calibrating, gyro_data, gyro_scale, gyro_start_time, gyro_timer
    calibrating = 'gyro_scale'
    gyro_data = [[], [], [], []]
    gyro_scale = [[1., 0, 0],
                  [0, 1., 0],
                  [0, 0, 1.]]
    # Timer to throttle gyro status messages to 1 Hz
    gyro_timer = rospy.Timer(rospy.Duration(1), log_gyro_status)
    gyro_start_time = rospy.Time.now().to_sec()
    return EmptyResponse()


def log_gyro_status(event):
    """
    Helper to log gyro scale calibration status messages to /infoLog. Used
    as the callback to a Timer initialized in start_gyro_scale_calibration()
    """
    global gyro_status_msg
    msg = String(gyro_status_msg)
    info_log.publish(msg)


def store_calibration(req):
    """
    Stores all current calibration matrices and resets dataset lists.
    """
    global calibrating, cal, rover, acc_data, mag_data, gyro_data
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global misalignment, gyro_bias, gyro_scale, gyro_timer

    CAL_FILE_PATH = rospy.get_param(
        '~calibration_file_path',
        default='/home/robot/'
    )
    calibrating = None
    acc_data = [[], [], []]
    mag_data = [[], [], []]
    gyro_data = [[], [], []]

    if gyro_timer is not None:
        gyro_timer.shutdown()
        gyro_timer = None

    cal['acc_offsets'] = acc_offsets
    cal['acc_transform'] = acc_transform
    cal['mag_offsets'] = mag_offsets
    cal['mag_transform'] = mag_transform
    cal['misalignment'] = misalignment
    cal['gyro_bias'] = gyro_bias
    cal['gyro_scale'] = gyro_scale

    with open(CAL_FILE_PATH+rover+'_calibration.json', 'w') as f:
        f.write(json.dumps(cal, sort_keys=True, indent=2))

    publish_diagnostic_msg()

    return EmptyResponse()


def load_json_calibration_file(req):
    """
    Loads calibration from JSON file if one exists.
    Returns TriggerResponse indicating whether the file was found and
    loaded successfully.
    """
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global gyro_bias, gyro_scale, misalignment

    CAL_FILE_PATH = rospy.get_param(
        '~calibration_file_path',
        default='/home/robot/'
    )
    cal = {}
    response = TriggerResponse()

    try:
        with open(CAL_FILE_PATH + rover + '_calibration.json', 'r') as f:
            cal = json.loads(f.read())
            rospy.loginfo(
                rover + ': IMU calibration file found at ' +
                CAL_FILE_PATH + rover + '_calibration.json'
            )
            response.success = True
            response.message = (rover + ': IMU calibration file found at ' +
                                CAL_FILE_PATH + rover + '_calibration.json')
    except IOError as e:
        rospy.logerr(
            rover +
            ': No IMU calibration file found. Keeping current calibration.'
        )
        response.success = False
        response.message = (rover +
                            ': No IMU calibration file found. Keeping ' +
                            'current calibration.')
    except ValueError as e:
        rospy.logerr(
            rover +
            ': Invalid IMU calibration file. Keeping current calibration.'
        )
        response.success = False
        response.message = (rover +
                            ': Invalid IMU calibration file. Keeping ' +
                            'current calibration.')

    if response.success:
        acc_offsets = cal['acc_offsets']
        acc_transform = cal['acc_transform']
        mag_offsets = cal['mag_offsets']
        mag_transform = cal['mag_transform']
        misalignment = cal['misalignment']
        gyro_bias = cal['gyro_bias']
        gyro_scale = cal['gyro_scale']
        publish_diagnostic_msg()

    return response


if __name__ == "__main__":
    global rover, calibrating, cal, acc_data, mag_data, gyro_data
    global acc_offsets, acc_transform, mag_offsets, mag_transform
    global gyro_bias, gyro_scale, gyro_start_time, gyro_timer, gyro_status_msg
    global misalignment

    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rover = sys.argv[1]
    rospy.init_node(rover + '_IMU')
    calibrating = None
    gyro_timer = None
    cal = {}
    LOAD_RAW_DATA = rospy.get_param(
        '~load_raw_data',
        default=False
    )
    RAW_DATA_PATH = rospy.get_param(
        '~raw_data_path',
        default='/home/robot/'
    )
    CAL_FILE_PATH = rospy.get_param(
        '~calibration_file_path',
        default='/home/robot/'
    )
    # Data is stored in a list of lists, which is converted to a numpy array
    # when needed.
    acc_data = [[], [], []]
    mag_data = [[], [], []]
    gyro_data = [[], [], []]

    if LOAD_RAW_DATA:
        try:
            raw_data = numpy.loadtxt(
                RAW_DATA_PATH + 'raw_cal.txt',
                delimiter=','
            )
            (cal['mag_offsets'], cal['mag_transform']) = ellipsoid_fit(
                raw_data[:,0],
                raw_data[:,1],
                raw_data[:,2]
            )
            (cal['acc_offsets'], cal['acc_transform']) = ellipsoid_fit(
                raw_data[:,3],
                raw_data[:,4],
                raw_data[:,5]
            )
            cal['misalignment'] = [[1., 0, 0],
                                   [0, 1., 0],
                                   [0, 0, 1.]]
            cal['gyro_bias'] = [[0], [0], [0]]
            cal['gyro_scale'] = [[1., 0, 0],
                                 [0, 1., 0],
                                 [0, 0, 1.]]
            rospy.loginfo(
                rover + ': IMU raw data file loaded from ' +
                RAW_DATA_PATH+'raw_cal.txt'
            )
        except IOError as e:
            rospy.logerr(
                rover +
                ': Error reading raw IMU data file. Starting from scratch.'
            )

    else:
        try:
            with open(CAL_FILE_PATH + rover + '_calibration.json', 'r') as f:
                cal = json.loads(f.read())
            rospy.loginfo(
                rover + ': IMU calibration file found at ' + CAL_FILE_PATH +
                rover + '_calibration.json'
            )
        except IOError as e:
            rospy.logerr(rover+': No IMU calibration file found.')
        except ValueError as e:
            rospy.logerr(
                rover +
                ': Invalid IMU calibration file. Starting from scratch.'
            )

    # Calibration matrices are stored as lists and converted to numpy arrays
    # when needed.
    if not cal:
        acc_offsets = [[0], [0], [0]]
        acc_transform = [[1., 0, 0],
                         [0, 1., 0],
                         [0, 0, 1.]]
        mag_offsets = [[0], [0], [0]]
        mag_transform = [[1., 0, 0],
                         [0, 1., 0],
                         [0, 0, 1.]]
        misalignment = [[1., 0, 0],
                        [0, 1., 0],
                        [0, 0, 1.]]
        gyro_bias = [[0], [0], [0]]
        gyro_scale = [[1., 0, 0],
                      [0, 1., 0],
                      [0, 0, 1.]]
    else:
        acc_offsets = cal['acc_offsets']
        acc_transform = cal['acc_transform']
        mag_offsets = cal['mag_offsets']
        mag_transform = cal['mag_transform']
        misalignment = cal['misalignment']
        gyro_bias = cal['gyro_bias']
        gyro_scale = cal['gyro_scale']


    # Subscribers
    imu_raw_sub = rospy.Subscriber(
        rover + '/imu/raw',
        SwarmieIMU,
        imu_callback,
        queue_size=10
    )

    # Publishers
    imu_pub = rospy.Publisher(
        rover + '/imu',
        Imu,
        queue_size=10
    )
    imu_diag_pub = rospy.Publisher(
        rover + '/imu/cal_diag',
        DiagnosticArray,
        queue_size=10,
        latch=True
    )
    imu_cal_data_pub = rospy.Publisher(
        rover + '/imu/raw/calibrated',
        SwarmieIMU,
        queue_size=10
    )
    info_log = rospy.Publisher(
        '/infoLog',
        String,
        queue_size=10
    )

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
    start_gyro_cal = rospy.Service(
        rover + '/start_gyro_bias_calibration',
        Empty,
        start_gyro_bias_calibration
    )
    start_gyro_cal = rospy.Service(
        rover + '/start_gyro_scale_calibration',
        Empty,
        start_gyro_scale_calibration
    )
    load_json_file = rospy.Service(
        rover + '/load_json_calibration_file',
        Trigger,
        load_json_calibration_file
    )

    # Publish current calibration once:
    publish_diagnostic_msg()

    rospy.spin()

