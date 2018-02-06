#! /usr/bin/env python
"""
IMU Node. Gets raw IMU data from ABridge, and publishes calibrated IMU
messages.

Performs a 2-D IMU Calibration. Can be performed at the start of a round.

todo: is storing the globals as lists and building numpy arrays every time ok?
It looks like numpy.append doens't occur in-place, so it might not be
practical to store the data in a numpy array either
todo: where should calibration files be stored? add rosparam for path
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

from swarmie_msgs.msg import SwarmieIMU


def ellipse_fit(x, y):
    """
    Fits the data points in x and y to a circle centered at the origin.
    http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html

    Returns 3R x 1C offset matrix, and a 3 x 3 transformation matrix. Only
    first 2 rows and columns are calculated in transform matrix, since this
    is only a 2-D calibration.
    """
    x = x[:,numpy.newaxis]
    y = y[:,numpy.newaxis]

    D =  numpy.hstack((x*x, x*y, y*y, x, y, numpy.ones_like(x)))
    S = numpy.dot(D.T,D)
    C = numpy.zeros([6,6])
    C[0,2] = C[2,0] = 2; C[1,1] = -1
    E, V = numpy.linalg.eig(numpy.dot(numpy.linalg.inv(S), C))
    n = numpy.argmax(numpy.abs(E))
    A = V[:,n]

    center = ellipse_center(A)
    beta = ellipse_angle_of_rotation(A)
    major, minor = ellipse_axis_length(A)

    # Singular Value Decomposition:
    # commons.wikimedia.org/wiki/File:Singular-Value-Decomposition.svg
    # CCW rot through beta
    U = numpy.array([[math.cos(beta), -math.sin(beta)],
                  [math.sin(beta), math.cos(beta)]])
    phi = math.tan(beta) + 1
    alpha = math.atan(phi)
    # CW rot through alpha
    V_star = numpy.array([[math.cos(alpha), math.sin(alpha)],
                       [-math.sin(alpha), math.cos(alpha)]])
    r = math.sqrt(major * minor)  # preserve approximate area
    sigma = numpy.diag([r / minor, r / major])
    transform = numpy.linalg.inv(U.dot(sigma).dot(V_star))

    TR = numpy.eye(3)
    TR[0:2, 0:2] = transform

    # Append 0 for z-axis
    center.append(0.0)
    offset = numpy.vstack(center)

    return offset.tolist(), TR.tolist()


def ellipse_center(A):
    """Returns ellipse's center, given ellipse parameters in A."""
    b,c,d,f,g,a = A[1]/2, A[2], A[3]/2, A[4]/2, A[5], A[0]
    num = b*b-a*c
    x0=(c*d-b*f)/num
    y0=(a*f-b*d)/num
    return [x0,y0]


def ellipse_angle_of_rotation(A):
    """Returns ellipse's angle of rotation, given ellipse parameters in A."""
    b,c,d,f,g,a = A[1]/2, A[2], A[3]/2, A[4]/2, A[5], A[0]
    if b == 0:
        if a > c:
            return 0
        else:
            return numpy.pi/2
    else:
        if a > c:
            return numpy.arctan(2*b/(a-c))/2
        else:
            return numpy.pi/2 + numpy.arctan(2*b/(a-c))/2


def ellipse_axis_length(A):
    """Returns ellipse axes lengths, given ellipse parameters in A."""
    b,c,d,f,g,a = A[1]/2, A[2], A[3]/2, A[4]/2, A[5], A[0]
    up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
    down1=(b*b-a*c)*((c-a)*numpy.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    down2=(b*b-a*c)*((a-c)*numpy.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
    res1=numpy.sqrt(up/down1)
    res2=numpy.sqrt(up/down2)
    return [res1, res2]


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


def compute_calibrated_data(x, y, z, offset, transform=None,
                            use_misalignment=True):
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

    # Dot product of transformation matrix, if given
    if transform is not None:
        transform = numpy.array(transform)
        T = M_m.dot(transform)
        v = T.dot(v - offset)
    else:
        v = v - offset

    return v.item(0), v.item(1), v.item(2)


def publish_diagnostic_msg():
    """
    Helper to imu_callback. Publishes a DiagnosticArray containing calibration
    information.
    """
    global mag_offsets, mag_transform, misalignment
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
    global calibrating, mag_data, gyro_data
    global mag_offsets, mag_transform
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
            gyro_status_msg = rover+': Collecting data from first gyro rotation.'
            gyro_data[0].append(imu_raw_msg.header.stamp.to_sec())
            gyro_data[1].append(imu_cal.angular_velocity.z)
        elif current_time - gyro_start_time < 20:
            gyro_status_msg = rover+': Collecting data from second gyro rotation.'
            gyro_data[2].append(imu_raw_msg.header.stamp.to_sec())
            gyro_data[3].append(imu_cal.angular_velocity.z)
        else:
            angle_1 = numpy.trapz(gyro_data[1], x=gyro_data[0])
            angle_2 = numpy.trapz(gyro_data[3], x=gyro_data[2])
            z_scale = (abs(math.pi/angle_1) + abs(math.pi/angle_2)) / 2
            gyro_scale[2][2] = z_scale
            msg = String(rover
                         +': Finished collecting gyro rotation data. Z-Scale: '
                         +str(z_scale))
            info_log.publish(msg)
            store_calibration(EmptyRequest())

    if calibrating == 'imu':
        mag_data[0].append(imu_raw_msg.magnetometer.x)
        mag_data[1].append(imu_raw_msg.magnetometer.y)
        mag_data[2].append(imu_raw_msg.magnetometer.z)

        if len(mag_data[0]) > DATA_SIZE_LIMIT:
            rospy.logwarn('IMU calibration timeout exceeded. Saving current calculated matrix.')
            store_calibration(EmptyRequest())

        # Don't fit until some data has been collected.
        # May still get a runtime warning until enough data in all directions
        # has been collected.
        elif len(mag_data[0]) > MIN_DATA_SIZE:
            (mag_offsets, mag_transform) = ellipse_fit(
                numpy.array(mag_data[0]),
                numpy.array(mag_data[1])
            )
            publish_diagnostic_msg()

    acc_x = imu_raw_msg.accelerometer.x
    acc_y = imu_raw_msg.accelerometer.y
    acc_z = imu_raw_msg.accelerometer.z

    imu_cal_data.accelerometer.x = acc_x
    imu_cal_data.accelerometer.y = acc_y
    imu_cal_data.accelerometer.z = acc_z

    # swap x and y to orient measurements in the rover's frame
    tmp = -acc_x
    acc_x = acc_y
    acc_y = tmp

    # Convert accelerometer digits to milligravities, then to gravities, and finally to meters per second squared
    acc_x *= 0.061 / 1000
    acc_y *= 0.061 / 1000
    acc_z *= 0.061 / 1000

    # Scale accelerations back to m/s**2.
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
            rospy.logwarn('Misalignment calibration timeout exceeded. Saving current calculated matrix.')
            store_calibration(EmptyRequest())

        # Wait until some data has been collected.
        elif len(mag_data[0]) > MIN_DATA_SIZE:
            data = numpy.array(mag_data)
            misalignment = calc_misalignment(data.T, misalignment)
            publish_diagnostic_msg()

    # swap x and y to orient measurements in the rover's frame
    # Done here so the misalignment calibration can use data in the original
    # IMU frame
    tmp = mag_x
    mag_x = mag_y
    mag_y = -tmp

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

    # Original roll, pitch, yaws as calculated on Arduino:
    # This yaw is very unstable during even small rolls or pitches
    # Xh = mag_y * math.cos(pitch) + mag_z * math.sin(pitch)
    # Yh = (mag_y * math.sin(roll) * math.sin(pitch)
    #       + mag_x * math.cos(roll)
    #       - mag_z * math.sin(roll) * math.cos(pitch))
    # yaw = math.atan(Yh / Xh)

    # roll = math.atan2(
    #     imu_cal.linear_acceleration.y,
    #     math.sqrt(imu_cal.linear_acceleration.x**2
    #               + imu_cal.linear_acceleration.z**2)
    # )
    # pitch = -math.atan2(
    #     imu_cal.linear_acceleration.x,
    #     math.sqrt(imu_cal.linear_acceleration.y**2
    #               + imu_cal.linear_acceleration.z**2)
    # )
    # yaw = math.pi + math.atan2(
    #     -mag_y*math.cos(roll) + mag_z*math.sin(roll),
    #     mag_x*math.cos(pitch)
    #     + mag_y*math.sin(pitch)*math.sin(roll)
    #     + mag_z*math.sin(pitch)*math.cos(roll)
    # )
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
    Reset magnetometer offset and transform matrices, and enter magnetometer
    calibration state.

    This calibration should be performed before the rover starts operating in a
    new environment.

    Raw magnetometer data is collected and fit to an ellipse. During this time,
    the rover should perform 2-3 full round rotations with its body axis z up.
    This can be done using the teleop.

    Calculated calibration matrices can be viewed on the
    /rover/imu/cal_diag topic while calibration is in progress.
    """
    global calibrating, mag_data
    global mag_offsets, mag_transform
    calibrating = 'imu'
    mag_data = [[], [], []]
    mag_offsets = [[0], [0], [0]]
    mag_transform = [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]
    return EmptyResponse()


def start_misalignment_calibration(req):
    """
    Reset misalignment matrix, and enter misalignment calibration state. This
    should be performed after the ellipse fit magnetometer calibration.

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
    global calibrating, rover, mag_data, gyro_data
    global mag_offsets, mag_transform
    global misalignment, gyro_bias, gyro_scale, gyro_timer
    FILE_PATH = rospy.get_param(
        '~calibration_file_path',
        default='/home/robot/'
    )
    calibrating = None
    mag_data = [[], [], []]
    gyro_data = [[], [], []]

    if gyro_timer is not None:
        gyro_timer.shutdown()
        gyro_timer = None

    cal = {
        'mag_offsets': mag_offsets,
        'mag_transform': mag_transform,
        'misalignment': misalignment,
        'gyro_bias': gyro_bias,
        'gyro_scale': gyro_scale
    }
    with open(FILE_PATH+rover+'_calibration_2d.json', 'w') as f:
        f.write(json.dumps(cal, sort_keys=True, indent=2))
    return EmptyResponse()


if __name__ == "__main__":
    global rover, calibrating, mag_data, gyro_data
    global mag_offsets, mag_transform
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
    FILE_PATH = rospy.get_param(
        '~calibration_file_path',
        default='/home/robot/'
    )
    # Data is stored in a list of lists, which is converted to a numpy array
    # when needed.
    mag_data = [[], [], []]
    gyro_data = [[], [], []]

    try:
        with open(FILE_PATH+rover+'_calibration_2d.json', 'r') as f:
            cal = json.loads(f.read())
        rospy.loginfo('IMU calibration file found at '+FILE_PATH+rover+'_calibration.json')
    except IOError as e:
        rospy.logwarn('No IMU calibration file found.')
    except ValueError as e:
        rospy.logwarn('Invalid IMU calibration file. Starting from scratch.')

    # Calibration matrices are stored as lists and converted to numpy arrays
    # when needed.
    if not cal:
        mag_offsets = [[0], [0], [0]]
        mag_transform = [[1, 0, 0],
                         [0, 1, 0],
                         [0, 0, 1]]
        misalignment = [[1., 0, 0],
                        [0, 1., 0],
                        [0, 0, 1.]]
        gyro_bias = [[0], [0], [0]]
        gyro_scale = [[1., 0, 0],
                      [0, 1., 0],
                      [0, 0, 1.]]
    else:
        mag_offsets = cal['mag_offsets']
        mag_transform = cal['mag_transform']
        misalignment = cal['misalignment']
        gyro_bias = cal['gyro_bias']
        gyro_scale = cal['gyro_scale']


    # Subscribers
    # imu_sub = message_filters.Subscriber(
    #     rover + '/imu/arduino',
    #     Imu
    # )
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

    # Publish current calibration once:
    publish_diagnostic_msg()

    rospy.spin()

