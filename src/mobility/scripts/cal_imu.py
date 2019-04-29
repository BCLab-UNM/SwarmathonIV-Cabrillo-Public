#! /usr/bin/env python
"""
IMU Node. Gets raw IMU data from ABridge and publishes calibrated IMU messages.

Can perform a 2D IMU Calibration as a fallback at the start of a round.

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

todo: pick validation thresholds that make sense. Something that means the 2D calibration is likely going to be better.
todo: ok for node to crash if extended cal file is missing or corrupt?
"""
from __future__ import print_function
import math
import numpy
import sys
import textwrap
import tf
import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from control_msgs.srv import QueryCalibrationState, QueryCalibrationStateResponse

from swarmie_msgs.msg import SwarmieIMU


class IMU:
    """Global State Variables"""

    STATE_IDLE = 0
    STATE_NORMAL = 1
    STATE_VALIDATE = 2
    STATE_CAL_GYRO_BIAS = 3
    STATE_CAL_GYRO_SCALE = 4
    STATE_CAL_MAG = 5
    STATE_CAL_MISALIGN = 6

    # Mode variables
    MODE_3D = 0
    MODE_2D = 1

    # In case someone forgets to exit either calibration state.
    DATA_SIZE_LIMIT = 3000  # 5 min worth of data at 10 Hz
    MIN_DATA_SIZE = 50

    # For extended file validation
    ROLL_PITCH_TOLERANCE = 3.0  # degrees
    MAG_VAR_TOLERANCE = 1e-3
    ACC_VAR_TOLERANCE = 4e-3

    REPLACEMENT_MSG = ' '.join(
        textwrap.wrap(
            textwrap.dedent(
                '''
                This fails the standard validation test and the rover should
                be replaced. However, it will continue to run if you choose
                not to replace it.
                '''
            )
        )
    )

    def __init__(self, rover):
        self.rover = rover
        rospy.init_node('imu')

        if rospy.has_param('~imu_is_failed'):
            rospy.logfatal(
                'The IMU node has previously encountered a fatal error. ' +
                'Exiting now.'
            )
            sys.exit(-1)

        if rospy.has_param('~imu_mode'):  # if respawning
            self._get_mode()
        else:
            self.current_mode = IMU.MODE_3D  # default to 3D mode
        self.current_state = IMU.STATE_IDLE  # idle until data file is loaded
        self.gyro_timer = None
        self.gyro_start_time = None
        self.gyro_status_msg = ''
        self.cal = {}

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # used during file validation
        self.rolls = []
        self.pitches = []

        # Default param values. Set to final values after validating
        self.finished_validating = False
        self.needs_calibration = False

        self.DEBUG = rospy.get_param(
            '~publish_debug_topic',
            default=False
        )
        self.LOAD_RAW_DATA = rospy.get_param(
            '~load_raw_data',
            default=False
        )
        self.RAW_DATA_PATH = rospy.get_param(
            '~raw_data_path',
            default='/home/swarmie/KSC_extended_calibration.csv'
        )

        # Raw data collected while in a calibration state is stored in a list
        # of lists, which is converted to a numpy array when needed.
        self.mag_data = [[], [], []]
        self.gyro_data = [[], [], []]

        # Default matrices
        self.acc_offsets = [[0], [0], [0]]
        self.acc_transform = [[1., 0, 0],
                              [0, 1., 0],
                              [0, 0, 1.]]
        self.mag_offsets = [[0], [0], [0]]
        self.mag_transform = [[1., 0, 0],
                              [0, 1., 0],
                              [0, 0, 1.]]
        self.misalignment = [[1., 0, 0],
                             [0, 1., 0],
                             [0, 0, 1.]]
        self.gyro_bias = [[0], [0], [0]]
        self.gyro_scale = [[1., 0, 0],
                           [0, 1., 0],
                           [0, 0, 1.]]

        # Subscribers
        self.imu_raw_sub = rospy.Subscriber(
            'imu/raw',
            SwarmieIMU,
            self.imu_callback,
            queue_size=10
        )

        # Publishers
        self.imu_pub = rospy.Publisher(
            'imu',
            Imu,
            queue_size=10
        )
        self.imu_diag_pub = rospy.Publisher(
            'cal_diag',
            DiagnosticArray,
            queue_size=10,
            latch=True
        )
        if self.DEBUG:
            self.imu_cal_data_pub = rospy.Publisher(
                'imu/raw/calibrated',
                SwarmieIMU,
                queue_size=10
            )
        self.info_log = rospy.Publisher(
            '/infoLog',
            String,
            queue_size=10
        )
        self.diags_log = rospy.Publisher(
            '/diagsLog',
            String,
            queue_size=10,
            latch=True
        )

        # Services
        self.start_imu_cal = rospy.Service(
            'start_imu_calibration',
            Empty,
            self.start_imu_calibration
        )
        self.store_cal = rospy.Service(
            'store_imu_calibration',
            Empty,
            self.store_calibration
        )
        self.start_misalign_cal = rospy.Service(
            'start_misalignment_calibration',
            Empty,
            self.start_misalignment_calibration
        )
        self.start_gyro_bias_cal = rospy.Service(
            'start_gyro_bias_calibration',
            Empty,
            self.start_gyro_bias_calibration
        )
        self.start_gyro_scale_cal = rospy.Service(
            'start_gyro_scale_calibration',
            Empty,
            self.start_gyro_scale_calibration
        )
        self._is_finished_val = rospy.Service(
            'imu/is_finished_validating',
            QueryCalibrationState,
            self._is_finished_validating
        )
        self._needs_cal = rospy.Service(
            'imu/needs_calibration',
            QueryCalibrationState,
            self._needs_calibration
        )

        # Try waiting for subscriber on /diagsLog. Helps to make sure first
        # message or two actually make it onto the rqt gui.
        rate = rospy.Rate(10)
        for i in range(20):
            if self.diags_log.get_num_connections() > 0:
                break
            rate.sleep()

        # If node is respawning for some reason
        if rospy.has_param('~imu_calibration_matrices'):
            self.cal = rospy.get_param('~imu_calibration_matrices')
            self._get_mode()

            self.acc_offsets = self.cal['acc_offsets']
            self.acc_transform = self.cal['acc_transform']
            self.mag_offsets = self.cal['mag_offsets']
            self.mag_transform = self.cal['mag_transform']
            self.misalignment = self.cal['misalignment']
            self.gyro_bias = self.cal['gyro_bias']
            self.gyro_scale = self.cal['gyro_scale']

            self.current_state = IMU.STATE_NORMAL
            self.finished_validating = True
            self.needs_calibration = False
            msg = 'Reloaded calibration matrices after respawn.'
            if self.current_mode == IMU.MODE_2D:
                msg += ' Using 2D mode.'
            elif self.current_mode == IMU.MODE_3D:
                msg += ' Using 3D mode.'

            self._diag_info(msg)

        elif self.LOAD_RAW_DATA:
            self.load_and_validate_calibration()

        # Publish current calibration once:
        self.publish_diagnostic_msg()

    def _diag_info(self, msg):
        """Log an info message and publish it on /diagsLog."""
        m = '{}: {}'.format(self.rover, msg)
        rospy.loginfo(m)
        self._diag_pub(m, 'White')

    def _diag_warn(self, msg):
        """Log a warning message and publish it on /diagsLog."""
        m = '{}: {}'.format(self.rover, msg)
        rospy.logwarn(m)
        self._diag_pub(m, 'Yellow')

    def _diag_fatal(self, msg):
        """Log a fatal error message and publish it on /diagsLog."""
        m = '{}: {}'.format(self.rover, msg)
        rospy.logfatal(m)
        self._diag_pub(m, 'Red')

    def _diag_pub(self, msg, color):
        m = '<font color="{}" size="2">{}</font>'.format(color, msg)
        self.diags_log.publish(m)

    def _set_mode(self, mode):
        """Sets the IMU mode to mode and puts it onto the parameter server.
        Useful if node respawns, so it knows which mode (2D/3D) it was in."""
        self.current_mode = mode
        rospy.set_param('~imu_mode', mode)

    def _get_mode(self):
        """Gets the IMU mode from the parameter server. Useful if node
        respawns, so it knows which mode (2D/3D) it was in."""
        self.current_mode = rospy.get_param('~imu_mode', default=IMU.MODE_3D)

    def _is_finished_validating(self, req):
        """Service to allow Swarmie API to wait until extended calibration file
        has been loaded and validated."""
        response = QueryCalibrationStateResponse()
        response.is_calibrated = self.finished_validating
        return response

    def _needs_calibration(self, req):
        """Service to allow Swarmie API to ask if the IMU needs to be
        calibrated using the 2D fallback."""
        response = QueryCalibrationStateResponse()
        response.is_calibrated = self.needs_calibration
        return response

    def load_and_validate_calibration(self):
        """Load the extended calibration file.

        Raises:
        * IOError if calibration file can't be found.
        * ValueError if calibration file is corrupt.
        """
        try:
            data = numpy.loadtxt(self.RAW_DATA_PATH, delimiter=',')
            mag_x = data[:,0]
            mag_y = data[:,1]
            mag_z = data[:,2]
            acc_x = data[:,3]
            acc_y = data[:,4]
            acc_z = data[:,5]

            self.cal['mag_offsets'], self.cal['mag_transform'] = \
                self.ellipsoid_fit(mag_x, mag_y, mag_z)
            self.cal['acc_offsets'], self.cal['acc_transform'] = \
                self.ellipsoid_fit(acc_x, acc_y, acc_z)
            self.cal['misalignment'] = [[1., 0, 0],
                                        [0, 1., 0],
                                        [0, 0, 1.]]
            self.cal['gyro_bias'] = [[0], [0], [0]]
            self.cal['gyro_scale'] = [[1., 0, 0],
                                      [0, 1., 0],
                                      [0, 0, 1.]]

            rospy.loginfo(
                self.rover + ': IMU raw data file loaded from ' +
                self.RAW_DATA_PATH
            )
        except IOError:
            self._diag_fatal(
                ('FATAL ERROR. Extended calibration file {} ' +
                 'not found. This rover must be replaced.').format(
                    self.RAW_DATA_PATH
                )
            )
            rospy.set_param('~imu_is_failed', True)
            # Wait in hope the message gets published and received by any
            # subscribers before the node exits.
            rospy.sleep(3.0)
            raise
        except ValueError as e:
            self._diag_fatal(
                ('FATAL ERROR. Error reading extended calibration ' +
                 'file {}. The file is corrupt: {}. This rover must ' +
                 'be replaced.').format(self.RAW_DATA_PATH, e.message)
            )
            rospy.set_param('~imu_is_failed', True)
            # Wait in hope the message gets published and received by any
            # subscribers before the node exits.
            rospy.sleep(3.0)
            raise

        # Calibration matrices are stored as lists and converted to numpy
        # arrays when needed.
        self.acc_offsets = self.cal['acc_offsets']
        self.acc_transform = self.cal['acc_transform']
        self.mag_offsets = self.cal['mag_offsets']
        self.mag_transform = self.cal['mag_transform']
        self.misalignment = self.cal['misalignment']
        self.gyro_bias = self.cal['gyro_bias']
        self.gyro_scale = self.cal['gyro_scale']

        # Check variance in errors
        mag_var_err = self.error(mag_x, mag_y, mag_z,
                                 self.mag_offsets, self.mag_transform)
        acc_var_err = self.error(acc_x, acc_y, acc_z,
                                 self.acc_offsets, self.acc_transform)

        self._diag_info(('IMU calibration validation: Magnetometer ' +
                         'v[Err]: {:7.6f}').format(mag_var_err))
        self._diag_info(('IMU calibration validation: Accelerometer ' +
                         'v[Err]: {:7.6f}').format(acc_var_err))

        if (math.isnan(mag_var_err) or
                abs(mag_var_err) >= IMU.MAG_VAR_TOLERANCE):
            self._diag_warn(
                ("The magnetometer fit is too poor to use. The data's " +
                 "variance ({:7.6f}) exceeds the threshold of {}. {}").format(
                    mag_var_err,
                    IMU.MAG_VAR_TOLERANCE,
                    IMU.REPLACEMENT_MSG
                )
            )
            self.needs_calibration = True
            self._set_mode(IMU.MODE_2D)

        if (math.isnan(acc_var_err) or
                abs(acc_var_err) >= IMU.ACC_VAR_TOLERANCE):
            self._diag_warn(
                ("The accelerometer fit is too poor to use. The data's " +
                 "variance ({:7.6f}) exceeds the threshold of {}. {}").format(
                    acc_var_err,
                    IMU.ACC_VAR_TOLERANCE,
                    IMU.REPLACEMENT_MSG
                )
            )
            self.needs_calibration = True
            self._set_mode(IMU.MODE_2D)

        # Check roll and pitch
        self.current_state = IMU.STATE_VALIDATE
        try:
            rospy.wait_for_message(
                'imu/raw',
                SwarmieIMU,
                timeout=5
            )
        except rospy.ROSException:
            # hopefully this doesn't happen
            pass

        # wait for 2 seconds for messages to come in and populate
        # self.rolls and self.pitches
        rospy.sleep(2)

        avg_roll = numpy.average(self.rolls) * 180 / math.pi
        avg_pitch = numpy.average(self.pitches) * 180 / math.pi

        self._diag_info(('IMU calibration validation: ' +
                         'Average roll: {:6.3f} deg').format(avg_roll))
        self._diag_info(('IMU calibration validation: ' +
                         'Average pitch: {:6.3f} deg').format(avg_pitch))

        if abs(avg_roll) > IMU.ROLL_PITCH_TOLERANCE:
            self._diag_warn(
                ('The avg roll exceeds the tolerance threshold of ' +
                 '{:.1f} deg. {}').format(
                    IMU.ROLL_PITCH_TOLERANCE,
                    IMU.REPLACEMENT_MSG
                )
            )
            self.needs_calibration = True
            self._set_mode(IMU.MODE_2D)

        if abs(avg_pitch) > IMU.ROLL_PITCH_TOLERANCE:
            self._diag_warn(
                ('The avg pitch exceeds the tolerance threshold of ' +
                 '{:.1f} deg. {}').format(
                    IMU.ROLL_PITCH_TOLERANCE,
                    IMU.REPLACEMENT_MSG
                )
            )
            self.needs_calibration = True
            self._set_mode(IMU.MODE_2D)

        self.finished_validating = True
        self.store_calibration(EmptyRequest())
        self.current_state = IMU.STATE_NORMAL

    def error(self, x, y, z, offsets, transform):
        """Compute the variance of errors of data in numpy arrays x,
        y, z. Errors are the distances of the calibrated points from the
        surface of the unit sphere.
        """
        v = numpy.array([x, y, z])
        offsets = numpy.array(offsets)
        transform = numpy.array(transform)
        v = transform.dot(v - offsets)
        var_err = numpy.var(numpy.sqrt(numpy.sum(numpy.square(v), 0)) - 1)
        return var_err

    def ellipsoid_fit(self, x, y, z):
        """Fit the data points contained in numpy arrays x, y and z to a unit
        sphere centered at the origin.

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

    def ellipse_fit(self, x, y):
        """Fits the data points in x and y to a circle centered at the x-y
        origin.
        http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html

        Returns 3R x 1C offset matrix and a 3x3 transformation matrix. Only the
        first 2 rows and columns are calculated in the transformation matrix,
        since this is only a 2-D calibration.
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

        center = self.ellipse_center(A)
        beta = self.ellipse_angle_of_rotation(A)
        major, minor = self.ellipse_axis_length(A)

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

        # Append current z-mag_offset value for z-axis
        center.append(self.mag_offsets[2][0])
        offset = numpy.vstack(center)

        return offset.tolist(), TR.tolist()

    def ellipse_center(self, A):
        """Returns ellipse's center coordinates, given ellipse parameters in
        A."""
        b,c,d,f,g,a = A[1]/2, A[2], A[3]/2, A[4]/2, A[5], A[0]

        num = b*b-a*c
        x0=(c*d-b*f)/num
        y0=(a*f-b*d)/num

        return [x0,y0]

    def ellipse_angle_of_rotation(self, A):
        """Returns ellipse's angle of rotation, given ellipse parameters in
        A."""
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

    def ellipse_axis_length(self, A):
        """Returns ellipse axes lengths, given ellipse parameters in A."""
        b,c,d,f,g,a = A[1]/2, A[2], A[3]/2, A[4]/2, A[5], A[0]

        up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
        down1=(b*b-a*c)*((c-a)*numpy.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        down2=(b*b-a*c)*((a-c)*numpy.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        res1=numpy.sqrt(up/down1)
        res2=numpy.sqrt(up/down2)

        return [res1, res2]

    def calc_misalignment(self, H, current_misalign):
        """Misalignment calibration.
        From: https://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf

        We will only perform calibration for the rotation around the z-axis.

        Calculates compensation to align the IMU sensor axis to the rover's
        body axis using numpy array, H, the data from a 2D rotation around one
        axis.
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

    def compute_calibrated_data(self, x, y, z, offset, transform,
                                use_misalignment=True):
        """Map the raw x, y, z accelerometer or magnetometer vector onto the
        calibrated unit sphere. Skips misalignment transformation if we are in
        the misalignment calibration state.
        """
        v = numpy.array([[x],
                         [y],
                         [z]])
        offset = numpy.array(offset)

        # Misalignment calibration needs to get only hard-iron and soft-iron
        # calibrated data.
        if self.current_state == IMU.STATE_CAL_MISALIGN or use_misalignment is False:
            M_m = numpy.eye(3)
        else:
            M_m = numpy.array(self.misalignment)

        transform = numpy.array(transform)
        T = M_m.dot(transform)
        v = T.dot(v - offset)

        return v.item(0), v.item(1), v.item(2)

    def publish_diagnostic_msg(self):
        """Helper to imu_callback. Publishes a DiagnosticArray containing
        calibration information.
        """
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = rospy.Time.now()
        diag_msg.status = [
            DiagnosticStatus(
                level = DiagnosticStatus.OK,
                name = 'IMU Calibration Info',
                values = [
                    KeyValue(key='Gyro Bias',
                             value=str(self.gyro_bias)),
                    KeyValue(key='Gyro Scale',
                             value=str(self.gyro_scale)),
                    KeyValue(key='Accel Offsets',
                             value=str(self.acc_offsets)),
                    KeyValue(key='Accel Transform',
                             value=str(self.acc_transform)),
                    KeyValue(key='Mag Offsets',
                             value=str(self.mag_offsets)),
                    KeyValue(key='Mag Transform',
                             value=str(self.mag_transform)),
                    KeyValue(key='Misalignment',
                             value=str(self.misalignment))
                ]
            )
        ]
        self.imu_diag_pub.publish(diag_msg)
        return

    def imu_callback(self, imu_raw):
        """Callback for the SwarmieIMU message containing raw accelerometer and
        magnetometer data.

        Calibrates IMU by fitting an ellipsoid, or calculating the misalignment
        matrix if we are in either of those states.

        Computes calibrated accelerometer and magnetometer data, transformed
        from the IMU's frame into the rover's frame, calculates roll, pitch,
        yaw, and publishes a calibrated IMU message.
        """
        if self.current_state == IMU.STATE_IDLE:
            return

        if self.DEBUG:
            # Message for raw, calibrated data. Published only when debugging
            imu_raw_cal = SwarmieIMU()
            imu_raw_cal.header = imu_raw.header
            imu_raw_cal.angular_velocity = imu_raw.angular_velocity

        # IMU Message
        imu_cal = Imu()
        imu_cal.header = imu_raw.header
        imu_cal.orientation_covariance[8] = 0.00004
        imu_cal.angular_velocity_covariance[8] = 0.00001

        if self.current_state == IMU.STATE_CAL_GYRO_BIAS:
            self.gyro_data[0].append(imu_raw.angular_velocity.x)
            self.gyro_data[1].append(imu_raw.angular_velocity.y)
            self.gyro_data[2].append(imu_raw.angular_velocity.z)
            self.gyro_bias[0][0] = float(numpy.mean(self.gyro_data[0]))
            self.gyro_bias[1][0] = float(numpy.mean(self.gyro_data[1]))
            self.gyro_bias[2][0] = float(numpy.mean(self.gyro_data[2]))

        (gyro_x, gyro_y, gyro_z) = self.compute_calibrated_data(
            imu_raw.angular_velocity.x,
            imu_raw.angular_velocity.y,
            imu_raw.angular_velocity.z,
            self.gyro_bias,
            self.gyro_scale,
            use_misalignment=False
        )
        if self.DEBUG:
            imu_raw_cal.angular_velocity.x = gyro_x
            imu_raw_cal.angular_velocity.y = gyro_y
            imu_raw_cal.angular_velocity.z = gyro_z

        # Convert gyroscope digits to millidegrees per second, then to degrees
        # per second, and finally to radians per second. axes mismatched as in
        # original arduino code to match rover's coord frame
        imu_cal.angular_velocity.x = gyro_y * 8.75 / 1000 * (math.pi / 180)
        imu_cal.angular_velocity.y = -gyro_x * 8.75 / 1000 * (math.pi / 180)
        imu_cal.angular_velocity.z = gyro_z * 8.75 / 1000 * (math.pi / 180)

        if self.current_state == IMU.STATE_CAL_GYRO_SCALE:
            current_time = rospy.Time.now().to_sec()
            if current_time - self.gyro_start_time < 10:
                self.gyro_status_msg = (
                    self.rover + ': Collecting data from first gyro rotation.'
                )
                self.gyro_data[0].append(imu_raw.header.stamp.to_sec())
                self.gyro_data[1].append(imu_cal.angular_velocity.z)
            elif current_time - self.gyro_start_time < 20:
                self.gyro_status_msg = (
                    self.rover + ': Collecting data from second gyro rotation.'
                )
                self.gyro_data[2].append(imu_raw.header.stamp.to_sec())
                self.gyro_data[3].append(imu_cal.angular_velocity.z)
            else:
                angle_1 = numpy.trapz(self.gyro_data[1], x=self.gyro_data[0])
                angle_2 = numpy.trapz(self.gyro_data[3], x=self.gyro_data[2])
                z_scale = (abs(math.pi/angle_1) + abs(math.pi/angle_2)) / 2
                self.gyro_scale[2][2] = z_scale
                msg = (
                    self.rover + ': Finished collecting gyro rotation data. ' +
                    'Z-Scale: ' + str(z_scale)
                )
                self.info_log.publish(msg)
                self.store_calibration(EmptyRequest())

        if self.current_state == IMU.STATE_CAL_MAG:
            self.mag_data[0].append(imu_raw.magnetometer.x)
            self.mag_data[1].append(imu_raw.magnetometer.y)
            self.mag_data[2].append(imu_raw.magnetometer.z)

            if len(self.mag_data[0]) > IMU.DATA_SIZE_LIMIT:
                rospy.logwarn(
                    'IMU calibration timeout exceeded. ' +
                    'Saving current calculated matrix.'
                )
                self.store_calibration(EmptyRequest())

            # Don't fit until some data has been collected. May still get a
            # runtime warning until enough data in all directions has been
            # collected.
            elif len(self.mag_data[0]) > IMU.MIN_DATA_SIZE:
                self.mag_offsets, self.mag_transform = self.ellipse_fit(
                    numpy.array(self.mag_data[0]),
                    numpy.array(self.mag_data[1])
                )
                self.publish_diagnostic_msg()

        if self.current_mode == IMU.MODE_3D:
            (acc_x, acc_y, acc_z) = self.compute_calibrated_data(
                imu_raw.accelerometer.x,
                imu_raw.accelerometer.y,
                imu_raw.accelerometer.z,
                self.acc_offsets,
                self.acc_transform
            )
        elif self.current_mode == IMU.MODE_2D:
            # Convert accelerometer digits to milligravities, then to
            # gravities
            acc_x = imu_raw.accelerometer.x * 0.061 / 1000
            acc_y = imu_raw.accelerometer.y * 0.061 / 1000
            acc_z = imu_raw.accelerometer.z * 0.061 / 1000

        if self.DEBUG:
            imu_raw_cal.accelerometer.x = acc_x
            imu_raw_cal.accelerometer.y = acc_y
            imu_raw_cal.accelerometer.z = acc_z

        # swap x and y to orient measurements in the rover's frame
        tmp = -acc_x
        acc_x = acc_y
        acc_y = tmp

        # Scale accelerations back to m/s**2.
        imu_cal.linear_acceleration.x = acc_x * 9.81
        imu_cal.linear_acceleration.y = acc_y * 9.81
        imu_cal.linear_acceleration.z = acc_z * 9.81

        (mag_x, mag_y, mag_z) = self.compute_calibrated_data(
            imu_raw.magnetometer.x,
            imu_raw.magnetometer.y,
            imu_raw.magnetometer.z,
            self.mag_offsets,
            self.mag_transform
        )
        if self.DEBUG:
            imu_raw_cal.magnetometer.x = mag_x
            imu_raw_cal.magnetometer.y = mag_y
            imu_raw_cal.magnetometer.z = mag_z

        if self.current_state == IMU.STATE_CAL_MISALIGN:
            self.mag_data[0].append(mag_x)
            self.mag_data[1].append(mag_y)
            self.mag_data[2].append(mag_z)

            if len(self.mag_data[0]) > IMU.DATA_SIZE_LIMIT:
                rospy.logwarn(
                    'Misalignment calibration timeout exceeded. ' +
                    'Saving current calculated matrix.'
                )
                self.store_calibration(EmptyRequest())

            # Wait until some data has been collected.
            elif len(self.mag_data[0]) > IMU.MIN_DATA_SIZE:
                data = numpy.array(self.mag_data)
                self.misalignment = self.calc_misalignment(data.T, self.misalignment)
                self.publish_diagnostic_msg()

        # swap x and y to orient measurements in the rover's frame Done here so
        # the misalignment calibration can use data in the original IMU frame
        tmp = -mag_x
        mag_x = mag_y
        mag_y = tmp

        # From: Computing tilt measurement and tilt-compensated e-compass
        # www.st.com/resource/en/design_tip/dm00269987.pdf
        self.roll = math.atan2(acc_y, acc_z)
        Gz2 = (acc_y * math.sin(self.roll) + acc_z * math.cos(self.roll))

        # Gimbal-lock. Special case when pitched + or - 90 deg.
        # Heading is unreliable here, but plenty of other bad things will be
        # happening if the rover is ever in this position.
        if abs(Gz2) < 0.01:
            if acc_x > 0:
                rospy.loginfo('Special compass case: pitch is -90 deg')
                self.pitch = -math.pi / 2
            else:
                rospy.loginfo('Special compass case: pitch is +90 deg')
                self.pitch = math.pi / 2
            alpha = .01  # Can be set from [0.01 - 0.05]
            self.roll = math.atan2(acc_y, acc_z + acc_x * alpha)
        else:
            self.pitch = math.atan(-acc_x / Gz2)

        By2 = mag_z * math.sin(self.roll) - mag_y * math.cos(self.roll)
        Bz2 = mag_y * math.sin(self.roll) + mag_z * math.cos(self.roll)
        Bx3 = mag_x * math.cos(self.pitch) + Bz2 * math.sin(self.pitch)
        self.yaw = math.pi / 2 + math.atan2(By2, Bx3)

        imu_cal.orientation = Quaternion(
            *tf.transformations.quaternion_from_euler(
                self.roll,
                self.pitch,
                self.yaw
            )
        )
        if self.current_state == IMU.STATE_VALIDATE:
            self.rolls.append(self.roll)
            self.pitches.append(self.pitch)

        if self.DEBUG:
            self.imu_cal_data_pub.publish(imu_raw_cal)
        self.imu_pub.publish(imu_cal)

        return

    def start_imu_calibration(self, req):
        """Reset accelerometer and magnetometer offset and transform matrices,
        and enter magnetometer calibration state. Accelerometer matrices are
        reset here in case they're still invalid from a bad extended
        calibration file.

        Raw magnetometer data is collected and fit to an ellipse. During this
        time, the rover should perform 2-3 full round rotations with its body
        axis z up. This can be done using the teleop or can be programmed in
        init.py for the start of a round.

        Calculated calibration matrices can be viewed on the
        /rover/imu/cal_diag topic while calibration is in progress.
        """
        self.current_state = IMU.STATE_CAL_MAG
        self.mag_data = [[], [], []]

        # reset acc matrices, although they aren't used in 2D mode
        self.acc_offsets = [[0], [0], [0]]
        self.acc_transform = [[1., 0, 0],
                              [0, 1., 0],
                              [0, 0, 1.]]

        # leave z-offset alone, hopefully it'll still be ok from data file
        self.mag_offsets[0][0] = 0
        self.mag_offsets[1][0] = 0
        # self.mag_offsets = [[0], [0], [0]]
        self.mag_transform = [[1., 0, 0],
                              [0, 1., 0],
                              [0, 0, 1.]]
        return EmptyResponse()


    def start_misalignment_calibration(self, req):
        """Reset misalignment matrix, and enter misalignment calibration state.
        This should be performed after the ellipsoid/ellipse fit IMU
        calibration.

        I think this calibration should only need to be performed once for a
        given rover.

        Raw magnetometer data is collected while the rover performs at least
        one slow 2D rotation with its z axis up. This can be performed by
        having the rover spin slowly in place on level ground. This is only one
        third of a typical misalignment calibration procedure, but since the
        rover only operates in two dimensions on relatively level ground, it
        should be ok to skip the x-down and y-down rotations.
        """
        self.current_state = IMU.STATE_CAL_MISALIGN
        self.mag_data = [[], [], []]
        self.misalignment = [[1., 0, 0],
                             [0, 1., 0],
                             [0, 0, 1.]]
        return EmptyResponse()

    def start_gyro_bias_calibration(self, req):
        """Start gyro bias calibration. Rover should remain static for this."""
        self.current_state = IMU.STATE_CAL_GYRO_BIAS
        self.gyro_data = [[], [], []]
        self.gyro_bias = [[0], [0], [0]]
        return EmptyResponse()

    def start_gyro_scale_calibration(self, req):
        """Start gyro scale calibration. Rover should rotate 180 degrees in one
        direction during first 10 seconds, and 180 degress in opposite
        direction during second 10 seconds.
        """
        self.current_state = IMU.STATE_CAL_GYRO_SCALE
        self.gyro_data = [[], [], [], []]
        self.gyro_scale = [[1., 0, 0],
                           [0, 1., 0],
                           [0, 0, 1.]]

        # Timer to throttle gyro status messages to 1 Hz
        self.gyro_timer = rospy.Timer(rospy.Duration(1), self.log_gyro_status)
        self.gyro_start_time = rospy.Time.now().to_sec()
        return EmptyResponse()

    def log_gyro_status(self, event):
        """Helper to log gyro scale calibration status messages to /infoLog.
        Used as the callback to a Timer initialized in
        start_gyro_scale_calibration()
        """
        msg = String(self.gyro_status_msg)
        self.info_log.publish(msg)

    def store_calibration(self, req):
        """Stores all current calibration matrices onto the parameter server
        and resets dataset lists."""
        self.current_state = IMU.STATE_NORMAL
        self.mag_data = [[], [], []]
        self.gyro_data = [[], [], []]

        if self.gyro_timer is not None:
            self.gyro_timer.shutdown()
            self.gyro_timer = None

        self.cal['acc_offsets'] = self.acc_offsets
        self.cal['acc_transform'] = self.acc_transform
        self.cal['mag_offsets'] = self.mag_offsets
        self.cal['mag_transform'] = self.mag_transform
        self.cal['misalignment'] = self.misalignment
        self.cal['gyro_bias'] = self.gyro_bias
        self.cal['gyro_scale'] = self.gyro_scale

        rospy.set_param(
            '~imu_calibration_matrices',
            self.cal
        )

        self.publish_diagnostic_msg()

        return EmptyResponse()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rover = sys.argv[1]
    imu = IMU(rover)

    rospy.spin()
