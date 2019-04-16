'''
Common rover interface in Python
''' 

from __future__ import print_function 

import sys 
import rospy
import math 
import random 
import angles 
import tf
import dynamic_reconfigure.client

from mobility.srv import Core
from mapping.srv import GetNavPlan, GetNavPlanRequest
from mobility.msg import MoveResult, MoveRequest
from swarmie_msgs.msg import Obstacle 

from std_srvs.srv import Empty 
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry
from control_msgs.srv import QueryCalibrationState, QueryCalibrationStateRequest
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Twist, Pose2D
from apriltags2to1.msg import AprilTagDetection, AprilTagDetectionArray

import threading 

swarmie_lock = threading.Lock()

from .utils import block_detection, block_pose, filter_detections, is_moving
from mobility import sync

class DriveException(Exception):
    def __init__(self, st):
        self.status = st
    
class VisionException(DriveException):
    pass

class TagException(VisionException):
    pass

class HomeException(VisionException):
    pass

class AprilTagBoundaryException(VisionException):
    pass 

class InsideHomeException(VisionException):
    pass

class HomeCornerException(VisionException):
    pass

class ObstacleException(DriveException):
    pass 

class PathException(DriveException):
    pass

class AbortException(DriveException):
    pass

class TimeoutException(DriveException):
    pass

class Location: 
    '''A class that encodes an EKF provided location and accessor methods''' 

    def __init__(self, odo):
        self.Odometry = odo 
    
    def get_pose(self):
        '''Return a std_msgs.Pose from this Location. Useful because Pose 
        has angles represented as roll, pitch, yaw.
        
        Returns:
        
        * (`std_msgs.msg.Pose`) The pose. 
        '''
        quat = [self.Odometry.pose.pose.orientation.x, 
                self.Odometry.pose.pose.orientation.y, 
                self.Odometry.pose.pose.orientation.z, 
                self.Odometry.pose.pose.orientation.w, 
                ]        
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)
        pose = Pose2D()
        pose.x = self.Odometry.pose.pose.position.x 
        pose.y = self.Odometry.pose.pose.position.y 
        pose.theta = y 
        return pose

    def at_goal(self, goal, distance):
        '''Determine if the pose is within acceptable distance of this location
        
        Returns:
        
        * (`bool`) True if within the target distance.

        ''' 
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x, 
                          goal.y - self.Odometry.pose.pose.position.y)

        return dist < distance

    def get_variances(self):
        '''Return the X, Y and Z variances from the covariance matrix of this location. 
        Variance is defined as the expectation of the squared deviation of a random 
        variable from its mean. 
        
        https://en.wikipedia.org/wiki/Variance
        
        Returns: 
        
        * (`tuple`): `(vX, vY, vZ)`
        '''
        return (self.Odometry.pose.covariance[0:15:7])

class Swarmie(object):
    '''Class that embodies the Swarmie's API
    
    This is the Python API used to drive the rover. The API interfaces 
    with ROS topics and services to perform action and acquire sensor data. 
    '''
    def __init__(self):
        '''Constructor.
        '''

        self.rover_name = None
        self.Obstacles = 0
        self.OdomLocation = Location(None)
        self._home_odom_position = None  # type: PointStamped
        self._home_odom_approx_position = None  # type: PointStamped
        self.sm_publisher = None
        self.status_publisher = None
        self.info_publisher = None
        self.mode_publisher = None
        self.finger_publisher = None
        self.wrist_publisher = None

        self.block_size = None

        self._param_client = None
        self._drive_speeds = {
            'slow':   (None, None),
            'normal': (None, None),
            'fast':   (None, None)
        }

        self.control = None
        self._get_plan = None
        self._imu_is_finished_validating = None
        self._imu_needs_calibration =  None
        self._start_imu_calibration = None
        self._start_misalignment_calibration = None
        self._start_gyro_bias_calibration = None
        self._start_gyro_scale_calibration = None
        self._store_imu_calibration = None

        self.xform = None


    def start(self, **kwargs):
        """
        Start rover services. This initializes the ROS node.

        kwargs:

            tf_rover_name: (str) The name of the rover used for transforms. If
                not specified the result of rospy.get_namespace() will determine
                the value.

            node_name: (str) The name of the ROS node to be given to rospy.init().
                If not specified the node will be called "controller".

            anonymous: (bool) Tell ROS to create an anonymous node. You can read about
                anonymous nodes here: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown#Initializing_your_ROS_Node
        """

        if 'tf_rover_name' in kwargs :
            self.rover_name = kwargs['tf_rover_name']
        else:
            self.rover_name = rospy.get_namespace()
            
        self.rover_name = self.rover_name.strip('/')
           
        self.Obstacles = 0
        self.OdomLocation = Location(None)
        self.CIRCULAR_BUFFER_SIZE = 90
        self.targets = [[]]*self.CIRCULAR_BUFFER_SIZE  # The rolling buffer of targets msgs was AprilTagDetectionArray()
        self.targets_index = 0  # Used to keep track of the most recent targets index, holds the values 0-89
        
        # Intialize this ROS node.
        anon = False
        if 'anonymous' in kwargs : 
            anon = kwargs['anonymous']

        node_name = 'controller'
        if 'node_name' in kwargs :
            node_name = kwargs['node_name']

        rospy.init_node(node_name, anonymous=anon)

        self.block_size = rospy.get_param('block_size', 0.05)

        # Setup dynamic reconfigure client to get notifications of drive speed
        # changes on the server.
        self._param_client = dynamic_reconfigure.client.Client(
            'mobility',
            config_callback=self._update_params
        )
        config = self._param_client.get_configuration()
        self._update_params(config)
        
        # Create publishiers.
        self.sm_publisher = rospy.Publisher('state_machine', String, queue_size=10, latch=True)
        self.status_publisher = rospy.Publisher('status', String, queue_size=10, latch=True)
        self.info_publisher = rospy.Publisher('/infoLog', String, queue_size=10, latch=True)
        self.mode_publisher = rospy.Publisher('mode', UInt8, queue_size=1, latch=True)
        self.finger_publisher = rospy.Publisher('fingerAngle/cmd', Float32, queue_size=1, latch=True)
        self.wrist_publisher = rospy.Publisher('wristAngle/cmd', Float32, queue_size=1, latch=True)

        # Wait for necessary services to be online. 
        # Services are APIs calls to other neodes. 
        rospy.wait_for_service('control')
        rospy.wait_for_service('map/get_plan')

        # Connect to services.
        self.control = rospy.ServiceProxy('control', Core)
        self._get_plan = rospy.ServiceProxy('map/get_plan', GetNavPlan)
        self._imu_is_finished_validating = rospy.ServiceProxy('imu/is_finished_validating', QueryCalibrationState)
        self._imu_needs_calibration = rospy.ServiceProxy('imu/needs_calibration', QueryCalibrationState)
        self._start_imu_calibration = rospy.ServiceProxy('start_imu_calibration', Empty)
        self._start_misalignment_calibration = rospy.ServiceProxy('start_misalignment_calibration', Empty)
        self._start_gyro_bias_calibration = rospy.ServiceProxy('start_gyro_bias_calibration', Empty)
        self._start_gyro_scale_calibration = rospy.ServiceProxy('start_gyro_scale_calibration', Empty)
        self._store_imu_calibration = rospy.ServiceProxy('store_imu_calibration', Empty)

        # Transform listener. Use this to transform between coordinate spaces.
        # Transform messages must predate any sensor messages so initialize this first.
        self.xform = tf.TransformListener()

        # Subscribe to useful topics 
        # These topics only update data. The data is used in other places to initialize
        # these and wait for valid data so that users of this data don't get errors.
        rospy.Subscriber('odom/filtered', Odometry, self._odom)
        rospy.Subscriber('obstacle', Obstacle, self._obstacle)
        rospy.Subscriber('home_point', PointStamped, self._home_point)
        rospy.Subscriber('home_point/approx', PointStamped, self._home_point,
                         callback_args=True)

        # Wait for Odometry messages to come in.
        # Don't wait for messages on /obstacle because it's published infrequently
        try:
            rospy.wait_for_message('odom/filtered', Odometry, 2)
        except rospy.ROSException:
            rospy.logwarn(self.rover_name +
                          ': timed out waiting for filtered odometry data.')

        # The targets subscriber needs odom data since Carter's patch. Make sure odom data
        # exists before we get a target callback.
        rospy.Subscriber('targets', AprilTagDetectionArray, self._targets)
        try:
            rospy.wait_for_message('targets', AprilTagDetectionArray, 2)
        except rospy.ROSException:
            rospy.logwarn(self.rover_name +
                          ': timed out waiting for /targets data.')

        print ('Welcome', self.rover_name, 'to the world of the future.')

    def _update_params(self, config):
        """Update drive parameters with the new configuration."""
        self._drive_speeds['slow'] = (config['DRIVE_SPEED_SLOW'],
                                      config['TURN_SPEED_SLOW'])
        self._drive_speeds['normal'] = (config['DRIVE_SPEED'],
                                        config['TURN_SPEED'])
        self._drive_speeds['fast'] = (config['DRIVE_SPEED_FAST'],
                                      config['TURN_SPEED_FAST'])

    @sync(swarmie_lock)
    def _odom(self, msg) : 
        self.OdomLocation.Odometry = msg
            
    @sync(swarmie_lock)
    def _obstacle(self, msg) :
        self.Obstacles &= ~msg.mask 
        self.Obstacles |= msg.msg

    @sync(swarmie_lock)
    def _home_point(self, msg, approx=False):
        """Update the home plate's position in the odometry frame.

        Args:
        * msg (`PointStamped`) - The ROS message.
        * approx (`bool`) - Whether this is an approximate location.
        """
        if approx:
            self._home_odom_approx_position = msg
        else:
            self._home_odom_position = msg

    @sync(swarmie_lock)
    def _targets(self, msg):
        self.targets_index = (self.targets_index + 1) % self.CIRCULAR_BUFFER_SIZE
        self.targets[self.targets_index] = msg.detections


    def __drive(self, request, **kwargs):
        request.obstacles = ~0
        if 'ignore' in kwargs :
            request.obstacles = ~kwargs['ignore']
            if kwargs['ignore'] & Obstacle.INSIDE_HOME == Obstacle.INSIDE_HOME:
                rospy.logwarn_throttle(10.0,
                                       'Ignoring INSIDE_HOME exceptions.')
            if kwargs['ignore'] & Obstacle.VISION_HOME == Obstacle.TAG_HOME:
                rospy.logwarn_throttle(
                    10.0,
                    'Ignoring only TAG_HOME and not also HOME_CORNER. ' +
                    'You usually want to use ignore=VISION_HOME'
                )

        request.timeout = 120
        if 'timeout' in kwargs :
            request.timeout = kwargs['timeout']

        if 'linear' in kwargs:
            request.linear = kwargs['linear']

        if 'angular' in kwargs:
            request.angular = kwargs['angular']
            
        value = self.control([request]).result.result

        # Always raise AbortExceptions when the service response is USER_ABORT,
        # even if throw=False was passed as a keyword argument.
        if value == MoveResult.USER_ABORT:
            raise AbortException(value)

        if 'throw' not in kwargs or kwargs['throw'] : 
            if value == MoveResult.OBSTACLE_SONAR :
                raise ObstacleException(value)
            elif value == MoveResult.OBSTACLE_TAG : 
                raise TagException(value)
            elif value == MoveResult.OBSTACLE_HOME : 
                raise HomeException(value)
            elif value == MoveResult.PATH_FAIL :
                raise PathException(value)
            elif value == MoveResult.TIMEOUT :
                raise TimeoutException(value)
            elif value == MoveResult.INSIDE_HOME:
                raise InsideHomeException(value)
            elif value == MoveResult.OBSTACLE_CORNER:
                raise HomeCornerException(value)
        
        return value

    def _get_speeds(self, speeds):
        if speeds[0] is None or speeds[1] is None:
            raise AttributeError(
                'Drive speeds are not set. Have you called swarmie.start()?'
            )
        return {'linear': speeds[0], 'angular': speeds[1]}

    @property
    def speed_slow(self):
        """Get the current slow speeds in the format:
            {'linear': lin_speed, 'angular': ang_speed}
        """
        return self._get_speeds(self._drive_speeds['slow'])

    @property
    def speed_normal(self):
        """Get the current normal/default speeds in the format:
            {'linear': lin_speed, 'angular': ang_speed}
        """
        return self._get_speeds(self._drive_speeds['normal'])

    @property
    def speed_fast(self):
        """Get the current fast speeds in the format:
            {'linear': lin_speed, 'angular': ang_speed}
        """
        return self._get_speeds(self._drive_speeds['fast'])
        
    def stop(self):
        '''Stop the rover by placing it in manual mode.''' 
        msg = UInt8() 
        msg.data = 1
        self.mode_publisher.publish(msg)
    
    def go(self):
        '''Start the rover by placing it into autonomous mode.'''
        msg = UInt8() 
        msg.data = 2
        self.mode_publisher.publish(msg)

    def circle(self, **kwargs):
        '''Drive in a small circle
        
        Args:        
            
        * See keyword arguments for drive commands 

        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        '''
        if self.simulator_running():
            time = 17
        else:
            time = 8

        if random.randint(0,1) == 0 :
            return self.timed_drive(time, 0.1, self.speed_normal['angular'], **kwargs)
        else:
            return self.timed_drive(time, 0.1, -self.speed_normal['angular'], **kwargs)

    def timed_drive(self, time, linear, angular, **kwargs):
        '''Send the specified velocity command for a given period of time.
        
        Args:
        
        * time (float) The duration of the timed command. 
        * linear (float) The linear velocity of the rover (m/s) 
        * angular (float) The angular velocity of the rover (radians/s) 

            SEE: Keyword Arguments for drive commands 
            
        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        '''
        req = MoveRequest(
            timer=time, 
            linear=linear, 
            angular=angular, 
        )
        return self.__drive(req, **kwargs)
    
    def drive(self, distance, **kwargs):
        '''Drive the specfied distance
        
        Args:
        
        * `distance` (`float`) Meters to drive. 
        
        Keyword arguments:
    
        * `ignore` (`int`) - Bitmask with Obstacle messages that will be ignored while driving.         
        * `throw` (`bool`) - Raise a DriveException if an obstacle is encountered (default True). 
        * `timeout` (`int`) - The command will fail after this number of seconds (defulat: 120)
        * `linear` (`float`) - The linear velocity used to execute the command (default is set with DynParam)
        * `angular` (`float`) - The angular velocity used to execute the command (default is set with DynParam)

        Returns:
            
        * If `throw=False` was given returns a `mobility_msgs.msg.MoveResult` containing an integer. \
         Values are described in `src/mobility/msg/MoveResult.msg`. Note: if the rover is placed in \
         manual mode mid-drive, a `mobility`swarmie.AbortException` will be raised regardless of the \
         `throw` keyword argument's value.

        * If `throw=True` (the default) is given the return value will be converted into an exception. \
        The following exceptions are defined:
        
            * `mobility.swarmie.DriveException` - Base class for driving exceptions. 

            * `mobility.swarmie.VisionException` - Base class for exceptions caused by seeing a tag            

            * `mobility.swarmie.TagException` - Exception caused when the target tag (0) is seen.

            TODO: add AprilTagBoundaryException
            * `mobility.swarmie.AprilTagBoundaryException` - Exception caused when boundary tag (1) is seen.

            * `mobility.swarmie.HomeException` - Exception caused when the home tag (256) is seen. 

            * `mobility.swarmie.ObstacleException` - Exception caused when sonar senses the rover \
            is close to an obstacle.  

            * `mobility.swarmie.PathException` - Exception caused when the rover encounters an \
            error while driving. If the rover detects a large angle between itself and its goal this \
            happens. It's usually because the rover has driven over some kind of obstacle.              

            * `mobility.swarmie.AbortException` - Exception caused when the user places the rover into \
            manual mode while it was driving. 

            * `mobility.swarmie.TimeoutException` - Exception caused when the drive command fails to \
            complete in the amount of time specified with the `timeout=` argument. 
             
            * `mobility.swarmie.InsideHomeException` - Exception caused when the rover thinks it's inside \
            of the home ring.

            * `mobility.swarmie.HomeCornerException` - Exception caused when the rover sees a corner of \
            the home ring.

        Examples:

            >>> from mobility.swarmie import swarmie
            >>> swarmie.start(node_name='swarmie')
            >>>
            >>> # Drive the current default speed
            >>> swarmie.drive(1)
            >>>
            >>> # Drive using the current slow speed
            >>> swarmie.drive(1, **swarmie.speed_slow)
            >>>
            >>> # Drive using the current fast speed
            >>> swarmie.drive(1, **swarmie.speed_fast)
        '''
        req = MoveRequest(
            r=distance, 
        )        
        return self.__drive(req, **kwargs)

    def turn(self, theta, **kwargs):
        '''Turn theta degrees 
        
        Args: 
        
        * `theta` (float) Radians to turn 

        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        '''
        req = MoveRequest(
            theta=theta, 
        )
        return self.__drive(req, **kwargs)

    def wait(self, time, **kwargs):
        '''Wait for a period of time. This can be used to check for obstacles
        
        Args:

        * `time` (`float`) seconds to wait. 
            
        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        '''
        req = MoveRequest(
            timer=time, 
        )

        return self.__drive(req, **kwargs)
                
    def set_wrist_angle(self, angle):
        '''Set the wrist angle to the specified angle
        
        Args:
        
        * `angle` (`float`) Wrist angle in radians. 
        '''
        self.wrist_publisher.publish(Float32(angle))

    def set_finger_angle(self, angle):
        '''Set the finger angle to the spedified angle
        
        Args:
        
        * `angle` (`float`) Finger angle in radians.
        '''
        self.finger_publisher.publish(Float32(angle))
        
    def wrist_down(self):
        '''Lower the wrist to put it in the pickup position'''
        self.wrist_publisher.publish(Float32(1.25))
    
    def wrist_up(self):
        '''Raise the wrist to the top'''
        self.wrist_publisher.publish(Float32(0.0))
    
    def wrist_middle(self):
        '''Put the wrist in the middle of its travel so that the rover can drive with a block'''
        self.wrist_publisher.publish(Float32(0.75))

    def fingers_open(self):
        '''Open the fingers'''
        self.finger_publisher.publish(Float32(math.pi/2))
    
    def fingers_close(self):
        '''Close the fingers'''
        self.finger_publisher.publish(Float32(0.0))
    
    def print_state_machine(self, msg):
        '''Print a message to the /rover/state_machine topic.'''
        s = String()
        s.data = msg 
        self.sm_publisher.publish(s)

    def print_infoLog(self, msg):
        '''Print a message to the /infoLog topic this is viewable in the GUI'''
        s = String()
        s.data = msg 
        self.info_publisher.publish(s)

    def print_status(self, msg):
        '''Print a message to the /rover/status topic this is viewable in the GUI'''
        s = String()
        s.data = msg 
        self.status_publisher.publish(s)
        
    def has_block(self):
        '''Try to determine if a block is in our grasp. 
        
        Uses the algorithm:

        * Put wrist down to a middle position. Can help avoid any sun glare or \
          shadows seen in wrist up position (This skipped in the simulation \
          because there is no sun glare).
        * Check if we can see a block that's close to the camera. If so, return `True`
        * Raise the wrist all the way up.
        * Check if the center sonar is blocked at a close distance. If so, return `True`
        * Check if we can see a block that's very close. If so, return `True`
        * Return `False`
        '''
        if self.simulator_running():
            wrist_angles = (0.3, 0.0)
            max_z_dist = 0.13 #0.151+ is the top of the cube on the ground in the sim
        else:
            wrist_angles = (0.55, 0.0)
            max_z_dist = 0.18
        
        for angle in wrist_angles:
            self.set_wrist_angle(angle)
            rospy.sleep(1)
            blocks = self.get_targets_buffer(age=1, id=0)
            blocks = sorted(blocks, key=lambda x: abs(x.pose.pose.position.z))
            if len(blocks) > 0 :
                nearest = blocks[0]
                z_dist = nearest.pose.pose.position.z
                if abs(z_dist) < max_z_dist:
                    return True
        
        # Third test: is something blocking the center sonar at a short range.
        obstacles = self.get_obstacle_condition()        
        if obstacles & Obstacle.SONAR_BLOCK :
            return True
        # The block does not affect the sonar in the simulator.
        
        return False 
        
        
    def simulator_running(self): 
        '''Helper Returns True if there is a /gazebo/link_states topic otherwise False'''
        for t in rospy.get_published_topics(): 
            if t[0] == '/gazebo/link_states' :
                # This is the simulator
                return True
        return False

    def get_latest_targets(self,id=-1):
        """ Return the latest `apriltags2to1.msg.AprilTagDetectionArray`. (it might be out of date)
        and will be affected by twinkeling with an optional id flag"""
        # if self._is_moving():  if not possibly call get_targets_buffer with the last second of detections
        if id == -1:  # all of the tags
            return self.targets[self.targets_index]
        else:  # only the specified tags with id
            return [tag for tag in self.targets[self.targets_index] if tag.id == id]

    def get_targets_buffer(self, age=8, cleanup=True, id=-1):
        ''' Return a list of AprilTagDetections received in the last 'age' seconds,
        filtered by id, with duplicates removed, if specified.

        Args
        
        * `age` (`float`) - how many seconds worth of the buffer to return.
        * `cleanup` (`bool`) - default to True, will remove duplicate detections 
        * `id` (`int`) - default to -1(all tags), can be used to filter specific tags, cleanup must be True

        Returns:

        * `buffer` (`list` [`apriltags_2to1.msg._AprilTagDetection.AprilTagDetection`]) - the target detections buffer
        '''
        buffer = sum(self.targets, [])
        if cleanup:
            buffer = filter_detections(buffer, age, id, dist=0.01)
        return buffer

    def get_plan(self, goal, tolerance=0.0, use_home_layer=True):
        '''Get plan from current location to goal location.

        Args:

        * `goal` (`geometry_msgs/Point`) or ('geometery_msgs/Pose2D`) - The \
         goal location in the /odom frame.
        * `tolerance` (`float`) - The acceptable distance to the goal you \
         are willing to have the path return.
        * `use_home_layer` (`bool`) - Whether to plan a path considering \
         mapped home tags as obstacles.

        Returns:

        * `plan` (`nav_msgs/GetNavPlanResponse`) - contains a `nav_msgs/Path` \
         with an array of `geometry_msgs/PoseStamped` poses to navigate to.

        Raises:

        * (`rospy.ServiceException`) - if no path to the goal can be found. \
         This can happen if you requested an impossible goal to navigate to \
         given the current map and obstacle layers.
        '''
        request = GetNavPlanRequest()
        request.use_home_layer.data = use_home_layer

        cur_loc = self.get_odom_location().get_pose()
        request.start.pose.position.x = cur_loc.x
        request.start.pose.position.y = cur_loc.y
        request.goal.pose.position.x = goal.x
        request.goal.pose.position.y = goal.y
        if tolerance > 0:
            request.tolerance = tolerance
        else:
            request.tolerance = 0.0

        return self._get_plan(request)

    def imu_is_finished_validating(self):
        '''Query the IMU node to find out whether it's done validating the \
        extended calibration file.

        Returns:
        * `is_finished` (`bool`) - True if the IMU node is finished, or if \
        this is a simulated rover, in which case the IMU node isn't running. \
        False otherwise.
        '''
        if self.simulator_running():
            return True

        return self._imu_is_finished_validating().is_calibrated

    def imu_needs_calibration(self):
        '''Query the IMU node to find out whether it needs to be calibrated \
        using the fallback 2D method at the start of a round.

        Returns:
        * `needs_calibration` (`bool`) - True if the IMU still needs to be \
        calibrated. This can happen if the extended calibration file is \
        missing, invalid, or corrupted. False if this is a simulated rover or \
        the IMU has been successfully calibrated.
        '''
        if self.simulator_running():
            return False

        return self._imu_needs_calibration().is_calibrated

    def start_imu_calibration(self):
        '''Start fallback calibration Step One for the rover's IMU.

        This calibration should be perfomed if the extended calibration data
        file is missing, corrupted, or invalid.

        Raw accelerometer and magnetometer is collected during the
        calibration process. During this time, the rover should spin in place
        on the ground.'''
        self._start_imu_calibration()

    def start_misalignment_calibration(self):
        '''Start calibration Step Two for the IMU's misalignment.

        Raw magnetometer data is collected while the rover performs at least
        one slow 2D rotation with its z axis up. This can be performed by
        having the rover spin slowly in place on level ground using the teleop.

        This is only one third of a typical misalignment calibration procedure,
        but since the rover only operates in two dimensions on relatively level
        ground, it should be ok to skip the x-down and y-down rotations that
        are part of a complete misalignment calibration.'''
        self._start_misalignment_calibration()

    def start_gyro_bias_calibration(self):
        '''Start calibration Step Three for the rover's IMU.

        Calibrate gyroscope bias. Leave rover in place for a few seconds.'''
        self._start_gyro_bias_calibration()

    def start_gyro_scale_calibration(self):
        '''Start calibration Step Four for the rover's IMU.

        Calibrate gyroscope scale factor. Rover must rotate exactly 180 degrees
        in one direction during first 10 seconds afer calling this function,
        and rotate exactly 180 degrees in the opposite direction during the
        second 10 seconds after calling this function. Progress can be
        monitored in rdb.py or by echoing the /infoLog topic.'''
        self._start_gyro_scale_calibration()

    def store_imu_calibration(self):
        '''Finish calibrating the IMU on a rover. Save calibration matrices to \
        the parameter server.'''
        self._store_imu_calibration()
        
    def get_odom_location(self):
        '''Returns a `mobility.swarmie.Location` according to Odometery. This location will not 
        jump arround and is locally accurate but will drift over time.'''
        with swarmie_lock :
            return self.OdomLocation
    
    def get_obstacle_condition(self):
        '''Returns the current obstacle condition. The presence of obstacles is indicated
        by bits in the returned integer. 
        
        The definition of the bits can be found in `src/swarmie_msgs/msg/Obstacle.msg`
        
        Bit definitions:
            
            bit 1: SONAR_LEFT
            bit 2: SONAR_RIGHT
            bit 3: SONAR_CENTER
            bit 4: SONAR_BLOCK
            bit 8: TAG_TARGET
            bit 9: TAG_HOME
            bit 10: INSIDE_HOME
            bit 11: HOME_CORNER

        Bit masks:
        
            IS_SONAR = SONAR_LEFT | SONAR_CENTER | SONAR_RIGHT | SONAR_BLOCK 
            IS_VISION = TAG_TARGET | TAG_HOME | INSIDE_HOME | HOME_CORNER

        Convenience bit masks. These are more useful when driving the rover than when
        checking the obstacle condition:

            VISION_SAFE = TAG_TARGET | TAG_HOME | HOME_CORNER
            VISION_HOME = TAG_HOME | HOME_CORNER
        '''
        with swarmie_lock : 
            return self.Obstacles

    def get_home_odom_location(self, approx=False):
        '''Recall the home odometry location. This is probably the most reliable memory
        of the location of the nest. Odometry drifts, so if we haven't seen home in a 
        while this will be off.

        Args:
        * approx (`bool`) - Whether to get the less accurate, but easier to
          update approximate home location.
        
        Returns: 

        * (`geometry_msgs.msg.Point`) : The location of home.
        '''
        if approx:
            if self._home_odom_approx_position is None:
                rospy.logwarn(
                    ("{}: No approximate home location has been received " +
                     "yet, returning odom's origin as the estimated home " +
                     "location.").format(self.rover_name)
                )
                return Point(x=0, y=0)

            return Point(x=self._home_odom_approx_position.point.x,
                         y=self._home_odom_approx_position.point.y)

        if self._home_odom_position is None:
            # Try to set the home position using tf. This is unlikely, but could
            # possibly happen while running behaviors individually as standalone
            # scripts and this function is called before the most recent latched
            # home_point message has been received.
            try:
                home_origin = PoseStamped()
                home_origin.header.frame_id = 'home'
                home_origin.header.stamp = rospy.Time.now()
                home_odom = self.transform_pose('odom', home_origin, timeout=1.0)

                rospy.logwarn(("{}: No home location has been received yet, " +
                               "using tf to calculate it.").format(self.rover_name))
                return Point(x=home_odom.pose.position.x,
                             y=home_odom.pose.position.y)

            except tf.Exception:
                rospy.logwarn(("{}: No home location has been received yet, " +
                               "returning odom's origin as the estimated home " +
                               "location.").format(self.rover_name))
                return Point(x=0, y=0)

        return Point(x=self._home_odom_position.point.x,
                     y=self._home_odom_position.point.y)

    def has_home_odom_location(self, approx=False):
        '''Check to see if the rover knows home's odometry location.

        Args:
        * approx (`bool`) - Whether to check the less accurate, but easier to
          update approximate home location.

        Returns:
        
        * (`bool`): `True` if the rover knows where home is, `False` otherwise.
        '''
        home_odom = self.get_home_odom_location(approx=approx)

        return abs(home_odom.x) > 0.01 and abs(home_odom.y) > 0.01
    
    def drive_to(self, place, claw_offset=0, **kwargs):
        '''Drive directly to a particular point in space. The point must be in 
        the odometry reference frame. 
        
        Arguments:
        
        * `place`: (`geometry_msgs.msg.Point` or `geometry_msgs.msg.Pose2D`): The place to drive.

        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
        * claw_offset to the odometry reference frame.  Appropriate value
        to be passed in, otherwise the reference frame remains unchanged.
            
        '''
        loc = self.get_odom_location().get_pose()
        dist = math.hypot(loc.y - place.y, loc.x - place.x)
        angle = angles.shortest_angular_distance(loc.theta, 
                                                 math.atan2(place.y - loc.y,
                                                            place.x - loc.x))
        effective_dist = dist - claw_offset

        if effective_dist < 0:
            # The driver API skips the turn state if the request distance is
            # negative. This ensures the rover will perform the turn before
            # backing up slightly in this case.
            self.turn(angle, **kwargs)
            return self.drive(effective_dist, **kwargs)

        req = MoveRequest(
            theta=angle, 
            r=effective_dist,
        )        
        return self.__drive(req, **kwargs)
    
    def set_heading(self, heading, **kwargs):
        '''Turn to face an absolute heading in radians. (zero is east)
        
        Arguments:
        
        * `heading`: (`float`) The heading in radians.

        Keyword Arguments/Returns/Raises:
        
        * See `mobility.swarmie.Swarmie.drive`
            
        '''
        loc = self.get_odom_location().get_pose()
        angle = angles.shortest_angular_distance(loc.theta, heading)
        self.turn(angle, **kwargs)
    
    @sync(swarmie_lock)
    def is_moving(self):
        ''' calls _is_moving that uses OdomLocation angular.z & linear.x  
        Returns: 

        * (`bool`) : True if swarmie is moving and False if stationary or no Odometry
        '''
        return self._is_moving()
        
    def _is_moving(self):
        ''' uses OdomLocation angular.z & linear.x  
        Returns: 

        * (`bool`) : True if swarmie is moving and False if stationary or no Odometry
        '''
        if self.OdomLocation.Odometry is None:
            return False

        return is_moving(self.OdomLocation.Odometry)

    def transform_pose(self, target_frame, pose, timeout=3.0):
        """Transform PoseStamped into the target frame of reference.
        Returns a PoseStamped in the target frame.

        Args:
        * target_frame (`string`) - the frame of reference to transform to. Ex: '/odom' or `/base_link`
        * pose (`PoseStamped`) - the pose of the tag in the /camera_link frame
        * timeout (`float`) - the time to wait for the transform

        Returns:
        * pose - PoseStamped the pose of the tag in the /odom frame

        Raises:
        * tf.Exception if timeout is exceeded
        """
        target_frame = self.rover_name + '/' + target_frame.strip('/')

        swarmie.xform.waitForTransform(
            target_frame,
            pose.header.frame_id,
            pose.header.stamp,
            rospy.Duration(timeout)
        )

        return swarmie.xform.transformPose(target_frame, pose)

    def get_nearest_block_location(self, targets_buffer_age=0):
        '''Find the block closest to the rover's claw.

        Args:

        * `targets_buffer_age` (`float`) - how many seconds worth of the AprilTagDetections
          buffer to use. The default value of 0 uses `Swarmie.get_latest_targets()`, which
          you can rely on to return only targets currently in view, but may be affected by
          tag flicker from frame to frame.

        Returns:

        * (`geometry_msgs/Point`) The X, Y, Z location of the nearest block, or `None` if
          no blocks are seen or the nearest tag is a home tag.
        '''
        # Given the claw's position relative to the base_link is an x-only
        # translation, we can use this offset to modify tag poses in the
        # base_link frame and estimate their position relative to the claw.
        claw_offset = 0.2  # meters

        if targets_buffer_age > 0:
            detections = self.get_targets_buffer(age=targets_buffer_age)
        else:
            detections = self.get_latest_targets()

        detections_xformed = []
        now = rospy.Time.now()

        # Wait for a little more than a 1/10th sec for each transform, since
        # transforms are published at approximately 10 Hz in this system.
        timeout = 0.15

        for det in detections:  # type: AprilTagDetection
            try:
                if det.id == 0:
                    ps_det = block_pose(det, self.block_size)
                else:
                    ps_det = det.pose

                # One simple method of getting a tag's current position relative
                # to the base_link frame is using a 2-step transform: first into
                # the fixed odom frame, and then into the base_link frame. We
                # need to re-stamp the pose after performing the first transform
                # in order to get the tag's current pose in the base_link frame,
                # instead of its pose in the base_link frame when it was seen.
                ps_odom = swarmie.transform_pose('odom', ps_det,
                                                 timeout=timeout)
                ps_odom.header.stamp = now
                ps_base_link = swarmie.transform_pose('base_link', ps_odom,
                                                      timeout=timeout)
                ps_base_link.pose.position.x -= claw_offset

                detections_xformed.append(
                    (AprilTagDetection(det.id, det.size, ps_odom),
                     AprilTagDetection(det.id, det.size, ps_base_link))
                )
            except tf.Exception as e:
                rospy.logwarn_throttle(
                    1.0,
                    ('Transform exception in' +
                     'Swarmie.get_nearest_block_location(): {}').format(e)
                )

        if len(detections_xformed) == 0:
            return None

        # Sort blocks by their distance from the base_link frame
        detections_xformed = sorted(
            detections_xformed,
            key=lambda x: math.sqrt(x[1].pose.pose.position.x**2
                                    + x[1].pose.pose.position.y**2
                                    + x[1].pose.pose.position.z**2)
        )

        nearest = detections_xformed[0][0]

        # Check for a home tag between the rover and the block.
        if nearest.id == 256:
            return None

        return nearest.pose.pose.position

    def add_resource_pile_location(self, detection_time_tolerance=0.4, override=False, ignore_claw=False):
        '''Remember the search exit locations.
            Args:
            * detection_time_tolerance (`double`) - the time from now 
            * override (`bool`) - True will add to the list regarless of the number of tags in view
            * ignore_claw (`bool`) - True will remove all detections within the claw
        '''
        # TODO:project location to be in front of the rover & put in the homeframe
        # TODO: Make sure the location is not inside of home
        detections = self.get_targets_buffer(age=detection_time_tolerance, id=0)

        if ignore_claw:  # will remove cubes within the claw
            min_z_dist = 0.18
            if self.simulator_running():
                min_z_dist = .11
            detections = [d for d in detections
                          if d.pose.pose.position.z > min_z_dist]

        cubes = [block_detection(d, self.block_size) for d in detections]
        cubes = filter_detections(cubes, dist=swarmie.block_size - 0.01)
        num_cubes = len(cubes)

        # if 0,1 cubes are detected don't bother adding to the list unless overridden
        if num_cubes <= 1 and not override:
            rospy.loginfo('I only see ' + str(num_cubes) + ' cubes, Not Adding to list')
            return
        
        if num_cubes == 0: 
            location = self.get_odom_location().get_pose()
        else:
            location = swarmie.transform_pose('odom', cubes[0].pose, timeout=3
                                              ).pose.position
        pile_locations_list = rospy.get_param('resource_pile_locations', [])

        rospy.loginfo('Called add_resource_pile_location, num_cubes:'
                      + str(num_cubes)
                      + ' x:' + str(location.x)
                      + ' y:' + str(location.y))
        home = self.get_home_odom_location()
        if abs(location.x - home.x) < .5 and abs(location.y - home.y) < .5: 
            rospy.logwarn(self.rover_name + ": Pile inside home")
            return
        # numpy floats are not compatible with rosparam so have to convert to float
        pile_locations_list.append({'num_cubes': num_cubes,
                                    'x': float(location.x),
                                    'y': float(location.y)})
        rospy.set_param('resource_pile_locations', pile_locations_list)

    def remove_resource_pile_location(self, odom_to_remove, threshold=.4):
        """ remove_resource_pile_location should be called after search goes here and does not find anything 
        Args:
            * odom_to_remove (`geometry_msgs.msg.Pose2D`) - the odom that will be used to remove cube piles winthin
            * threshold (`float`) - the distance from odom_to_remove to remove from the list
        """
        pile_locations_list = rospy.get_param('resource_pile_locations', [])
        # this list comprehension will omit the matching dicts
        pile_locations_list = [x for x in pile_locations_list
                               if abs(odom_to_remove.x - x['x']) > threshold or
                               abs(odom_to_remove.y - x['y']) > threshold]
        if pile_locations_list:
            rospy.set_param('resource_pile_locations', pile_locations_list)
        else:
            if rospy.has_param('resource_pile_locations'):
                rospy.delete_param('resource_pile_locations')

    def get_resource_pile_locations(self):
        """ Gets the ros peram resource_pile_locations which contains a list of dicts that contains num_tags x & y"""
        return rospy.get_param('resource_pile_locations', [])

    def get_best_resource_pile_location(self):
        '''Get the odom (location) where pickup or planner saw most lucrative pile
        baised on number of tags seen then by closeness to the swarmie
        
        Returns:
        * `geometry_msgs.msg.Pose2D`: odom_pose - The pose in the /odom frame

        Will return invalid poses (containing all zeroes) if search exit
        location hasn't been set yet.

        Examples:
        >>> odom_pose = swarmie.get_best_resource_pile_location()
        >>> swarmie.drive_to(odom_pose, ignore=Obstacle.TAG_HOME|Obstacle.TAG_TARGET|Obstacle. )
        '''
        # TODO: when have in terms of homeframe convert to odom
        # for pile_locations_list if l[frame] == 'home' transform to odom
        pile_locations_list = rospy.get_param('resource_pile_locations', [])
        # Get the entry with the most tags
        if not pile_locations_list:
            return Pose2D(0, 0, 0)
        max_num_cubes = max(pile_locations_list,
                            key=lambda k: k['num_cubes'])['num_cubes']
        locations_w_most_tags = [pile for pile in pile_locations_list if
                                 max_num_cubes == pile['num_cubes']]
        cur_pose = swarmie.get_odom_location().get_pose()
        # get the closest pile with the most cubes
        best_pile = min(locations_w_most_tags, key=lambda k: math.sqrt(
                                                    (k['x'] - cur_pose.x) ** 2 +
                                                    (k['y'] - cur_pose.y) ** 2))
        return Pose2D(best_pile['x'], best_pile['y'], 0)  # theta is 0
    
    def has_resource_pile_locations(self):
        '''Check to see if the search exit locations parameter is set.
        Returns:
        * (`bool`): True if the parameter exists, False otherwise.
        '''
        return rospy.has_param('resource_pile_locations')
        

#
# Global singleton instance. No code should make a new instance
# of swarmie. Instead it should access this instance instead as shown
# below.
#
# from movility.swarmie import swarmie
#
# swarmie.drive()
#
swarmie = Swarmie()
