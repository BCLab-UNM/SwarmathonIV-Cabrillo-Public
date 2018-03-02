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

from mobility.srv import Core
from mapping.srv import FindTarget, GetMap
from mobility.msg import MoveResult, MoveRequest
from swarmie_msgs.msg import Obstacle 

from std_srvs.srv import Empty 
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Pose2D
from apriltags_ros.msg import AprilTagDetectionArray 
from rospy.numpy_msg import numpy_msg
from grid_map_msgs.msg import GridMap
from mapping import RoverMap 

import threading 

swarmie_lock = threading.Lock()

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

class Swarmie: 
    '''Class that embodies the Swarmie's API
    
    This is the Python API used to drive the rover. The API interfaces 
    with ROS topics and services to perform action and acquire sensor data. 
    ''' 
    
    def __init__(self, rover, **kwargs):
        '''Constructor.

        Args:

        * `rover` (`string`) - Name of the rover.

        Keyword arguments:

        * `node_suffix` (`string`) - Optional argument to start up this node
        without killing the main /rover_CONTROLLER node. Used by teleop_keyboard
        '''
        self.rover_name = rover 
        self.Obstacles = 0
        self.MapLocation = Location(None)
        self.OdomLocation = Location(None)
        self.Targets = AprilTagDetectionArray()
        self.TargetsDict = {}
        self.targets_timeout = 3
        
        # Intialize this ROS node.
        if 'node_suffix' in kwargs and kwargs['node_suffix']:
            rospy.init_node(rover + '_CONTROLLER' + kwargs['node_suffix'])
        else:
            rospy.init_node(rover + '_CONTROLLER')

        # Create publishiers. 
        self.sm_publisher = rospy.Publisher(rover + '/state_machine', String, queue_size=10, latch=True)
        self.status_publisher = rospy.Publisher(rover + '/status', String, queue_size=10, latch=True)
        self.info_publisher = rospy.Publisher('/infoLog', String, queue_size=10, latch=True)
        self.mode_publisher = rospy.Publisher(rover + '/mode', UInt8, queue_size=1, latch=True)
        self.finger_publisher = rospy.Publisher(rover + '/fingerAngle/cmd', Float32, queue_size=1, latch=True)
        self.wrist_publisher = rospy.Publisher(rover + '/wristAngle/cmd', Float32, queue_size=1, latch=True)

        # Wait for necessary services to be online. 
        # Services are APIs calls to other neodes. 
        rospy.wait_for_service(rover + '/control')
        rospy.wait_for_service(rover + '/map/find_nearest_target')
        rospy.wait_for_service(rover + '/map/get_obstacle_map')
        rospy.wait_for_service(rover + '/map/get_target_map')

        # Numpy-ify the GridMap 
        GetMap._response_class = rospy.numpy_msg.numpy_msg(GridMap)
        
        # Connect to services.
        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self._find_nearest_target = rospy.ServiceProxy(rover + '/map/find_nearest_target', FindTarget)
        self._get_obstacle_map = rospy.ServiceProxy(rover + '/map/get_obstacle_map', GetMap)
        self._get_target_map = rospy.ServiceProxy(rover + '/map/get_target_map', GetMap)
        self._start_imu_calibration = rospy.ServiceProxy(rover + '/start_imu_calibration', Empty)
        self._start_misalignment_calibration = rospy.ServiceProxy(rover + '/start_misalignment_calibration', Empty)
        self._start_gyro_bias_calibration = rospy.ServiceProxy(rover + '/start_gyro_bias_calibration', Empty)
        self._start_gyro_scale_calibration = rospy.ServiceProxy(rover + '/start_gyro_scale_calibration', Empty)
        self._store_imu_calibration = rospy.ServiceProxy(rover + '/store_imu_calibration', Empty)

        # Transform listener. Use this to transform between coordinate spaces.
        # Transform messages must predate any sensor messages so initialize this first.
        self.xform = tf.TransformListener()
        rospy.sleep(1)

        # Subscribe to useful topics 
        # These topics only update data. The data is used in other places to initialize
        # these and wait for valid data so that users of this data don't get errors.
        rospy.Subscriber(rover + '/odom/filtered', Odometry, self._odom)
        rospy.Subscriber(rover + '/odom/ekf', Odometry, self._map)
        rospy.Subscriber(rover + '/obstacle', Obstacle, self._obstacle)
        rospy.sleep(0.5)

        # The targets subscriber needs odom data since Carter's patch. Make sure odom data
        # exists before we get a target callback.
        rospy.Subscriber(rover + '/targets', AprilTagDetectionArray, self._targets)
        rospy.sleep(1)

        print ('Welcome', self.rover_name, 'to the world of the future.')

    @sync(swarmie_lock)
    def _odom(self, msg) : 
        self.OdomLocation.Odometry = msg
            
    @sync(swarmie_lock)
    def _map(self, msg) : 
        self.MapLocation.Odometry = msg

    @sync(swarmie_lock)
    def _obstacle(self, msg) :
        self.Obstacles &= ~msg.mask 
        self.Obstacles |= msg.msg 

    @sync(swarmie_lock)
    def _targets(self, msg) : 
        self.TargetsDict = {key:tag for key,tag in self.TargetsDict.iteritems() if ((tag.pose.header.stamp.secs + self.targets_timeout ) > rospy.Time.now().secs) }
        #adding currently seen tags to the dict
        self.TargetsDict.update({(round(tag.pose.pose.position.x, 2),round(tag.pose.pose.position.y, 2),round(tag.pose.pose.position.z, 2)): tag for tag in msg.detections })
        #get the tags from the dict and saves them to Targets
        self.Targets.detections = self.TargetsDict.values() 
    
    '''#old code using is moving
        if self._is_moving():
            self.Targets = msg
            #create a dict of tags as values and rounded coordinates as the key
            self.TargetsDict = {(round(tag.pose.pose.position.x, 2),round(tag.pose.pose.position.y, 2),round(tag.pose.pose.position.z, 2)): tag for tag in msg.detections }
            
        else:
            #remove old tags from the dict
            self.TargetsDict = {key:tag for key,tag in self.TargetsDict.iteritems() if ((tag.pose.header.stamp.secs + self.targets_timeout ) > rospy.Time.now().secs) }
            #adding currently seen tags to the dict
            self.TargetsDict.update({(round(tag.pose.pose.position.x, 2),round(tag.pose.pose.position.y, 2),round(tag.pose.pose.position.z, 2)): tag for tag in msg.detections })
            #get the tags from the dict and saves them to Targets
            self.Targets.detections = self.TargetsDict.values() 
    '''
    def __drive(self, request, **kwargs):
        request.obstacles = ~0
        if 'ignore' in kwargs :
            request.obstacles = ~kwargs['ignore']

        request.timeout = 120
        if 'timeout' in kwargs :
            request.timeout = kwargs['timeout']

        value = self.control([request]).result.result

        if 'throw' not in kwargs or kwargs['throw'] : 
            if value == MoveResult.OBSTACLE_SONAR :
                raise ObstacleException(value)
            elif value == MoveResult.OBSTACLE_TAG : 
                raise TagException(value)
            elif value == MoveResult.OBSTACLE_HOME : 
                raise HomeException(value)
            elif value == MoveResult.PATH_FAIL : 
                raise PathException(value)
            elif value == MoveResult.USER_ABORT : 
                raise AbortException(value)
            elif value == MoveResult.TIMEOUT :
                raise TimeoutException(value)
        
        return value
        
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
        if random.randint(0,1) == 0 :
            return self.timed_drive(15, 0.1, 0.62, **kwargs)
        else:
            return self.timed_drive(15, 0.1, -0.62, **kwargs)

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

        Returns:
            
        * If `throw=False` was given returns a `mobility_msgs.msg.MoveResult` containing an integer. \
         Values are described in `src/mobility/msg/MoveResult.msg`    

        * If `throw=True` (the default) is given the return value will be converted into an exception. \
        The following exceptions are defined:
        
            * `mobility.swarmie.DriveException` - Base class for driving exceptions. 

            * `mobility.swarmie.VisionException` - Base class for exceptions caused by seeing a tag            

            * `mobility.swarmie.TagException` - Exception caused when the target tag (0) is seen.

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
          shadows seen in wrist up position.
        * Check if we can see a block that's close to the camera. If so, return `True`
        * Raise the wrist all the way up.
        * Check if the center sonar is blocked at a close distance. If so, return `True`
        * Check if we can see a block that's very close. If so, return `True`
        * Return `False`
        '''

        # First test: Can we see a bock that's close to the camera with the wrist middle.
        self.set_wrist_angle(.55)
        rospy.sleep(1)
        blocks = self.get_latest_targets()
        blocks = sorted(blocks.detections, key=lambda x : abs(x.pose.pose.position.z))
        if len(blocks) > 0 :
            nearest = blocks[0]
            z_dist = nearest.pose.pose.position.z 
            if abs(z_dist) < 0.18 :
                return True 

        # Second test: Can we see a bock that's close to the camera with the wrist up.
        self.wrist_up()
        rospy.sleep(1)
        blocks = self.get_latest_targets()
        blocks = sorted(blocks.detections, key=lambda x : abs(x.pose.pose.position.z))
        if len(blocks) > 0 :
            nearest = blocks[0]
            z_dist = nearest.pose.pose.position.z 
            if abs(z_dist) < 0.15 :
                return True 

        # Third test: is something blocking the center sonar at a short range.
        obstacles = self.get_obstacle_condition()        
        if obstacles & Obstacle.SONAR_BLOCK :
            return True

        # The block does not affect the sonar in the simulator. 
        # Use the below check if having trouble with visual target check.
        # return(self.simulator_running())
        return(self.sees_resource(6))
        
    def simulator_running(self): 
        '''Helper Returns True if there is a /gazebo/link_states topic otherwise False'''
        for t in rospy.get_published_topics(): 
            if t[0] == '/gazebo/link_states' :
                # This is the simulator
                return True
        return False

    def pickup(self):
        '''Picks up the block'''
        finger_close_angle = .5
        if self.simulator_running():
            finger_close_angle = 0

        self.set_finger_angle(2) #open
        rospy.sleep(1)
        self.set_wrist_angle(1)
        rospy.sleep(.7)
        self.set_finger_angle(finger_close_angle) #close
        rospy.sleep(1)
        self.wrist_up()
        return(self.has_block())
       
    def putdown(self):
        '''Puts the block down'''
        self.set_wrist_angle(1)
        rospy.sleep(.7)
        self.set_finger_angle(2) #open
        rospy.sleep(1)
        self.wrist_up()


    def find_nearest_target(self) :
        '''Broken: Return a XXX that is the odom location of the nearest target on the map.''' 
        return self._find_nearest_target()
    
    def get_latest_targets(self) :
        '''Return the latest `apriltags_ros.msg.ArpilTagDetectionArray`. (it might be out of date)'''
        return self.Targets
    
    def get_obstacle_map(self):
        '''Return a `mapping.msg.RoverMap` that is the obstacle map.
            See `./src/mapping/src/mapping/__init__.py` for documentation of RoverMap'''
        return RoverMap(self._get_obstacle_map())

    def get_target_map(self):
        '''Return a `mapping.msg.RoverMap` that is the targets map.
            See `./src/mapping/src/mapping/__init__.py` for documentation of RoverMap'''
        return RoverMap(self._get_target_map())
    
    def start_imu_calibration(self):
        '''Start calibration Step One for the rover's IMU.

        This calibration should be perfomed before the rover starts operating
        in a new environment.

        Raw accelerometer and magnetometer is collected during the
        calibration process. During this time, the rover should perform six
        full in-place rotations, one rotation with each of its body axes
        up and down. These should be slow 2D rotations.

        When the 2D rotations are complete, the rover should also perform
        3D random rotations to put its IMU in as many additional orientations
        as possible.'''
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
        '''Finish calibrating the IMU on a rover. Save calibration file to disk.'''
        self._store_imu_calibration()
        
    def get_odom_location(self):
        '''Returns a `mobility.swarmie.Location` according to Odometery. This location will not 
        jump arround and is locally accurate but will drift over time.'''
        with swarmie_lock :
            return self.OdomLocation
    
    def get_gps_location(self):
        '''Returns a `mobility.swarmie.Location` that is the output of the EKF that fuses GPS.
        This value has varying accuracy. The accuracy is reported in a covariance matrix. 
        If you want to wait for a __good__ reading use `mobility.swarmie.Swarmie.wait_for_fix`'''
        with swarmie_lock :
            return self.MapLocation

    def wait_for_fix(self, distance=4, time=30):    
        '''Wait until the GPS fix is reasonably accurate. This could take a while!

        Arguments: 

        * `distance`: (`float`) The desired accuracty (+/- distance meters) with a likelyhood \
            of 50%. Defaults to 4 meters.

        * `time`: (`int`) The number of seconds to wait for a good GPS measurement before giving up. \
            Defaults to 30 seconds. 

        Returns: 

        * A `mobility.swarmie.Location` if the fix was successful. None if we waited until the timeout. 
        '''        
        for i in xrange(time) : 
            rospy.sleep(1)
            loc = self.get_gps_location()
            vx, vy, vz = loc.get_variances()
            if math.sqrt(vy) < distance : 
                return loc 
        return None
    
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

        Bit masks:
        
            IS_SONAR = SONAR_LEFT | SONAR_CENTER | SONAR_RIGHT | SONAR_BLOCK 
            IS_VISION = TAG_TARGET | TAG_HOME 
        '''
        with swarmie_lock : 
            return self.Obstacles

    def set_home_gps_location(self, loc):
        '''Remember the home GPS location reading. The location can be recalled by other 
        control programs. 
        
        Arguments:
        
        * loc: (`mobility.swarmie.Location`) The GPS coordinates to remember. 
        '''
        rospy.set_param('/' + self.rover_name + '/home_gps', 
                        {'x' : loc.Odometry.pose.pose.position.x, 
                         'y' : loc.Odometry.pose.pose.position.y})
    
    def get_home_gps_location(self):
        '''Recall the home GPS location.
        
        Returns: 

        * (`geometry_msgs.msg.Point`) : The location of home.
        '''
        return Point(**rospy.get_param('/' + self.rover_name + '/home_gps', {'x' : 0, 'y' : 0})) 

    def has_home_gps_location(self):
        '''Check to see if the home location parameter is set.
        
        Returns:
        
        * (`bool`): True if the parameter exists, False otherwise.
        '''
        return rospy.has_param('/' + self.rover_name + '/home_gps')
    
    def set_home_odom_location(self, loc):
        '''Remember the home odometry location. The location can be recalled by other 
        control programs. Set this every time we see the nest to minimize the effect
        of drift. 
        
        Arguments:
        
        * loc: (`mobility.swarmie.Location`) The coordinates to remember. 
        '''
        rospy.set_param('/' + self.rover_name + '/home_odom', 
                        {'x' : loc.Odometry.pose.pose.position.x, 
                         'y' : loc.Odometry.pose.pose.position.y})
    
    def get_home_odom_location(self):
        '''Recall the home odometry location. This is probably the most reliable memory
        of the location of the nest. Odometry drifts, so if we haven't seen home in a 
        while this will be off. 
        
        Returns: 

        * (`geometry_msgs.msg.Point`) : The location of home.
        '''
        return Point(**rospy.get_param('/' + self.rover_name + '/home_odom', {'x' : 0, 'y' : 0})) 

    def has_home_odom_location(self):
        '''Check to see if the home odometry location parameter is set.
        
        Returns:
        
        * (`bool`): `True` if the parameter exists, `False` otherwise.
        '''
        return rospy.has_param('/' + self.rover_name + '/home_odom')
    
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

        req = MoveRequest(
            theta=angle, 
            r=dist-claw_offset,
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

        * (`bool`) : True if swarmie is moving and False if stationary
        '''
        return(self._is_moving())
        
    def _is_moving(self):
        ''' uses OdomLocation angular.z & linear.x  
        Returns: 

        * (`bool`) : True if swarmie is moving and False if stationary
        '''
        return((abs(self.OdomLocation.Odometry.twist.twist.angular.z) > 0.2) or (abs(self.OdomLocation.Odometry.twist.twist.linear.x) > 0.1))
                
    def get_nearest_block_location(self):
        '''Searches the lastest block detection array and returns the nearest target block. (Home blocks are ignored.)

        Nearest block will be the nearest to the camera, which should almost always be good enough.

        Returns:

        * (`geometry_msgs/Point`) The X, Y, Z location of the nearest block, or `None` if no blocks are seen.
        '''
        global rovername, swarmie

        # Finds all visable  apriltags
        blocks = self.get_latest_targets().detections
        if len(blocks) == 0 :
            return None

        # Sort blocks by their distance from the camera_link frame
        blocks = sorted(blocks, key=lambda x :
                        math.sqrt(x.pose.pose.position.x**2
                                  + x.pose.pose.position.y**2
                                  + x.pose.pose.position.z**2))

        nearest = blocks[0]

        # checks for hometag between rover and block.
        if nearest.id==256:
            return None

        self.xform.waitForTransform(self.rover_name + '/odom',
                        nearest.pose.header.frame_id, nearest.pose.header.stamp,
                        rospy.Duration(3.0))
        
        # returns the closes block to the rover.
        return self.xform.transformPose(self.rover_name + '/odom', nearest.pose).pose.position
        
    def set_search_exit_poses(self):
        '''Remember the search exit location.'''
        odom =  self.get_odom_location().get_pose()
        gps = self.get_gps_location().get_pose()
    
        rospy.set_param(
            '/' + self.rover_name + '/search_exit_poses',
            {'odom': {'x': odom.x, 'y': odom.y, 'theta': odom.theta},
             'gps': {'x': gps.x, 'y': gps.y, 'theta': gps.theta}}
        )

    def get_search_exit_poses(self):
        '''Get the odom and gps poses (location and heading) where search last
        exited (saw a tag).

        Returns:

        * (`geometry_msgs.msg.Pose2D`): odom_pose - The pose in the /odom frame
        * (`geometry_msgs.msg.Pose2D`): gps_pose - The pose in the /map frame

        Will return invalid poses (containing all zeroes) if search exit
        location hasn't been set yet.

        Use:
            odom_pose, gps_pose = swarmie.get_search_exit_poses()
            swarmie.drive_to(odom_pose,ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
            swarmie.set_heading(odom_pose.theta,ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
        '''
        poses = rospy.get_param(
            '/' + self.rover_name + '/search_exit_poses',
            {'odom': {'x': 0, 'y': 0, 'theta': 0},
             'gps': {'x': 0, 'y': 0, 'theta': 0}}
        )
        return ((Pose2D(**poses['odom']), Pose2D(**poses['gps'])))

    def has_search_exit_poses(self):
        '''Check to see if the search exit location parameter is set.

        Returns:

        * (`bool`): True if the parameter exists, False otherwise.
        '''
        return rospy.has_param('/' + self.rover_name + '/search_exit_poses')
    

    def sees_resource(self, required_matches,crop=True):
        '''Check to see if a resource can be seen between the grippers
        Args:
        * (`int`): the minimum number of matches required to return true
        Returns:
        * (`bool`): True if the number of required matches has been met, False otherwise.
        '''
        'most of the code from https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_matcher/py_matcher.html'
        ### TODO crop the image on where the tag in the claws would be
        from sensor_msgs.msg import CompressedImage
        import numpy as np
        import cv2
        
        try:
            test = rospy.wait_for_message('/' + self.rover_name + '/camera/image/compressed', CompressedImage, timeout=3)
        except(rospy.ROSException), e:
            print("Camera Broke?")
            print("Error message: ", e)
        np_arr = np.fromstring(test.data, np.uint8)
        img1 = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # queryImage
        #img2 = cv2.imread(rospy.get_param('/' + self.rover_name + '_MOBILITY/april_tag_resource'),0) 
        import pickle
        img2 = pickle.load( open(rospy.get_param('/' + self.rover_name + '_MOBILITY/april_tag_resource_pickel'), "rb" ) ) #this 
        if(crop):       #(y1:y2, x1:x2)
            img1 = img1[60:220, 50:220]
        ''' #for testing1
        cv2.imshow("cropped", img1)
        cv2.waitKey(0)        
        ''' #end for testing1
        
        # Initiate SIFT detector
        #sift = cv2.SIFT() #for opencv2
        sift = cv2.xfeatures2d.SIFT_create()

        # find the keypoints and descriptors with SIFT
        kp1, des1 = sift.detectAndCompute(img1,None)
        kp2, des2 = sift.detectAndCompute(img2,None) #<-- this guy
        '''
        Throws
        OpenCV Error: Bad argument (image is empty or has incorrect depth (!=CV_8U)) in detectAndCompute, file /tmp/binarydeb/ros-kinetic-opencv3-3.3.1/opencv_contrib/xfeatures2d/src/sift.cpp, line 1116
        error: /tmp/binarydeb/ros-kinetic-opencv3-3.3.1/opencv_contrib/xfeatures2d/src/sift.cpp:1116: error: (-5) image is empty or has incorrect depth (!=CV_8U) in function detectAndCompute
        '''

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
                
        '''#for testing2 ###############################################
        from matplotlib import pyplot as plt
        # Need to draw only good matches, so create a mask
        matchesMask = [[0,0] for i in xrange(len(matches))]

        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                matchesMask[i]=[1,0]

        draw_params = dict(matchColor = (0,255,0),
                           singlePointColor = (255,0,0),
                           matchesMask = matchesMask,
                           flags = 0)
        img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,matches,None,**draw_params)
        plt.imshow(img3,),plt.show()
        '''#end for testing2   ###############################################
        
        print("Seen a resource with",len(good)*5,"% confidence")
        
        if len(good)>required_matches-1:
            return(True)
        else:
            return(False)
        
