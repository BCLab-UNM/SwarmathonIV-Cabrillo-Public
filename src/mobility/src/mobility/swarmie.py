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
from geometry_msgs.msg import Point 
from apriltags_ros.msg import AprilTagDetectionArray 

import threading 
swarmie_lock = threading.Lock()

from mobility import sync, Location 

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

class Swarmie: 
    '''Class that embodies the Swarmie's API
    
    This is the Python API used to drive the rover. The API interfaces 
    with ROS topics and services to perform action and acquire sensor data. 
    
    The following commands move the rover: 
        
        circle()
        drive()
        timed_drive()
        turn()
        wait()
             
    Keyword Arguments for drive commands:
    
        ignore (Obstacle) 
          Obstacle messages that will be ignored while driving. 
        
        throw (bool) 
          Raise a DriveException if an obstacle is encountered (default True). 
        
        timeout (int) 
          The command will fail after this number of seconds (defulat: 120)
    
    Drive Command Return and Raise 
    
        Drive commands return a mobility_msgs.msg.MoveResult which contains an
        integer. Values are described in 
        
          src/mobility/msg/MoveResult.msg
          
        Unless throw=False is given the return value will be converted into an
        exception. The following exceptions are defined:
        
        DriveException(Exception)
            Base class for driving exceptions. 
            
        VisionException(DriveException)
            Base class for exceptions caused by seeing a tag
            
        TagException(VisionException)
            Exception caused when the target tag (0) is seen.
        
        HomeException(VisionException)
            Exception caused when the home tag (256) is seen. 
            
        ObstacleException(DriveException)
            Exception caused when sonar senses the rover is close to an obstacle.
            
        PathException(DriveException)
            Exception caused when the rover encounters an error while driving.
            If the rover detects a large angle between itself and its goal this 
            happens. It's usually because the rover has driven over some kind of 
            obstacle.  
            
        AbortException(DriveException)
            Exception caused when the user places the rover into manual mode while
            it was driving. 

        TimeoutException(DriveException)
            Exception caused when the drive command fails to complete in the amount
            of time specified with the timeout= argument. 
    ''' 
    
    def __init__(self, rover):
        self.rover_name = rover 
        self.Obstacles = 0
        self.MapLocation = Location(None)
        self.OdomLocation = Location(None)
        self.Targets = AprilTagDetectionArray()
        
        # Intialize this ROS node.
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

        # Connect to services.
        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self._find_nearest_target = rospy.ServiceProxy(rover + '/map/find_nearest_target', FindTarget)
        self._get_obstacle_map = rospy.ServiceProxy(rover + '/map/get_obstacle_map', GetMap)
        self._start_magnetometer_calibration = rospy.ServiceProxy(rover + '/start_magnetometer_calibration', Empty)
        self._store_magnetometer_calibration = rospy.ServiceProxy(rover + '/store_magnetometer_calibration', Empty)

        # Subscribe to useful topics 
        rospy.Subscriber(rover + '/odom/filtered', Odometry, self._odom)
        rospy.Subscriber(rover + '/odom/ekf', Odometry, self._map)
        rospy.Subscriber(rover + '/obstacle', Obstacle, self._obstacle)
        rospy.Subscriber(rover + '/targets', AprilTagDetectionArray, self._targets)

        # Transform listener. Use this to transform between coordinate spaces.
        self.xform = tf.TransformListener() 

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
        self.Targets = msg

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
            
            SEE: Keyword Arguments for drive commands 

        Returns/Raises:
            Documented in drive()            
        ''' 
        if random.randint(0,1) == 0 :
            return self.timed_drive(15, 0.1, 0.62, **kwargs)
        else:
            return self.timed_drive(15, 0.1, -0.62, **kwargs)

    def timed_drive(self, time, linear, angular, **kwargs):
        '''Send the specified velocity command for a given period of time.
        
        Args:
            time (float) The duration of the timed command. 
            linear (float) The linear velocity of the rover (m/s) 
            angular (float) The angular velocity of the rover (radians/s) 

            SEE: Keyword Arguments for drive commands 
            
        Returns/Raises:
            Documented in drive()
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
            distance (float) Meters to drive. 
        
            SEE: Keyword Arguments for drive commands 

        Returns:
            
            Obstacle: Indicating what obstacle was hit (0 for success)
            This function will only return an obstacle message if throw=False is passed. 
             
        '''
        req = MoveRequest(
            r=distance, 
        )        
        return self.__drive(req, **kwargs)

    def turn(self, theta, **kwargs):
        '''Turn theta degrees 
        
        Args: 
            theta (radians) Degrees to turn 

            SEE: Keyword Arguments for drive commands

        Returns/Raises:
            Documented in move()
        '''
        req = MoveRequest(
            theta=theta, 
        )
        return self.__drive(req, **kwargs)

    def wait(self, time, **kwargs):
        '''Wait for a period of time. This can be used to check for obstacles
        
        Args:
            time (float) seconds to wait. 
            
        Returns/Raises:
            Documented in move()        
        '''
        req = MoveRequest(
            timer=time, 
        )

        return self.__drive(req, **kwargs)
                
    def set_wrist_angle(self, angle):
        '''Set the wrist angle to the specified angle
        
        Args:
            angle (float) Wrist angle in radians. 
        '''
        self.wrist_publisher.publish(Float32(angle))

    def set_finger_angle(self, angle):
        '''Set the finger angle to the spedified angle
        
        Args:
            angle (float) Finger angle in radians.
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
        '''Return True if the center sonar detects an object at very short distance.''' 

        self.wrist_up()
        rospy.sleep(2)
        
        # First test: is something blocking the center sonar at a short range.
        obstacles = self.get_obstacle_condition()        
        if obstacles & Obstacle.SONAR_BLOCK :
            return True

        # Second test: Can we see a bock that's close to the camera.
        blocks = self.get_latest_targets()
        blocks = sorted(blocks.detections.detections, key=lambda x : abs(x.pose.pose.position.x))
        if len(blocks) == 0 :
            return False
        
        nearest = blocks[0]
        x_dist = nearest.pose.pose.position.x 
        if abs(x_dist) < 0.1 :
            return True 
            
        # Third test: The block never seems to affect the sonar in the simulator. 
        # Also, the grasped block rarely seems to be recognized in the simulator. 
        # Which is whack. 
        topics = rospy.get_published_topics() 
        for t in topics : 
            if t[0] == '/gazebo/link_states' :
                # This is the simulator
                return True
        
        return False
     
    def pickup(self):
      '''Picks up the block'''
      self.set_finger_angle(2) #open
      rospy.sleep(1) #not sure if sleeps are okay here
      self.set_wrist_angle(1)
      rospy.sleep(.3)
      self.set_finger_angle(.5) #close
      rospy.sleep(0.5)
      self.wrist_up()
       
    def putdown(self):
      '''Puts the block down'''
      self.set_wrist_angle(1)
      rospy.sleep(.5)
      self.set_finger_angle(2) #open
      rospy.sleep(.5)
      self.wrist_up()          

    def find_nearest_target(self) :
        '''Return a XXX that is the odom location of the nearest target on the map.''' 
        return self._find_nearest_target()
    
    def get_latest_targets(self) :
        '''Return the latest apriltags_ros.msg.ArpilTagDetectionArray. (it might be out of date)'''
        return self.Targets
    
    def get_obstacle_map(self):
        '''Return a XXX that is the obstacle map.'''
        return self._get_obstacle_map()

    def start_magnetometer_calibration(self):
        '''Start calibrating the magnetometer on a rover.'''
        self._start_magnetometer_calibration()
    
    def store_magnetometer_calibration(self):
        '''Finish calibrating the magnetometer on a rover.'''
        self._store_magnetometer_calibration()
        
    def get_odom_location(self):
        '''Returns a mobility.Location according to Odometery. This location will not 
        jump arround and is locally accurate but will drift over time.'''
        with swarmie_lock :
            return self.OdomLocation
    
    def get_gps_location(self):
        '''Returns a mobility.Location that is the output of the EKF that fuses GPS.
        This value has varying accuracy. The accuracy is reported in a covariance matrix. 
        If you want to wait for a __good__ reading use wait_for_fix()'''
        with swarmie_lock :
            return self.MapLocation

    def wait_for_fix(self, distance=4, time=30):    
        '''Wait until the GPS fix is reasonably accurate. This could take a while!

        Arguments: 

            distance: (float) The desired accuracty (+/- distance meters) with a likelyhood 
                of 50%. Defaults to 4 meters.

            time: (int) The number of seconds to wait for a good GPS measurement before giving up
                Defaults to 30 seconds. 

        Returns: 

            A mobility.Location if the fix was successful. None if we waited until the timeout. 
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
        
        The definition of the bits can be found in:
            src/swarmie_msgs/msg/Obstacle.msg
        
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
        
            loc: (mobility.Location) The GPS coordinates to remember. 
        '''
        rospy.set_param('/' + self.rover_name + '/home_gps', 
                        {'x' : loc.Odometry.pose.pose.position.x, 
                         'y' : loc.Odometry.pose.pose.position.y})
    
    def get_home_gps_location(self):
        '''Recall the home GPS location.
        
        Returns: 
        
            (dict) : { 'x' : x_location, 'y' : y_location }
        '''
        return rospy.get_param('/' + self.rover_name + '/home_gps', {'x' : 0, 'y' : 0})

    def has_home_gps_location(self):
        '''Check to see if the home location parameter is set.
        
        Returns:
            (bool): True if the parameter exists, False otherwise.
        '''
        return rospy.has_param('/' + self.rover_name + '/home_gps')
    
    def drive_to(self, place, **kwargs):
        '''Drive directly to a particular point in space. The point must be in 
        the odometry reference frame. 
        
        Arguments:
        
            place: (geometry_msgs.msg.Point) The place to drive.

            SEE: Keyword Arguments for drive commands 
            
        Returns/Raises:
            Documented in drive()
            
        '''
        loc = self.get_odom_location().get_pose()
        dist = math.hypot(loc.y - place.y, loc.x - place.x)
        angle = angles.shortest_angular_distance(loc.theta, 
                                                 math.atan2(place.y - loc.y,
                                                            place.x - loc.x))
        self.turn(angle, **kwargs)
        self.drive(dist, **kwargs)
    
    def set_heading(self, heading, **kwargs):
        '''Turn to face an absolute heading in radians. (zero is east)
        
        Arguments:
        
            heading: (float) The heading in radians.

            SEE: Keyword Arguments for drive commands 
            
        Returns/Raises:
            Documented in drive()
            
        '''
        loc = self.get_odom_location().get_pose()
        angle = angles.shortest_angular_distance(loc.theta, heading)
        self.turn(angle, **kwargs)
