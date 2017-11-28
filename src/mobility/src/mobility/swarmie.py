'''
Common rover interface in Python
''' 

from __future__ import print_function 

import sys 
import rospy
import math 
import random 

from mobility.srv import Core
from mapping.srv import FindTarget, LatestTarget, GetMap
from mobility.msg import MoveResult, MoveRequest
from swarmie_msgs.msg import Obstacle 

from std_srvs.srv import Empty 
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry

from mobility import sync, synchronized, Location 

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

        rospy.init_node(rover + '_CONTROLLER')

        self.sm_publisher = rospy.Publisher(rover + '/state_machine', String, queue_size=10, latch=True)
        self.status_publisher = rospy.Publisher(rover + '/status', String, queue_size=10, latch=True)
        self.info_publisher = rospy.Publisher('/infoLog', String, queue_size=10, latch=True)

        self.mode_publisher = rospy.Publisher(rover + '/mode', UInt8, queue_size=1, latch=True)
        self.finger_publisher = rospy.Publisher(rover + '/fingerAngle/cmd', Float32, queue_size=1, latch=True)
        self.wrist_publisher = rospy.Publisher(rover + '/wristAngle/cmd', Float32, queue_size=1, latch=True)

        print ('Waiting to connect to services.')
        rospy.wait_for_service(rover + '/control')
        rospy.wait_for_service(rover + '/map/find_nearest_target')
        rospy.wait_for_service(rover + '/map/get_latest_targets')
        rospy.wait_for_service(rover + '/map/get_obstacle_map')
        print ('Connected.')

        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self._find_nearest_target = rospy.ServiceProxy(rover + '/map/find_nearest_target', FindTarget)
        self._get_latest_targets = rospy.ServiceProxy(rover + '/map/get_latest_targets', LatestTarget)
        self._get_obstacle_map = rospy.ServiceProxy(rover + '/map/get_obstacle_map', GetMap)
        self._start_magnetometer_calibration = rospy.ServiceProxy(rover + '/start_magnetometer_calibration', Empty)
        self._store_magnetometer_calibration = rospy.ServiceProxy(rover + '/store_magnetometer_calibration', Empty)

        # Subscribe to useful topics 
        rospy.Subscriber(rover + '/odom/filtered', Odometry, self._odom)
        rospy.Subscriber(rover + '/odom/ekf', Odometry, self._map)
        rospy.Subscriber(rover + '/obstacle', Obstacle, self._obstacle)

    @sync
    def _odom(self, msg) : 
        self.OdomLocation = Location(msg)
            
    @sync    
    def _map(self, msg) : 
        self.MapLocation = Location(msg)

    @sync
    def _obstacle(self, msg) :
        self.Obstacles &= ~msg.mask 
        self.Obstacles |= msg.msg 

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
        request = MoveRequest(
            timer=1, 
            linear=0, 
            angular=0
        )
        val = self.__drive(request, throw=False)
        return bool(val & Obstacle.SONAR_BLOCK)

    def pickup(self):
      '''Picks up the block'''
      self.set_finger_angle(2) #open
      rospy.sleep(1) #not sure if sleeps are okay here
      self.set_wrist_angle(1)
      rospy.sleep(.3)
      self.set_finger_angle(.5) #close
      rospy.sleep(0.5)
      self.wrist_up()
      return(self.has_block())
       
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
        '''Return the latest target detection array. (it might be long out of date)'''
        return self._get_latest_targets()
    
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
        '''Returns the location according to Odometery. This location will not 
        jump arround and is locally accurate but will drift over time.'''
        with synchronized() :
            return self.OdomLocation
    
    def get_gps_location(self):
        '''Returns the location acording to Odometry and GPS. This location will jump
        around because of GPS and, because GPS is not always reliable, may be way way 
        off. Averaging this location for a long time with good GPS signal will be correct
        within a couple of meters.'''
        with synchronized() :
            return self.MapLocation
    
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
        with synchronized() : 
            return self.Obstacles
