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
    
    Methods:
    
        find_nearest_target() : XXX: Type - Returns a list of targets in the map. 
        
        get_latest_targets() : AprilTagDetectionArray : Return the last successful tag detection messages. 
            See here about AprilTagDetectionArray: http://docs.ros.org/indigo/api/apriltags_ros/html/msg/AprilTagDetectionArray.html
            
        stop() - Stop the rover and put it into manual mode. 
        
        go() - Put the rover into autonomous mode. 
        
        circle() - Drive in a circle. 
        
        timed_drive(time, linear, angular) - Send the drive command for a specified amount of time. 
        
        drive(distance) - Drive the specified distance. 
        
        turn(angle) - Turn the specified radians. 
        
        wait(time) - Wait for an amout of time (will return obstacle messages) 
        
        set_wrist_angle(angle) - Set the wrist to the specified angle (radians). 
        
        set_finger_angle(angle) - Set the finger angle (radians).
        
        wrist_down() - Lower the wrist. 
        
        wrist_up() - Raise the wrist. 
        
        wrist_middle() - Put the wrist into the middle position for carrying a block.
        
        fingers_open() - Open the fingers wide. 
        
        fingers_close() - Close the fingers all the way. 
        
        pickup() - Pickup a block that's in the finger's grasp (won't move the wheels)
        
        putdown() - Place a block on the ground. 
        
        print_state_machine(message) - Print the message to the state_machine debugging topic.
        
        print_infoLog(message) - Print the message to the infoLog topic (displays in the GUI) 
        
        print_status(message) - Print the message to the static target (displays in the GUI) 
        
    ''' 
    
    def __init__(self, rover):
        self.rover_name = rover 
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
        self.find_nearest_target = rospy.ServiceProxy(rover + '/map/find_nearest_target', FindTarget)
        self.get_latest_targets = rospy.ServiceProxy(rover + '/map/get_latest_targets', LatestTarget)
        self.get_obstacle_map = rospy.ServiceProxy(rover + '/map/get_obstacle_map', GetMap)

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
        
            kwargs -- Are documented in move()

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

            kwargs -- Are documened in move() 
            
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
        
        Keyword Arguments: 
        
            ignore (Obstacle) - Obstacle messages that will be ignored while driving. 
            throw (bool) - Raise a DriveException if an obstacle is encountered (default True). 
            timeout (int) - The command will fail after this number of seconds (defulat: 120)
            
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
       
    def putdown(self):
      '''Puts the block down'''
      self.set_wrist_angle(1)
      rospy.sleep(.5)
      self.set_finger_angle(2) #open
      rospy.sleep(.5)
      self.wrist_up()          
