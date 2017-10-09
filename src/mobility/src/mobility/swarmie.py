
from __future__ import print_function 

import sys 
import rospy
import math 
import random 

from mobility.srv import Core, FindTarget
from mobility.msg import MoveResult, MoveRequest, Obstacle

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
    '''Class that embodies the Swarmie's API''' 
    
    def __init__(self, rover):
        self.rover_name = rover 
        rospy.init_node(rover + '_CONTROLLER')

        self.sm_publisher = rospy.Publisher(rover + '/state_machine', String, queue_size=10, latch=True)
        self.status_publisher = rospy.Publisher(rover + '/status', String, queue_size=10, latch=True)
        self.info_publisher = rospy.Publisher('/infoLog', String, queue_size=10, latch=True)

        self.mode_publisher = rospy.Publisher(rover + '/mode', UInt8, queue_size=1, latch=True)
        self.finger_publisher = rospy.Publisher(rover + '/fingerAngle/cmd', Float32, queue_size=1, latch=True)
        self.wrist_publisher = rospy.Publisher(rover + '/wristAngle/cmd', Float32, queue_size=1, latch=True)

        rospy.wait_for_service(rover + '/control')
        rospy.wait_for_service(rover + '/map/find_nearest_target')
        rospy.wait_for_service(rover + '/map/clear_target_map')

        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self.find_nearest_target = rospy.ServiceProxy(rover + '/map/find_nearest_target', FindTarget)
        self.clear_target_map = rospy.ServiceProxy(rover + '/map/clear_target_map', Empty)

    def __drive(self, request, **kwargs):
        request.obstacles = ~0
        if 'ignore' in kwargs :
            request.obstacles = ~kwargs['ignore']

        request.timeout = 120
        if 'timeout' in kwargs :
            request.timeout = kwargs['timeout']

        value = self.control([request])

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
            
    def circle(self):
        '''Drive in a small circle''' 
        if random.randint(0,1) == 0 :
            return self.timed_drive(15, 0.1, 0.62)
        else:
            return self.timed_drive(15, 0.1, -0.62)

    def timed_drive(self, time, linear, angular, **kwargs):
        '''Send the specified velocity command for a given period of time.'''
        req = MoveRequest(
            timer=time, 
            linear=linear, 
            angular=angular, 
        )
        return self.__drive(req, **kwargs)
    
    def drive(self, distance, **kwargs):
        '''Drive the specfied distance'''
        req = MoveRequest(
            r=distance, 
        )        
        return self.__drive(req, **kwargs)

    def turn(self, theta, **kwargs):
        '''Turn theta degrees'''
        req = MoveRequest(
            theta=theta, 
        )
        return self.__drive(req, **kwargs)

    def wait(self, time, **kwargs):
        '''Wait for a period of time. This can be used to check for obstacles'''
        req = MoveRequest(
            timer=time, 
        )

        return self.__drive(req, **kwargs)
                
    def set_wrist_angle(self, angle):
        '''Set the wrist angle to the specified angle'''
        self.wrist_publisher.publish(Float32(angle))

    def set_finger_angle(self, angle):
        '''Set the finger angle to the spedified angle'''
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
      swarmie.set_finger_angle(2) #open
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
      swarmie.wrist_up()          
