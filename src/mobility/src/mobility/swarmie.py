
from __future__ import print_function 

import sys 
import rospy
import math 
import random 

from mobility.srv import Core
from mobility.msg import MoveResult, MoveRequest
from obstacle_detection.msg import Obstacle 

from std_msgs.msg import UInt8, String, Float32

class DriveException(Exception):
    pass

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

class Swarmie: 
    
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

        self.control = rospy.ServiceProxy(rover + '/control', Core)

    def stop(self):
        msg = UInt8() 
        msg.data = 1
        self.mode_publisher.publish(msg)
    
    def go(self):
        msg = UInt8() 
        msg.data = 2
        self.mode_publisher.publish(msg)
            
    def circle(self, edges=6, dist=1):
        if random.randint(0,1) == 0 :
            return self.timed_drive(15, 0.1, 0.62)
        else:
            return self.timed_drive(15, 0.1, -0.62)

    def __check_raise(self, value):        
        if value == MoveResult.OBSTACLE_SONAR :
            raise ObstacleException()
        elif value == MoveResult.OBSTACLE_TAG : 
            raise TagException()
        elif value == MoveResult.OBSTACLE_HOME : 
            raise HomeException()
        elif value == MoveResult.PATH_FAIL : 
            raise PathException()
        elif value == MoveResult.USER_ABORT : 
            raise AbortException()
        else:
            return MoveResult.SUCCESS

    def timed_drive(self, time, linear, angular, ignore_obstacles = 0):
        return self.__check_raise(
            self.control([MoveRequest(timer=time, linear=linear, angular=angular, obstacles=~ignore_obstacles)])
            )
    
    def drive(self, distance, theta, ignore_obstacles = 0):
        return self.__check_raise(
            self.control([MoveRequest(r=distance, theta=theta, timer=0, obstacles=~ignore_obstacles)]).result.result
            )

    def wait(self, time, ignore_obstacles = 0):
        return self.__check_raise(
            self.control([MoveRequest(timer=time, linear=0, angular=0, obstacles=~ignore_obstacles)]).result.result
            )
        
    def backup(self, distance, ignore_obstacles = 0):
        return self.__check_raise(
            self.control([MoveRequest(r=-distance, theta=0, timer=0, obstacles=~ignore_obstacles)]).result.result
            )
        
    def set_wrist_angle(self, angle):
        self.wrist_publisher.publish(Float32(angle))

    def set_finger_angle(self, angle):
        self.finger_publisher.publish(Float32(angle))
        
    def lower_wrist(self):
        self.wrist_publisher.publish(Float32(1.25))
    
    def raise_wrist(self):
        self.wrist_publisher.publish(Float32(0.0))
    
    def open_fingers(self):
        self.finger_publisher.publish(Float32(math.pi/2))
    
    def close_fingers(self):
        self.finger_publisher.publish(Float32(0.0))
    
    def print_state_machine(self, msg):
        s = String()
        s.data = msg 
        self.sm_publisher.publish(s)

    def print_infoLog(self, msg):
        s = String()
        s.data = msg 
        self.info_publisher.publish(s)

    def print_status(self, msg):
        s = String()
        s.data = msg 
        self.status_publisher.publish(s)
