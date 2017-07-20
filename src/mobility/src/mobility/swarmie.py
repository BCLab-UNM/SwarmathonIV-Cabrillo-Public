
from __future__ import print_function 

import sys 
import rospy
import math 

from mobility.srv import Core
from mobility.msg import MoveResult, MoveRequest
from obstacle_detection.srv import DetectionMask 
from obstacle_detection.msg import Obstacle 

from std_msgs.msg import UInt8, String, Float32
    
class Swarmie: 
    
    def __init__(self, rover):
        self.rover_name = rover 
        rospy.init_node(rover + '_CONTROLLER')

        self.state_machine = rospy.Publisher(rover + '/state_machine', String, queue_size=10, latch=True)
        self.mode_publisher = rospy.Publisher(rover + '/mode', UInt8, queue_size=1, latch=True)
        self.finger_publisher = rospy.Publisher(rover + '/fingerAngle/cmd', Float32, queue_size=1, latch=True)
        self.wrist_publisher = rospy.Publisher(rover + '/wristAngle/cmd', Float32, queue_size=1, latch=True)

        rospy.wait_for_service(rover + '/control')
        rospy.wait_for_service(rover + '/obstacleMask')

        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self.obstacleMask = rospy.ServiceProxy(rover + '/obstacleMask', DetectionMask)

    def stop(self):
        msg = UInt8() 
        msg.data = 1
        self.mode_publisher.publish(msg)
    
    def go(self):
        msg = UInt8() 
        msg.data = 2
        self.mode_publisher.publish(msg)
            
    def circle(self, edges=6, dist=1):
        cir = []
        for l in range(edges) :
            cir.append(MoveRequest(r=dist, theta=2*math.pi/edges, timer=0))
        return self.control(cir).result.result 

    def timed_drive(self, time, linear, angular):
        return self.control([MoveRequest(timer=time, linear=linear, angular=angular)])
    
    def drive(self, distance, theta):
        return self.control([MoveRequest(r=distance, theta=theta, timer=0)]).result.result
    
    def wait(self, time):
        return self.control([MoveRequest(timer=time, linear=0, angular=0)]).result.result
    
    def backup(self, distance):
        return self.control([MoveRequest(r=-distance, theta=0, timer=0)]).result.result
    
    def set_obstacles(self, mask):
        return self.obstacleMask(mask)

    def ignore_obstacles(self):
        return self.obstacleMask(0)

    def all_obstacles(self):
        return self.obstacleMask(Obstacle.IS_SONAR | Obstacle.IS_VISION)

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
    