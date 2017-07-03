
from __future__ import print_function 

import sys 
import rospy

from mobility.srv import Core
from mobility.msg import MoveResult
from obstacle_detection.srv import DetectionMask 
from obstacle_detection.msg import Obstacle 
from std_msgs.msg import String

class Point: 
    def __init__(self):
        pass 
    
class Swarmie: 
    
    def __init__(self, rover):
        self.rover_name = rover 
        rospy.init_node(rover + '_CONTROLLER')

        self.state_machine = rospy.Publisher(rover + '/state_machine', String, queue_size=10, latch=True)

        rospy.wait_for_service(rover + '/control')
        rospy.wait_for_service(rover + '/obstacleMask')

        self.drive_to = rospy.ServiceProxy(rover + '/control', Core)
        self.set_obstacle_mask = rospy.ServiceProxy(rover + '/obstacleMask', DetectionMask)
    
    def drive(self, distance, theta):
        return self.drive_to(distance, theta, 0, True).result.result
    
    def wait(self, time):
        return self.drive_to(0, 0, time, True).result.result
    
    def backup(self, distance):
        return self.drive_to(-distance, 0, 0, True).result.result
    
    def set_obstacle_mask(self, mask):
        return self.set_obstacle_mask(mask)
    