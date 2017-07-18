
from __future__ import print_function 

import sys 
import rospy
import math 

from mobility.srv import Core
from mobility.msg import MoveResult, MoveRequest
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

        self.control = rospy.ServiceProxy(rover + '/control', Core)
        self.obstacleMask = rospy.ServiceProxy(rover + '/obstacleMask', DetectionMask)

    def circle(self, edges=6, dist=1):
        cir = []
        for l in range(edges) :
            cir.append(MoveRequest(dist, 2*math.pi/edges, 0, False))
        return self.control(cir).result.result 
    
    def drive(self, distance, theta):
        return self.control([MoveRequest(distance, theta, 0, True)]).result.result
    
    def wait(self, time):
        return self.control([MoveRequest(0, 0, time, True)]).result.result
    
    def backup(self, distance):
        return self.control([MoveRequest(-distance, 0, 0, True)]).result.result
    
    def set_obstacles(self, mask):
        return self.obstacleMask(mask)

    def ignore_obstacles(self):
        return self.obstacleMask(0)

    def all_obstacles(self):
        return self.obstacleMask(Obstacle.IS_SONAR | Obstacle.IS_VISION)
    