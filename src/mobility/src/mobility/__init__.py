# Put mobility related libraries here.
from __future__ import print_function

import threading 
import math 
import tf

from functools import wraps
from contextlib import contextmanager

from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Pose2D

_SingletonStateLock = threading.Lock()

@contextmanager
def synchronized():
    '''A context manager to mark a critical section that needs to hold the global lock'''
    global _SingletonStateLock
    try:
        _SingletonStateLock.acquire()
        yield
    finally:
        _SingletonStateLock.release()

def sync(func) :
    '''This decorator forces serial access based on a package level lock. Crude but effective.''' 
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        func.__doc__
        global _SingletonStateLock
        with synchronized() :
            return func(self, *args, **kwargs)
    return wrapper

class Location: 
    '''A class that encodes an EKF provided location and accessor methods''' 
    def __init__(self, odo):
        self.Odometry = odo 
    
    def get_pose(self):
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
        '''Determine if the pose is within acceptable distance of this location''' 
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x, 
                          goal.y - self.Odometry.pose.pose.position.y)
        return dist < distance
                    
    def get_variances(self):
        return (self.Odometry.pose.covariance[0:15:7])
