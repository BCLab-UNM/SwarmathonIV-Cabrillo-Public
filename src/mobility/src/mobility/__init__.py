# Put mobility related libraries here.
from __future__ import print_function

import threading 
import math 
import tf

from functools import wraps

from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Pose2D

def sync(lock):
    '''This decorator forces serial access based on a package level lock. Crude but effective.''' 
    def _sync(func) :
        @wraps(func)
        def wrapper(*args, **kwargs):
            with lock :
                return func(*args, **kwargs)
        return wrapper
    return _sync

class Location: 
    '''A class that encodes an EKF provided location and accessor methods''' 

    def __init__(self, odo):
        self.Odometry = odo 
    
    def get_pose(self):
        '''Return a std_msgs.Pose from this Location. Useful because Pose 
        has angles represented as roll, pitch, yaw.
        
        Returns:
        
            (std_msgs.Pose) The pose. 
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
            (boolean) True if within the target distance.

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
        
            (tuple): (vX, vY, vZ)
        '''
        return (self.Odometry.pose.covariance[0:15:7])
