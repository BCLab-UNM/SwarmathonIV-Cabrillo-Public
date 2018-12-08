"""Put mobility related libraries here.

Any ROS module imports here will break namespace.py's ability to modify
namespaces.
"""
from __future__ import print_function

import threading 
import math 

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
