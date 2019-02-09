#! /usr/bin/env python 

from __future__ import print_function

import rospy 

from std_msgs.msg import String, UInt8

def main() :     
    rospy.init_node('diagnostics')

    
    diags_pub = rospy.Publisher('diagnostics', String, queue_size=2, latch=True)
    
    # Subscribers 
    rospy.Subscriber('mode', UInt8, mode)
    
    r = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        r.sleep()


