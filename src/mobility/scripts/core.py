#! /usr/bin/env python 

from __future__ import print_function

import rospy 

from std_msgs.msg import String, UInt8

from mobility.driver import State

def heartbeat(event):
    global heartbeat_pub, status_pub
    heartbeat_pub.publish("ok")

def mode(msg):
    global driver 
    driver.set_mode(msg)
        
def main() :     
    global driver, status_pub, info_pub, heartbeat_pub
    
    rospy.init_node('mobility')

    # Start the driver code. 
    driver = State()
    
    heartbeat_pub = rospy.Publisher('mobility/heartbeat', String, queue_size=1, latch=True)
    status_pub = rospy.Publisher('status', String, queue_size=1, latch=True)
    info_pub = rospy.Publisher('/infoLog', String, queue_size=1, latch=True)

    # Subscribers 
    rospy.Subscriber('mode', UInt8, mode)

    # Timers
    rospy.Timer(rospy.Duration(1), heartbeat)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        driver.run() 
        r.sleep()

if __name__ == '__main__' : 
    main()
