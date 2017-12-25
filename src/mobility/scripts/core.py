#! /usr/bin/env python 

from __future__ import print_function

import sys
import rospy 

from std_msgs.msg import String, UInt8

from mobility.driver import State
from mobility.task import Task 

def heartbeat(event):
    global heartbeat_pub, status_pub    
    heartbeat_pub.publish("ok")
    status_pub.publish('online')

def mode(msg):
    global driver, task 
    driver.set_mode(msg)
    task.set_mode(msg) 
        
def main() :     
    global driver, task, status_pub, info_pub, heartbeat_pub
    
    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    rover = sys.argv[1]
    rospy.init_node(rover + '_MOBILITY')

    # Start the driver code. 
    driver = State(rover)
    task = Task(rover) 
    
    heartbeat_pub = rospy.Publisher(rover + '/mobility/heartbeat', String, queue_size=1, latch=True)
    status_pub = rospy.Publisher(rover + '/status', String, queue_size=1, latch=True)
    info_pub = rospy.Publisher('/infoLog', String, queue_size=1, latch=True)

    # Subscribers 
    rospy.Subscriber(rover + '/mode', UInt8, mode)

    # Timers
    rospy.Timer(rospy.Duration(1), heartbeat)

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        driver.run() 
        task.run()
        r.sleep()

if __name__ == '__main__' : 
    main()
