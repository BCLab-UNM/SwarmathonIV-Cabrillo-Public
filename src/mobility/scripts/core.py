#! /usr/bin/env python 

from __future__ import print_function

import rospy 
import roslaunch

from std_msgs.msg import String, UInt8

from mobility.driver import State

def heartbeat(event):
    global heartbeat_pub, status_pub
    heartbeat_pub.publish("ok")

def mode(msg):
    global rover_mode, driver
    rover_mode = msg.data 
    driver.set_mode(msg)

def main() :     
    global driver, heartbeat_pub, rover_mode, status_pub
    
    rover_mode = 0 
    
    rospy.init_node('mobility')

    # Start the driver code. 
    driver = State()
    
    heartbeat_pub = rospy.Publisher('mobility/heartbeat', String, queue_size=1, latch=True)
    status_pub = rospy.Publisher('swarmie_status', String, queue_size=1, latch=True)

    # Subscribers 
    rospy.Subscriber('mode', UInt8, mode)

    # Timers
    rospy.Timer(rospy.Duration(1), heartbeat)

    launcher = roslaunch.scriptapi.ROSLaunch()
    launcher.start()
    task = None 
    status_pub.publish("idle")
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if rover_mode > 1 :
            if task is None: 
                status_pub.publish("starting")
                node = roslaunch.core.Node('mobility', 'task.py', namespace=rospy.get_namespace())
                task = launcher.launch(node)
            else:
                if not task.is_alive() : 
                    task = None
        else :
            if task is not None and task.is_alive() :
                status_pub.publish("stopped")
                task.stop()
                task = None 
                
        driver.run() 
        r.sleep()

if __name__ == '__main__' : 
    main()
