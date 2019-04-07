#! /usr/bin/env python 

from __future__ import print_function

import rospy 
import roslaunch

from std_msgs.msg import String, UInt8

from mobility.driver import State

def heartbeat(event):
    global heartbeat_pub, status_pub, task
    heartbeat_pub.publish("ok")
    if task is None:
        status_pub.publish("idle")

def mode(msg):
    global rover_mode, driver
    rover_mode = msg.data 
    driver.set_mode(msg)

def publish_status(msg):
    global status_pub, task_pub
    status_pub.publish(msg)
    task_pub.publish(msg)

def main() :     
    global driver, heartbeat_pub, rover_mode, status_pub, task_pub, task
    
    rover_mode = 0 
    
    rospy.init_node('mobility')

    # Start the driver code. 
    driver = State()
    
    heartbeat_pub = rospy.Publisher('mobility/heartbeat', String, queue_size=1, latch=True)
    # Published regularly on a timer.
    status_pub = rospy.Publisher('swarmie_status', String, queue_size=1, latch=True)
    # Published once when the status changes.
    task_pub = rospy.Publisher('task_state', String, queue_size=1, latch=True)

    # Subscribers 
    rospy.Subscriber('mode', UInt8, mode)

    task = None 
    
    # Timers
    rospy.Timer(rospy.Duration(1), heartbeat)

    launcher = roslaunch.scriptapi.ROSLaunch()
    launcher.start()
    publish_status("idle")
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if rover_mode > 1 :
            if task is None: 
                publish_status("starting")
                node = roslaunch.core.Node('mobility', 'task.py',
                                           name='task',
                                           namespace=rospy.get_namespace())
                task = launcher.launch(node)
            else:
                if not task.is_alive() : 
                    task = None
        else :
            if task is not None and task.is_alive() :
                publish_status("stopped")
                task.stop()
                task = None 
                
        driver.run() 
        r.sleep()

if __name__ == '__main__' : 
    main()
