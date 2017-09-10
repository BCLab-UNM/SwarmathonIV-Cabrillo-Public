#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy 
import StringIO 
import roslaunch

from std_msgs.msg import UInt8, String

from mobility.srv import Core
from mobility.msg import MoveRequest 

'''Node that coordinates the overall robot task''' 

def print_state(msg):
    global state_publisher
    s = String()
    s.data = msg 
    state_publisher.publish(s)

def launch(prog):
    global launcher
    global rover
    global task
    print_state("Launching task: " + prog)
    node = roslaunch.core.Node('mobility', prog, args=rover)
    task = launcher.launch(node)
    while task.is_alive() and not rospy.is_shutdown() :
        rospy.sleep(1.0)
    
    print_state("Task exited with code: " +  str(task.exit_code))
    return task.exit_code 

def mode(msg) :
    global rover_mode, launcher, task
    rover_mode = msg.data
    if rover_mode == 1 and task is not None :   
        if task.is_alive() :  
            print_state("Forcibly killing current task.")
            task.stop()
        
def main() :
    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    global task 
    task = None 
        
    global rover
    rover = sys.argv[1]
    rospy.init_node(rover + '_TASK')

    global core_service 
    rospy.wait_for_service(rover + '/control')
    core_service = rospy.ServiceProxy(rover + '/control', Core)
        
    rospy.Subscriber(rover + '/mode', UInt8, mode)

    global launcher
    launcher = roslaunch.scriptapi.ROSLaunch()
    launcher.start()

    global state_publisher
    state_publisher = rospy.Publisher('/infoLog', String, queue_size=5, latch=True)

    STATE_INIT     = 0 
    STATE_SEARCH   = 1 
    STATE_PICKUP   = 2 
    STATE_GOHOME   = 3 
    STATE_DROPOFF  = 4 
    
    PROG_INIT      = 'search.py'
    PROG_SEARCH    = 'search.py'
    PROG_PICKUP    = 'search.py'
    PROG_GOHOME    = 'search.py'
    PROG_DROPOFF   = 'search.py'
    
    CurrentState = STATE_INIT 
    
    global rover_mode
    rover_mode = 1 
    
    while not rospy.is_shutdown() : 
        if rover_mode == 1 :
            rospy.sleep(1.0)
            continue
        
        if CurrentState == STATE_INIT : 
            print_state ("Task: INIT")
            if launch(PROG_INIT) == 0 :
                CurrentState = STATE_SEARCH
            
        elif CurrentState == STATE_SEARCH : 
            print_state ("Task: SEARCH")
            if launch(PROG_SEARCH) == 0 :
                CurrentState = STATE_PICKUP
        
        elif CurrentState == STATE_PICKUP : 
            print_state ("Task: PICKUP")
            if launch(PROG_PICKUP) == 0 :
                CurrentState = STATE_GOHOME
            else:                
                CurrentState = STATE_SEARCH
        
        elif CurrentState == STATE_GOHOME : 
            print_state ("Task: GOHOME")
            if launch(PROG_GOHOME) == 0 : 
                CurrentState = STATE_DROPOFF
        
        elif CurrentState == STATE_DROPOFF : 
            print_state ("Task: DROPOFF")
            if launch(PROG_DROPOFF) == 0 :
                CurrentState = STATE_SEARCH
        
    launcher.stop()

if __name__ == '__main__' : 
    main()
