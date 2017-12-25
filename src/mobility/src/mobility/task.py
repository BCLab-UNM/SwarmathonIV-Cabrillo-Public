#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy 
import StringIO 
import roslaunch

from std_msgs.msg import UInt8, String

import threading 
task_lock = threading.Lock()

from mobility import sync

'''Node that coordinates the overall robot task''' 

class Task : 
    
    STATE_IDLE     = 0 
    STATE_INIT     = 1 
    STATE_SEARCH   = 2 
    STATE_PICKUP   = 3 
    STATE_GOHOME   = 4 
    STATE_DROPOFF  = 5 
    
    PROG_INIT      = 'init.py'
    PROG_SEARCH    = 'search.py'
    PROG_PICKUP    = 'pickup.py'
    PROG_GOHOME    = 'gohome.py'
    PROG_DROPOFF   = 'dropoff.py'

    def __init__(self, rover):
        self.task = None 
        self.rover = rover
        self.current_state = Task.STATE_IDLE 
        self.rover_mode = 1 
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
        
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=1, latch=False)

    @sync(task_lock)
    def set_mode(self, msg) :
        self.rover_mode = msg.data
                
    def print_state(self, msg):
        s = String()
        s.data = msg 
        self.state_publisher.publish(s)
        
    def launch(self, prog):
        self.print_state("Launching task: " + prog)
        node = roslaunch.core.Node('mobility', prog, args=self.rover)
        self.task = self.launcher.launch(node)

    @sync(task_lock)
    def run(self):
        if self.rover_mode == 1 :
            self.current_state = Task.STATE_IDLE 
            if self.task is not None and self.task.is_alive() : 
                self.print_state('Forcibly killing ' + self.task.name)
                self.task.stop()
            return 

        if self.task is not None and self.task.is_alive() :
            return 

        if self.task is not None :
            self.print_state('Task exited: ' + str(self.task.exit_code))
        
        if self.current_state == Task.STATE_IDLE : 
            self.launch(Task.PROG_INIT)
            self.current_state = Task.STATE_INIT 
            
        if self.current_state == Task.STATE_INIT : 
            if self.task.exit_code == 0 :
                self.launch(Task.PROG_SEARCH)
                self.current_State = Task.STATE_SEARCH
            else:
                # FIXME: What should happen when init fails? 
                self.launch(Task.PROG_INIT)
                            
        elif self.current_state == Task.STATE_SEARCH :
            if self.task.exit_code == 0 : 
                self.launch(Task.PROG_PICKUP)
                self.current_state = Task.STATE_PICKUP 
            else: 
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOGHOME 
            
        elif self.current_state == Task.STATE_PICKUP : 
            if self.task.exit_code == 0 :
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME 
            else:
                self.launch(Task.PROG_SEARCH)
                self.current_state = Task.STATE_SEARCH 
                
        elif self.current_state == Task.STATE_GOHOME : 
            if self.task.exit_code == 0 :
                self.launch(Task.PROG_DROPOFF)
                self.current_state = Task.STATE_DROPOFF
            else:
                # FIXME: What happens when we don't find home?
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME 
                        
        elif self.current_State == Task.STATE_DROPOFF : 
            self.launch(Task.PROG_SEARCH)
            self.current_state(Task.STATE_SEARCH)
