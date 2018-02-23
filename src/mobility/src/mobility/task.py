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
        self.has_block = False
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
        
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)

    @sync(task_lock)
    def set_mode(self, msg) :
        self.rover_mode = msg.data
                
    def print_state(self, msg):
        s = String()
        s.data = msg 
        self.state_publisher.publish(s)
        
    def launch(self, prog):
        node = roslaunch.core.Node('mobility', prog, args=self.rover)
        self.task = self.launcher.launch(node)

    @sync(task_lock)
    def get_task(self) :
        if self.current_state == Task.STATE_IDLE : 
            return "idle"
        elif self.current_state == Task.STATE_INIT : 
            return "init"
        elif self.current_state == Task.STATE_SEARCH :
            return "search"
        elif self.current_state == Task.STATE_PICKUP : 
            return "pickup"
        elif self.current_state == Task.STATE_GOHOME : 
            return "gohome"
        elif self.current_state == Task.STATE_DROPOFF : 
            return "dropoff"        
        return "unknown"
        
    @sync(task_lock)
    def run(self):
        if self.rover_mode == 1 :
            self.current_state = Task.STATE_IDLE 
            if self.task is not None and self.task.is_alive() : 
                self.print_state('Forcibly killing ' + self.task.name)
                self.task.stop()
        
        elif self.task is not None and self.task.is_alive() :
            # Wait for the task to complete.
            pass 
        
        elif self.current_state == Task.STATE_IDLE : 
            self.launch(Task.PROG_INIT)
            self.current_state = Task.STATE_INIT 
            
        elif self.current_state == Task.STATE_INIT : 
            if self.task.exit_code == 0 :
                self.print_state('Init succeeded. Starting search.')
                self.launch(Task.PROG_SEARCH)
                self.current_state = Task.STATE_SEARCH
            else:
                # FIXME: What should happen when init fails? 
                self.print_state('Init failed! FIXME!.')
                self.launch(Task.PROG_INIT)
                            
        elif self.current_state == Task.STATE_SEARCH :
            if self.task.exit_code == 0 : 
                self.print_state('Search succeeded. Do pickup.')
                self.launch(Task.PROG_PICKUP)
                self.current_state = Task.STATE_PICKUP 
            else: 
                self.print_state('Search failed. Going back home.')
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME 
            
        elif self.current_state == Task.STATE_PICKUP : 
            if self.task.exit_code == 0 :
                self.print_state('Pickup success. Going back home.')
                self.has_block = True
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME 
            else:
                self.print_state('Pickup failed. Back to search.')
                self.launch(Task.PROG_SEARCH)
                self.current_state = Task.STATE_SEARCH 
                
        elif self.current_state == Task.STATE_GOHOME : 
            if self.task.exit_code == 0 :
                if self.has_block : 
                    self.print_state('Home found and I have a block. Do drop off.')
                    self.launch(Task.PROG_DROPOFF)
                    self.current_state = Task.STATE_DROPOFF
                else:
                    self.print_state('Recalibrated home. Back to searching.')
                    self.launch(Task.PROG_SEARCH)
                    self.current_state = Task.STATE_SEARCH
                    
            else:
                # FIXME: What happens when we don't find home?
                self.print_state('Home NOT found. Try again.')
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME 
                        
        elif self.current_state == Task.STATE_DROPOFF : 
            if self.task.exit_code == 0 :
                self.print_state('Dropoff complete. Back to searching.')
                self.has_block = False
                self.launch(Task.PROG_SEARCH)
                self.current_state = Task.STATE_SEARCH
            else:
                self.print_state('Dropoff failed. Back to searching for home.')
                self.launch(Task.PROG_GOHOME)
                self.current_state = Task.STATE_GOHOME
