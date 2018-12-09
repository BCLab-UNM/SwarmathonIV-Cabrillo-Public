#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy 
import StringIO 
import traceback 

from std_msgs.msg import UInt8, String

import threading 
task_lock = threading.Lock()

from mobility import sync

# Behavior programs
import mobility.behavior.init
import mobility.behavior.search 
import mobility.behavior.pickup
import mobility.behavior.gohome
import mobility.behavior.dropoff
import mobility.behavior.escape_home

from mobility.swarmie import swarmie, AbortException, InsideHomeException

'''Node that coordinates the overall robot task''' 

class Task : 
    
    STATE_IDLE        = 0
    STATE_INIT        = 1
    STATE_SEARCH      = 2
    STATE_PICKUP      = 3
    STATE_GOHOME      = 4
    STATE_DROPOFF     = 5
    STATE_ESCAPE_HOME = 6
    
    PROG_INIT        = 'init.py'
    PROG_SEARCH      = 'search.py'
    PROG_PICKUP      = 'pickup.py'
    PROG_GOHOME      = 'gohome.py'
    PROG_DROPOFF     = 'dropoff.py'
    PROG_ESCAPE_HOME = 'escape_home.py'

    def __init__(self):
        self.current_state = Task.STATE_IDLE 
        self.has_block = False
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)
        self.status_pub = rospy.Publisher('status', String, queue_size=1, latch=True)
                        
    def print_state(self, msg):
        s = String()
        s.data = msg 
        self.state_publisher.publish(s)
        print (msg) 
        
    def launch(self, prog):
        try: 
            rval = prog(has_block=self.has_block)
        except SystemExit as e: 
            rval = e.code
        except AbortException as e:
            sys.exit(0)
        except InsideHomeException:
            # Momentarily interrupt the state machine's flow in order to get
            # out of the home ring.
            mobility.behavior.escape_home.main(has_block=self.has_block)
            rval = -99
        except Exception as e: 
            print ('Task caught unknown exception: ', e)
            traceback.print_exc()
            rval = -100
        return rval
    
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
        elif self.current_state == Task.STATE_ESCAPE_HOME :
            return "escape_home"
        return "unknown"
        
    @sync(task_lock)
    def run_next(self):
        self.status_pub.publish(self.get_task())
        if self.current_state == Task.STATE_IDLE : 
            self.current_state = Task.STATE_INIT 
            
        elif self.current_state == Task.STATE_INIT : 
            if self.launch(mobility.behavior.init.main) == 0:
                self.print_state('Init succeeded. Starting search.')
                self.current_state = Task.STATE_SEARCH
            else:
                # FIXME: What should happen when init fails? 
                self.print_state('Init failed! FIXME!.')
                            
        elif self.current_state == Task.STATE_SEARCH :
            if self.launch(mobility.behavior.search.main) == 0 : 
                self.print_state('Search succeeded. Do pickup.')
                self.current_state = Task.STATE_PICKUP 
            else: 
                self.print_state('Search failed. Going back home.')
                self.current_state = Task.STATE_GOHOME 
            
        elif self.current_state == Task.STATE_PICKUP : 
            if self.launch(mobility.behavior.pickup.main) == 0 :
                self.print_state('Pickup success. Going back home.')
                self.has_block = True
                self.current_state = Task.STATE_GOHOME 
            else:
                self.print_state('Pickup failed. Back to search.')
                self.current_state = Task.STATE_SEARCH 
                
        elif self.current_state == Task.STATE_GOHOME : 
            gohome_status = self.launch(mobility.behavior.gohome.main)
            if gohome_status == 0 :
                if self.has_block : 
                    self.print_state('Home found and I have a block. Do drop off.')
                    self.current_state = Task.STATE_DROPOFF
                else:
                    self.print_state('Recalibrated home. Back to searching.')
                    self.current_state = Task.STATE_SEARCH
            elif gohome_status == 1 :
                self.print_state('Go Home interrupted, I found a tag. Do pickup.')
                self.current_state = Task.STATE_PICKUP
                    
            else:
                # FIXME: What happens when we don't find home?
                self.print_state('Home NOT found. Try again.')
                self.current_state = Task.STATE_GOHOME 
                        
        elif self.current_state == Task.STATE_DROPOFF : 
            if self.launch(mobility.behavior.dropoff.main) == 0 :
                self.print_state('Dropoff complete. Back to searching.')
                self.has_block = False
                self.current_state = Task.STATE_SEARCH
            else:
                self.print_state('Dropoff failed. Back to searching for home.')
                self.current_state = Task.STATE_GOHOME

def main() :
    swarmie.start(node_name='task')
    taskman = Task() 
    while not rospy.is_shutdown():
        taskman.run_next() 

if __name__ == '__main__' : 
    main()
