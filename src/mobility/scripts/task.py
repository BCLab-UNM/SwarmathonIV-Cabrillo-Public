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
import mobility.behavior.calibrate_imu
import mobility.behavior.queue
import mobility.behavior.search 
import mobility.behavior.pickup
import mobility.behavior.gohome
import mobility.behavior.dropoff
import mobility.behavior.escape_home

from mobility.swarmie import swarmie, AbortException, InsideHomeException

'''Node that coordinates the overall robot task''' 

class Task : 
    
    STATE_IDLE        = 0
    STATE_CAL_IMU     = 1
    STATE_QUEUE       = 2
    STATE_INIT        = 3
    STATE_SEARCH      = 4
    STATE_PICKUP      = 5
    STATE_GOHOME      = 6
    STATE_DROPOFF     = 7
    STATE_ESCAPE_HOME = 8

    PROG_INIT        = 'init.py'
    PROG_CAL_IMU     = 'calibrate_imu.py'
    PROG_QUEUE       = 'queue.py'
    PROG_SEARCH      = 'search.py'
    PROG_PICKUP      = 'pickup.py'
    PROG_GOHOME      = 'gohome.py'
    PROG_DROPOFF     = 'dropoff.py'
    PROG_ESCAPE_HOME = 'escape_home.py'

    def __init__(self):
        self.current_state = rospy.get_param('~task_state', Task.STATE_IDLE)
        self.prev_state = None
        self.has_block = rospy.get_param('~has_block', False)
        self.state_publisher = rospy.Publisher('/infoLog', String, queue_size=2, latch=False)
        self.status_pub = rospy.Publisher('status', String, queue_size=1, latch=True)
        rospy.on_shutdown(self.save_state)

    def save_state(self):
        rospy.set_param('~task_state', self.current_state)
        rospy.set_param('~has_block', self.has_block)
                        
    def print_state(self, msg):
        s = String()
        s.data = msg 
        self.state_publisher.publish(s)
        print (msg) 
        
    def launch(self, prog):
        try: 
            rval = prog(has_block=self.has_block)
            if rval is None:
                rval = 0
        except SystemExit as e: 
            rval = e.code
        return rval
    
    def get_task(self) :
        if self.current_state == Task.STATE_IDLE : 
            return "idle"
        elif self.current_state == Task.STATE_CAL_IMU :
            return "cal IMU"
        elif self.current_state == Task.STATE_QUEUE :
            return "queue"
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

        try:
            if self.current_state == Task.STATE_IDLE:
                self.current_state = Task.STATE_CAL_IMU

            elif self.current_state == Task.STATE_CAL_IMU :
                if self.launch(mobility.behavior.calibrate_imu.main) == 0:
                    self.print_state('IMU is calibrated. Entering start queue.')
                    self.current_state = Task.STATE_QUEUE
                else:
                    # TODO: Can the 2D calibration behavior fail? ServiceException's for example?
                    self.print_state('IMU calibration failed!')

            elif self.current_state == Task.STATE_QUEUE:
                if self.launch(mobility.behavior.queue.main) == 0:
                    self.print_state('Finished queuing. Starting init.')
                    self.current_state = Task.STATE_INIT
                else:
                    # TODO: Can the queue behavior fail?
                    self.print_state('Queue failed!')

            elif self.current_state == Task.STATE_INIT:
                if self.launch(mobility.behavior.init.main) == 0:
                    self.print_state('Init succeeded. Starting search.')
                    self.current_state = Task.STATE_SEARCH
                else:
                    # FIXME: What should happen when init fails?
                    self.print_state('Init failed! FIXME!.')

            elif self.current_state == Task.STATE_SEARCH:
                if self.launch(mobility.behavior.search.main) == 0:
                    self.print_state('Search succeeded. Do pickup.')
                    self.current_state = Task.STATE_PICKUP
                else:
                    self.print_state('Search failed. Going back home.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_PICKUP:
                if self.launch(mobility.behavior.pickup.main) == 0:
                    self.print_state('Pickup success. Going back home.')
                    self.has_block = True
                    self.current_state = Task.STATE_GOHOME
                else:
                    self.print_state('Pickup failed. Back to search.')
                    self.current_state = Task.STATE_SEARCH

            elif self.current_state == Task.STATE_GOHOME:
                gohome_status = self.launch(mobility.behavior.gohome.main)
                if gohome_status == 0:
                    if self.has_block:
                        self.print_state('Home found and I have a block. Do drop off.')
                        self.current_state = Task.STATE_DROPOFF
                    else:
                        self.print_state('Recalibrated home. Back to searching.')
                        self.current_state = Task.STATE_SEARCH
                elif gohome_status == 1:
                    self.print_state('Go Home interrupted, I found a tag. Do pickup.')
                    self.current_state = Task.STATE_PICKUP

                else:
                    # FIXME: What happens when we don't find home?
                    self.print_state('Home NOT found. Try again.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_DROPOFF:
                if self.launch(mobility.behavior.dropoff.main) == 0:
                    self.print_state('Dropoff complete. Back to searching.')
                    self.has_block = False
                    self.current_state = Task.STATE_SEARCH
                else:
                    self.print_state('Dropoff failed. Back to searching for home.')
                    self.current_state = Task.STATE_GOHOME

            elif self.current_state == Task.STATE_ESCAPE_HOME:
                self.print_state("EMERGENCY: I'm in the home ring, escape!")
                escape_status = self.launch(mobility.behavior.escape_home.main)
                if escape_status == 0 :
                    self.print_state('Escape is complete.')

                    if self.prev_state == Task.STATE_INIT:
                        self.current_state = Task.STATE_INIT
                    elif self.has_block:
                        self.current_state = Task.STATE_GOHOME
                    else:
                        self.current_state = Task.STATE_SEARCH

                    self.prev_state = None
                else:
                    # FIXME: What should be done here?
                    self.print_state('EMERGENCY: Bad to worse escape reports failure. Searching...')
                    sys.exit(-1)

        except AbortException as e:
            self.print_state('STOP! Entering manual mode.')
            sys.exit(0)

        except InsideHomeException:
            self.prev_state = self.current_state
            self.current_state = Task.STATE_ESCAPE_HOME

        except Exception as e:
            # FIXME: What do we do with bugs in task code?
            print('Task caught unknown exception:\n' + traceback.format_exc())
            sys.exit(-2)

def main() :
    swarmie.start(node_name='task')
    taskman = Task() 
    while not rospy.is_shutdown():
        taskman.run_next() 

if __name__ == '__main__' : 
    main()
