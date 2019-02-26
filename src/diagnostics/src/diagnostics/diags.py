"""
Diags package. 

This package contains the Diagnostics class, which  implements the rover diagnostics. 
"""

from __future__ import print_function

import rospy 
import time 
import os 
import subprocess
import re 

from std_msgs.msg import String, Float32MultiArray, UInt8

from diagnostics.watcher import TopicWatcher

def _ok(*msg):
    return _format_font('Lime', *msg)

def _warn(*msg):
    return _format_font('Yellow', *msg)

def _err(*msg):
    return _format_font('Red', *msg)

def _format_font(color, *msg):
    str = '<font color="' + color + '" size=2>'
    str += ' '.join(msg)
    str += "</font>"
    return str 
    
def _is_failed(statuses):
    return TopicWatcher.FAILED_STATUS in statuses

def _is_warning(statuses):
    return TopicWatcher.WARNING_STATUS in statuses

def _is_init(statuses):
    return TopicWatcher.INIT_STATUS in statuses

def _all_ok(statuses):
    for stat in statuses:
        if stat != TopicWatcher.ACTIVE_STATUS:
            return False
    return True 

class Diagnostics:
    """
    Diagnostics class. 
    
    Publish diagnostics on two topics: 
    
    /rovername/diagnostics - Numerical data for the GUI
    /diagsLog - Diagnostic messages.     
    """
    
    INITIALIZING = 0 
    READY        = 1 
    WARNING      = 2
    FAILED       = 3

    def __init__(self):
        self._rover_name = rospy.get_namespace()      
        self._rover_name = self._rover_name.strip('/')
        
        self._diags_log = rospy.Publisher('/diagsLog', String, queue_size=2, latch=True)
        self._diags_msg = rospy.Publisher('diagnostics', Float32MultiArray, queue_size=1)
        self._mode_pub = rospy.Publisher('mode', UInt8, queue_size=1, latch=True)
        
        self._update_rate = 1 # Hz
        self._r = rospy.Rate(self._update_rate) 
    
        self._status = Diagnostics.INITIALIZING
    
        # Look for Gazebo
        self._is_simulator = False
        topics = rospy.get_published_topics()
        for t in topics:
            if t[0] == '/gazebo/link_states':
                self._is_simulator = True
       
        if self._is_simulator:
            self._diags_log.publish(_ok("Simulated rover", self._rover_name, 'is initializing.'))
        else:
            self._diags_log.publish(_ok("Physical rover", self._rover_name, 'is initializing.'))        

        # FIXME: Need to sub the rover's wifi interface
        self.interface = 'wlp2s0'
        self.topics = [
                "imu",
                "odom/filtered",
                "sonarLeft", "sonarRight", "sonarCenter",
                "targets"
            ]
    
        self._watchers = [TopicWatcher(t) for t in self.topics]
        self._last_if_bytes = self._get_if_bytes()
        self._last_wall = time.time()
        self._last_ros = rospy.get_time() 
        
    def _get_if_bytes(self):
        total = 0
        try:
            for fx in ["/sys/class/net/{}/statistics/tx_bytes".format(self.interface), 
                       "/sys/class/net/{}/statistics/rx_bytes".format(self.interface)]:
                with open(fx) as f:
                    total += int(f.readline())
        except IOError as e:
            # This can happen in the simulator
            print ('Could not open interface', self.interface)
        
        return total 
    
    def run(self):
        """
        Execute the diagnostics logic. Returns if the rover has failed. 
        """
        
        # Loop until we ge the shutdown message. 
        while not rospy.is_shutdown():
            statuses = []

            # Update all of the topic watchers.             
            for w in self._watchers:
                stat = w.check()
                statuses.append(stat)
                if stat == TopicWatcher.WARNING_STATUS:
                    self._diags_log.publish(_warn(w.get_message()))
                elif stat == TopicWatcher.FAILED_STATUS:
                    self._diags_log.publish(_err(w.get_message()))

            if self._status == Diagnostics.INITIALIZING:
                if _is_failed(statuses):
                    self._diags_log.publish(_err("Rover", self._rover_name, 'has failed.'))
                    self._status = Diagnostics.FAILED
                
                if _all_ok(statuses):
                    self._diags_log.publish(_ok("Rover", self._rover_name, 'is ready.'))
                    self._status = Diagnostics.READY
            
            elif self._status == Diagnostics.READY:
                if _is_failed(statuses):
                    self._diags_log.publish(_err("Rover", self._rover_name, 'has failed.'))
                    self._status = Diagnostics.FAILED
                    break
    
            elif self._status == Diagnostics.WARNING:
                pass
    
            elif self._status == Diagnostics.FAILED:
                pass
            
            else:
                raise ValueError("Bad state!")
    
            if self._is_simulator:
                # Compute timeslip 
                curr_wall = time.time()
                curr_ros = rospy.get_time() 
                rate = (self._last_ros - curr_ros) / (self._last_wall - curr_wall)
                self._last_wall = curr_wall 
                self._last_ros = curr_ros 
    
                data = [0,0,rate]
            else:
                # Caluclate Bps 
                curr_if_bytes = self._get_if_bytes()
                Bps = float(curr_if_bytes - self._last_if_bytes) / (1.0 / float(self._update_rate))
                self._last_if_bytes = curr_if_bytes
    
                # Link quality 
                link = 0
                try:
                    iwconfig = subprocess.check_output(['iwconfig', self.interface])
                    m = re.search(r'Link Quality=(\d+)/70', iwconfig)
                    if m is not None:
                        link = int(m.group(1)) 
                except Exception as e:
                    link = 0
                    
                data = [link,Bps,-1]
                
            self._diags_msg.publish(Float32MultiArray(data=data))                
            self._r.sleep()

        # The run loop has exited. Stop the rover and shutdown.
        msg = UInt8() 
        msg.data = 1
        self._mode_pub.publish(msg)
        self._diags_log.publish(_err('Rover', self._rover_name, 'is shutting down!'))
        self._r.sleep()
        self._r.sleep()
        self._r.sleep()
