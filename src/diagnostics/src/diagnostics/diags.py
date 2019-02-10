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
    
class Diagnostics:
    
    INITIALIZING = 0 
    READY        = 1 
    WARNING      = 2 
    FAILED       = 3

    def _is_failed(self, statuses):
        return TopicWatcher.FAILED_STATE in statuses
    
    def _is_warning(self, statuses):
        return TopicWatcher.WARNING_STATE in statuses
    
    def _is_init(self, statuses):
        return TopicWatcher.INIT_STATE in statuses
    
    def _all_ok(self, statuses):
        for stat in statuses:
            if stat != TopicWatcher.ACTIVE_STATE:
                return False
        return True 

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
        self.interface = 'eno1'
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
        for fx in ["/sys/class/net/{}/statistics/tx_bytes".format(self.interface), 
                   "/sys/class/net/{}/statistics/rx_bytes".format(self.interface)]:
            with open(fx) as f:
                total += int(f.readline())
        return total 
    
    def run(self):

        while not rospy.is_shutdown():
            statuses = []
            
            for w in self._watchers:
                w.check()
                statuses.append(w._state)
                if w._state == TopicWatcher.WARNING_STATE:
                    self._diags_log.publish(_warn('WARNING:', self._rover_name, 'no messages on:', w._topic))
                elif w._state == TopicWatcher.FAILED_STATE:
                    self._diags_log.publish(_err('ERROR:', self._rover_name, 'topic failed:', w._topic))
                
            if self._status == Diagnostics.INITIALIZING:
                if self._is_failed(statuses):
                    self._diags_log.publish(_err("Rover", self._rover_name, 'has failed.'))
                    self._status = Diagnostics.FAILED
                
                if self._all_ok(statuses):
                    self._diags_log.publish(_ok("Rover", self._rover_name, 'is ready.'))
                    self._status = Diagnostics.READY
            
            elif self._status == Diagnostics.READY:
                if self._is_failed(statuses):
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
                Bps = float(curr_if_bytes - last_if_bytes) / (1.0 / float(self._update_rate))
                self._last_if_bytes = curr_if_bytes
    
                # Link quality 
                link = 0
                try:
                    # FIXME: Need an RE to extract the link quality. 
                    iwconfig = subprocess.check_output(['iwconfig', self.interface], stderr=subprocess.DEVNULL)
                except:
                    link = 70
                    
                data = [link,Bps,-1]
                
            self._diags_msg.publish(Float32MultiArray(data=data))                
            self._r.sleep()
    
        msg = UInt8() 
        msg.data = 1
        self._mode_pub.publish(msg)
        self._diags_log.publish(err('Rover', self._rover_name, 'is shutting down!'))
        self._r.sleep()
        self._r.sleep()
        self._r.sleep()
