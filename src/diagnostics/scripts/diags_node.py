#! /usr/bin/env python 

from __future__ import print_function

import rospy 
import time 
import os 
import subprocess
import re 

from std_msgs.msg import String, Float32MultiArray, UInt8

from diagnostics.watcher import TopicWatcher
from __builtin__ import False

class RoverStatus:
    
    INITIALIZING = 0 
    READY        = 1 
    WARNING      = 2 
    FAILED       = 3

def is_failed(statuses):
    return TopicWatcher.FAILED_STATE in statuses

def is_warning(statuses):
    return TopicWatcher.WARNING_STATE in statuses

def is_init(statuses):
    return TopicWatcher.INIT_STATE in statuses

def all_ok(statuses):
    for stat in statuses:
        if stat != TopicWatcher.ACTIVE_STATE:
            return False
    return True 

def format_font(color, *msg):
    str = '<font color="' + color + '" size=2>'
    str += ' '.join(msg)
    str += "</font>"
    return str 

def ok(*msg):
    return format_font('Lime', *msg)

def warn(*msg):
    return format_font('Yellow', *msg)

def err(*msg):
    return format_font('Red', *msg)

def get_if_bytes(ifname):
    total = 0
    for fx in ["/sys/class/net/{}/statistics/tx_bytes".format(ifname), "/sys/class/net/{}/statistics/rx_bytes".format(ifname)]:
        with open(fx) as f:
            total += int(f.readline())
    return total 

def main() :
    rospy.init_node('diagnostics')
    rover_name = rospy.get_namespace()      
    rover_name = rover_name.strip('/')
    
    diags_log = rospy.Publisher('/diagsLog', String, queue_size=2, latch=True)
    diags_msg = rospy.Publisher('diagnostics', Float32MultiArray, queue_size=1)
    mode_pub = rospy.Publisher('mode', UInt8, queue_size=1, latch=True)
    
    update_rate = 1 # Hz
    r = rospy.Rate(update_rate) 

    # Wait for a clock message.
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now > 0:
            break

    topics = [
        "imu",
        "odom",
        "sonarLeft", "sonarRight", "sonarCenter",
        "targets"
        ]
    
    # FIXME: Need to sub the rover's wifi interface
    interface = 'eno1'
    last_if_bytes = get_if_bytes(interface)
    
    watchers = [TopicWatcher(t) for t in topics]

    status = RoverStatus.INITIALIZING

    last_wall = time.time()
    last_ros = rospy.get_time() 

    # Look for Gazebo
    is_simulator = False
    topics = rospy.get_published_topics()
    for t in topics:
        if t[0] == '/gazebo/link_states':
            is_simulator = True
   
    if is_simulator:
        diags_log.publish(ok("Simulated rover", rover_name, 'is initializing.'))
    else:
        diags_log.publish(ok("Physical rover", rover_name, 'is initializing.'))        

    while not rospy.is_shutdown():
        
        statuses = []
        
        for w in watchers:
            w.check()
            statuses.append(w._state)
            if w._state == TopicWatcher.WARNING_STATE:
                diags_log.publish(warn('WARNING:', rover_name, 'no messages on:', w._topic))
            elif w._state == TopicWatcher.FAILED_STATE:
                diags_log.publish(err('ERROR:', rover_name, 'topic failed:', w._topic))
            
        if status == RoverStatus.INITIALIZING:
            if is_failed(statuses):
                diags_log.publish(err("Rover", rover_name, 'has failed.'))
                status = RoverStatus.FAILED
            
            if all_ok(statuses):
                diags_log.publish(ok("Rover", rover_name, 'is ready.'))
                status = RoverStatus.READY
        
        elif status == RoverStatus.READY:
            if is_failed(statuses):
                diags_log.publish(err("Rover", rover_name, 'has failed.'))
                status = RoverStatus.FAILED
                break

        elif status == RoverStatus.WARNING:
            pass

        elif status == RoverStatus.FAILED:
            pass
        
        else:
            raise ValueError("Bad state!")

        if is_simulator:
            # Compute timeslip 
            curr_wall = time.time()
            curr_ros = rospy.get_time() 
            rate = (last_ros - curr_ros) / (last_wall - curr_wall)
            last_wall = curr_wall 
            last_ros = curr_ros 

            data = [0,0,rate]
        else:
            # Caluclate Bps 
            curr_if_bytes = get_if_bytes(interface)
            Bps = float(curr_if_bytes - last_if_bytes) / (1.0 / float(update_rate))
            last_if_bytes = curr_if_bytes

            # Link quality 
            link = 0
            try:
                # FIXME: Need an RE to extract the link quality. 
                iwconfig = subprocess.check_output(['iwconfig', interface], stderr=subprocess.DEVNULL)                
            except:
                link = 70
                
            data = [link,Bps,-1]
            
        diags_msg.publish(Float32MultiArray(data=data))
            
        r.sleep()

    msg = UInt8() 
    msg.data = 1
    mode_pub.publish(msg)
    diags_log.publish(err('Rover', rover_name, 'is shutting down!'))
    r.sleep()
    r.sleep()
    r.sleep()
        

if __name__ == '__main__':
    main()
    