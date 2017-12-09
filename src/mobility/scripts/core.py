#! /usr/bin/env python 

from __future__ import print_function

import sys
import rospy 

from mobility.driver import State

def heartbeat(event):
    self.heartbeat.publish("ok")

def run(event):
    global driver 
    driver.run() 
    
def main() :     
    global driver
    
    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    rover = sys.argv[1]
    rospy.init_node(rover + '_MOBILITY')

    # Start the driver code. 
    driver = State(rover)

    # Timers
    rospy.Timer(rospy.Duration(1), heartbeat)
    rospy.Timer(rospy.Duration(0.1), run)

    rospy.spin()

if __name__ == '__main__' : 
    main()
