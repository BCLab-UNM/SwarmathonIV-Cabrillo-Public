#! /usr/bin/env python

from __future__ import print_function

import rospy 
import sys
import signal
import os
from std_msgs.msg import String
from mobility.srv import Command 
import readline
import rlcompleter

def logHandler(source, msg): 
    if not quiet : 
        print (source, msg.data)

def handle(signum, frame):
    global quiet 
    quiet = not quiet

def main() :
    global quiet 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    rover = sys.argv[1]
    rospy.init_node(rover + '_rdb')

    print ("Waiting to connect to rover.")
    rospy.wait_for_service(rover + '/cmd')

    quiet = False   
    rospy.Subscriber(rover + '/status', String, lambda msg : logHandler('status:', msg))
    rospy.Subscriber(rover + '/infoLog', String, lambda msg : logHandler('infoLog:', msg))
    rospy.Subscriber(rover + '/state_machine', String, lambda msg : logHandler('state_machine:', msg))

    signal.signal(signal.SIGQUIT, handle)    
    print ('Ready.')
    readline.parse_and_bind("tab: complete")
    
    try :
        cmd = rospy.ServiceProxy(rover + '/cmd', Command)
        try :
            while True : 
                line = raw_input('>>> ')
                if line is not None and line != '' :
                    ret = cmd(line)
                    print (ret.str, end='')    
        except EOFError as e : 
            print ("Goodbye")

        cmd.close()
    except Exception as e : 
        print ("Exception while using service:", e)
        
if __name__ == '__main__' :
    main()
