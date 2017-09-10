#! /usr/bin/env python

from __future__ import print_function

import os
import sys
import math
import signal
import random 
import readline
import rlcompleter

import rospy 

from std_msgs.msg import String

from mobility.srv import Core
from mobility.msg import MoveResult

from obstacle_detection.msg import Obstacle 

from mobility.swarmie import Swarmie 
from ctypes import CDLL, util

redraw = CDLL(util.find_library('readline')).rl_forced_update_display

def logHandler(source, msg): 
    if not quiet : 
        print (source, msg.data)
        redraw()
        
def handle(signum, frame):
    global quiet 
    quiet = not quiet
    if quiet : 
        print ('Topic echo is OFF')
    else:
        print ('Topic echo is ON') 


if __name__ == '__main__' :
    global quiet 
    quiet = False   
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    rover = sys.argv[1]

    swarmie = Swarmie(rover)
    print ('Connected.')
    
    rospy.Subscriber(rover + '/status', String, lambda msg : logHandler('/status:', msg))
    print ("Subscribed to", rover + '/status')

    rospy.Subscriber('/infoLog', String, lambda msg : logHandler('/infoLog:', msg))
    print ("Subscribed to /infoLog")
    
    rospy.Subscriber(rover + '/state_machine', String, lambda msg : logHandler('/state_machine:', msg))
    print ("Subscribed to", rover + '/state_machine')

    signal.signal(signal.SIGQUIT, handle)    

    print ('Topic data will be displayed. Press CTRL-\ to hide output.')
    readline.parse_and_bind("tab: complete")
    
    try :
        while True : 
            line = raw_input('>>> ')
            if line is not None and line != '' :
                try :
                    exec (line)
                except Exception as e :
                    print (e)
    except EOFError as e : 
        print ("Goodbye")

