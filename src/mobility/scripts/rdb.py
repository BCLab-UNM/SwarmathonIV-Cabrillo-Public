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
from swarmie_msgs.msg import Obstacle

from mobility import behavior

from mobility.swarmie import swarmie
from ctypes import CDLL, util

redraw = CDLL(util.find_library('readline')).rl_forced_update_display

msg_hist = {}

def logHandler(source, msg): 
    global msg_hist 
    update = False
    if source not in msg_hist or msg_hist[source] != msg.data :
        update = True 
    msg_hist[source] = msg.data
    if not quiet and update: 
        print (source, msg.data)
        redraw()
        
def toggle_output():
    global quiet 
    quiet = not quiet
    if quiet : 
        print ('Topic echo is OFF')
    else:
        print ('Topic echo is ON') 


if __name__ == '__main__' :
    global quiet 
    quiet = False   
    
    namespace = rospy.get_namespace()
    rover = namespace.strip('/')

    swarmie.start(tf_rover_name=rover, node_name='rdb')
    print ('Connected.')
    
    rospy.Subscriber('status', String, lambda msg : logHandler('/status:', msg))
    print ("Subscribed to", rospy.resolve_name('status'))

    rospy.Subscriber('/infoLog', String, lambda msg : logHandler('/infoLog:', msg))
    print ("Subscribed to /infoLog")
    
    rospy.Subscriber('state_machine', String, lambda msg : logHandler('/state_machine:', msg))
    print ("Subscribed to", rospy.resolve_name('state_machine'))

    print ('\033[31;1mTopic data will be displayed. Call toggle_output() to toggle.\033[0m')
    readline.parse_and_bind("tab: complete")
    
    print("Starting the enhanced Interactive Python shell")

    #Systems will have an unmet dependency run "sudo pip install ipython"
    try :
        from IPython import embed
        embed(user_ns=globals())
    except ImportError as e:
        print("Missing IPython run 'sudo pip install ipython'\n Failing over")
        try: 
            while True : 
                line = raw_input('>>> ')
                if line is not None and line != '' :
                    try :
                        exec (line)
                    except Exception as e :
                        print (e)
        except EOFError as e : 
            print ("Goodbye")

    print ("Qapla'!")

