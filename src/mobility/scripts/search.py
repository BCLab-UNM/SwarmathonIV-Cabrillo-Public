#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math

from mobility.srv import Core

'''Searcher node.''' 

def run(state):
    pass 

def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    global rover
    rover = sys.argv[1]
    rospy.init_node(rover + '_SEARCH')

    global core_service 
    rospy.wait_for_service(rover + '/control')
    core_service = rospy.ServiceProxy(rover + '/control', Core)

    core_service(15, math.pi/2, 0, True)

    return 0

if __name__ == '__main__' : 
    main()

