#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from std_msgs.msg import String

from mobility.msg import MoveResult
from swarmie_msgs.msg import Obstacle

import mobility.swarmie 

def main():
	rover = sys.argv[1]

	print("rospy.get_name()", rospy.get_name())

	swarmie = Swarmie(rover)
	print ('Connected.')
	swarmie.putdown()
	exit(0)
	
if __name__ == '__main__' : 
	main()
