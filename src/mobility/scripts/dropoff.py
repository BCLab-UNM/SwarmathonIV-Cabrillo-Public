#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 

from std_msgs.msg import String

from mobility.msg import MoveResult, Obstacle
from mobility.swarmie import Swarmie

import mobility.swarmie 

def main():
	if len(sys.argv) < 2 :
		robolist = []
		for topic in rospy.get_published_topics():
			if topic[1] == 'sensor_msgs/Imu':
				robolist.append(topic[0].split('/')[1])
		robolist=list(set(robolist))
		if len(robolist) < 1:
			print('\033[91m',"No Rovers Detected",'\033[0m')
			print ('usage:', sys.argv[0], '<rovername>')
			exit (-1)
		else: 
			rover = robolist[0] #in the future view subscribed topics and to pick a different rover
			print("Detected rovers", robolist)
			print('\033[92m',"Auto selected:",rover,'\033[0m')
	else: 
		rover = sys.argv[1]

	print("rospy.get_name()", rospy.get_name())

	swarmie = Swarmie(rover)
	print ('Connected.')
	swarmie.putdown()
	
if __name__ == '__main__' : 
	main()
