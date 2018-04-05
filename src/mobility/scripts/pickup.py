#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 
import tf
import angles 

from std_msgs.msg import String
from geometry_msgs.msg import Point 

from mobility.msg import MoveResult
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Pickup node.''' 

def main():
    global swarmie 
    global rovername
    global claw_offset_distance
    
    if len(sys.argv) < 2:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    claw_offset_distance = 0.24 
    if(swarmie.simulator_running()):
        claw_offset_distance -= 0.02

    print ('Waiting for camera/base_link tf to become available.')
    swarmie.xform.waitForTransform(rovername + '/base_link', rovername + '/camera_link', rospy.Time(), rospy.Duration(10))

    try:
            block = swarmie.get_nearest_block_location()
    except tf.Exception as e:
            print("Something went wrong and we can't locate the block. ", e)
            exit(1)

    if block is not None:           
        if swarmie.simulator_running():
            swarmie.drive_to(block, claw_offset = claw_offset_distance, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR )
            for i in range(0,10):
                swarmie.set_wrist_angle(0)
                rospy.sleep(.4)
                swarmie.set_wrist_angle(.1)
                rospy.sleep(.4)
            
            exit(0)
    exit(1)
if __name__ == '__main__' : 
    main()

