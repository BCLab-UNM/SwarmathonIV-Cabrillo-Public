#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 
import tf

from std_msgs.msg import String

from obstacle_detection.msg import Obstacle 
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Pickup node.''' 

def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
           
    tlist = tf.TransformListener() 
    block = swarmie.find_nearest_target() 
    rel_block = None

    while rel_block is None :    
        try: 
            tlist.waitForTransform(rovername + '/base_link', block.result.header.frame_id, rospy.Time(0), rospy.Duration(3))
            block.result.header.stamp = rospy.Time(0)
            rel_block = tlist.transformPoint(rovername + '/base_link', block.result)
        
            print ("Block is: ", block)
            print ("REL Block is: ", rel_block)
            
        except Exception as e: 
            print ("Fucked: ", e)

    swarmie.lower_wrist()
    swarmie.open_fingers()
    swarmie.drive(rel_block.point.x * 3, 
                  rel_block.point.x * 3 * math.sin(rel_block.point.y), 
                  Obstacle.IS_SONAR | Obstacle.IS_VISION)
    swarmie.close_fingers()
    rospy.sleep(0.5)
    swarmie.raise_wrist()    

    rospy.sleep(2)
    swarmie.open_fingers()
        
    return 0

if __name__ == '__main__' : 
    main()

