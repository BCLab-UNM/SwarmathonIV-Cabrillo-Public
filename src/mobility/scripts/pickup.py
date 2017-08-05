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
from mobility.srv import FindTarget

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Pickup node.''' 

def main():
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
           
    tlist = tf.TransformListener() 
    try :
        block = swarmie.find_nearest_target() 

        tlist.waitForTransform(rovername + '/base_link', 
                               block.result.header.frame_id, block.result.header.stamp, 
                               rospy.Duration(3))
        rel_block = tlist.transformPoint(rovername + '/base_link', block.result)
    
        swarmie.wrist_down()
        swarmie.fingers_open()
        swarmie.drive(rel_block.point.x * 3.1, 
                      rel_block.point.x * 3.1 * math.sin(rel_block.point.y * 3.1), 
                      Obstacle.IS_SONAR | Obstacle.IS_VISION)
        swarmie.fingers_close()
        rospy.sleep(0.5)
        
        swarmie.wrist_up()
        rospy.sleep(1)
        
        try:
            swarmie.wait(1, Obstacle.IS_VISION | Obstacle.SONAR_LEFT | Obstacle.SONAR_RIGHT | Obstacle.SONAR_CENTER)
            exit(-1)
        except ObstacleException as e:
            pass
        
        swarmie.wrist_middle()
        exit(0)
        
    except rospy.ServiceException as e:
        print ("There doesn't seem to be any blocks on the map.")
        exit(-1)
    
    except tf.TransformException as e: 
        print ("Transform failed: ", e)
        
    exit(0)

if __name__ == '__main__' : 
    main()

