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

def get_block_location():
    global rovername, swarmie 
    
        
    # Find the nearest block
    blocks = swarmie.get_latest_targets()        
    blocks = sorted(blocks.detections.detections, key=lambda x : abs(x.pose.pose.position.x))
    nearest = blocks[0]

    swarmie.xform.waitForTransform(rovername + '/odom', 
                    nearest.pose.header.frame_id, nearest.pose.header.stamp, 
                    rospy.Duration(3.0))
        
    point = swarmie.xform.transformPose(rovername + '/odom', nearest.pose).pose.position
    print ('Transform says that the block is at: ', point)
    return point


def approach():
    global swarmie 
    print ("Attempting a pickup.")
    try :
        swarmie.fingers_open()
        swarmie.wrist_down()


        # Drive to the block
        try: 
            block = get_block_location()
        except tf.Exception as e : 
            # Something went wrong and we can't locate the block.
            print(e)
            return False
            
        swarmie.drive_to(block, ignore=Obstacle.IS_VISION)
   
        # Grab
        swarmie.pickup()

        if swarmie.has_block() :
            swarmie.wrist_middle()
            return True        
                
    except rospy.ServiceException as e:
        print ("There doesn't seem to be any blocks on the map.", e)

    swarmie.fingers_open()
    swarmie.wrist_middle()
    return False

def recover():
    global swarmie 
    print ("Missed, trying to recover.")
    
    try :
        swarmie.drive(-0.5);
        #swarmie.turn(math.pi/2)
        #swarmie.turn(-math.pi)
        #swarmie.turn(math.pi/2)
    except: 
        # Hopefully this means we saw something.
        pass

def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)       

    print ('Waiting for camera/base_link tf to become available.')
    swarmie.xform.waitForTransform(rovername + '/base_link', rovername + '/camera_link', rospy.Time(), rospy.Duration(10))

    for i in range(3) : 
        if approach() :
            print ("Got it!")
            exit(0)        
        recover()
        
    print ("Giving up after too many attempts.")
    exit(1)

if __name__ == '__main__' : 
    main()

