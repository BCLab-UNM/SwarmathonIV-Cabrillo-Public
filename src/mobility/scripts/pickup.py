#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random 
import tf

from std_msgs.msg import String
import angles 

from mobility.msg import MoveResult
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Pickup node.''' 

def get_block_location():
    global tlist
    global rovername 
    
    while True : 
        
        # Find the nearest block
        blocks = swarmie.get_latest_targets()        
        blocks = sorted(blocks.detections.detections, key=lambda x : abs(x.pose.pose.position.x))
        nearest = blocks[0]

        try: 
            tlist.waitForTransform(rovername + '/base_link', 
                           nearest.pose.header.frame_id, nearest.pose.header.stamp, 
                           rospy.Duration(1.0))
        except tf.Exception as e : 
            print ('Fuck you.')
            continue
        
        nearest_rel = tlist.transformPose(rovername + '/base_link', nearest.pose)
        break
    
    r = math.hypot(nearest_rel.pose.position.x, nearest_rel.pose.position.y)
    theta = angles.shortest_angular_distance(math.pi/2, math.acos(nearest_rel.pose.position.x));

    return r, theta

def approach():
    global swarmie 
    print ("Attempting a pickup.")
    try :
        swarmie.fingers_open()
        swarmie.wrist_down()

        r, theta = get_block_location()
   
        # Drive to the block
        swarmie.turn(theta, ignore=Obstacle.IS_VISION)
        swarmie.drive(r, ignore=Obstacle.IS_VISION)

        # Grab
        swarmie.fingers_close()
        rospy.sleep(0.5)
        swarmie.wrist_up()
        rospy.sleep(1)

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
    global tlist
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)       
    tlist = tf.TransformListener() 

    print ('Waiting for camera/base_link tf to become available.')
    tlist.waitForTransform(rovername + '/base_link', rovername + '/camera_link', rospy.Time(), rospy.Duration(10))

    for i in range(3) : 
        if approach() :
            print ("Got it!")
            exit(0)        
        recover()
        
    print ("Giving up after too many attempts.")
    exit(1)

if __name__ == '__main__' : 
    main()

