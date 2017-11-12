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
    block = swarmie.find_nearest_target() 

    # FIXME: Why do I have to time shift the TF. This is a problem.
    block.result.header.stamp.secs = block.result.header.stamp.secs  + 1 
    tlist.waitForTransform(rovername + '/base_link', 
                            block.result.header.frame_id, block.result.header.stamp, 
                            rospy.Duration(3))
    rel_block = tlist.transformPoint(rovername + '/base_link', block.result)

    r = math.hypot(rel_block.point.x, rel_block.point.y)
    theta = angles.shortest_angular_distance(math.pi/2, math.acos(rel_block.point.x));

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

    except tf.TransformException as e: 
        print ("Transform failed: ", e)

    swarmie.fingers_open()
    swarmie.wrist_middle()
    return False

def recover():
    global swarmie 
    print ("Missed, trying to recover.")
    swarmie.clear_target_map()
    
    try :
        swarmie.drive(-0.5);
        swarmie.turn(math.pi/2)
        swarmie.turn(-math.pi)
        swarmie.turn(math.pi/2)
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

    for i in range(3) : 
        if approach() :
            print ("Got it!")
            exit(0)        
        recover()
        
    print ("Giving up after too many attempts.")
    exit(1)

if __name__ == '__main__' : 
    main()

