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

def approach():
    global swarmie 
    print ("Attempting a pickup.")
    try:
        swarmie.fingers_open()
        rospy.sleep(1)
        swarmie.wrist_down()
        
        try:
            block = swarmie.get_nearest_block_location()
            targets = [tag for tag in swarmie.get_latest_targets().detections if tag.id is 256 ]
        except tf.Exception as e:
            print("Something went wrong and we can't locate the block. ", e)
            swarmie.wrist_up()
            exit(1)

        if block is not None and targets is None:            
            # claw_offset should be a positive distance of how short drive_to needs to be.
            swarmie.drive_to(block, claw_offset = 0.2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR )
            # Grab - minimal pickup with sim_check.
            finger_close_angle = .5
            if swarmie.simulator_running():
                finger_close_angle = 0
            swarmie.set_finger_angle(finger_close_angle) #close
            rospy.sleep(1)
            swarmie.wrist_up()
            # did we succesuflly grab a block?
            if swarmie.has_block():
                swarmie.wrist_middle()
                return True
        else:
            print("No legal blocks detected.")
            swarmie.wrist_up()
            exit(1)
    except rospy.ServiceException as e:
        print ("There doesn't seem to be any blocks on the map. ", e)
        swarmie.wrist_up()
        exit(1)

    # otherwise reset claw and return Falase
    swarmie.wrist_up()
    return False

def recover():
    global swarmie 
    print ("Missed, trying to recover.")
    
    try:
        swarmie.drive(-1)
        #swarmie.turn(math.pi/2)
        #swarmie.turn(-math.pi)
        #swarmie.turn(math.pi/2)
    except: 
        # Hopefully this means we saw something.
        pass

def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)       

    print ('Waiting for camera/base_link tf to become available.')
    swarmie.xform.waitForTransform(rovername + '/base_link', rovername + '/camera_link', rospy.Time(), rospy.Duration(10))

    for i in range(3): 
        if approach():
            print ("Got it!")
            exit(0)        
        recover()
        
    print ("Giving up after too many attempts.")
    exit(1)

if __name__ == '__main__' : 
    main()

