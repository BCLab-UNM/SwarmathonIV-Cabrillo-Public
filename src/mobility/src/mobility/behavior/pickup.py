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

from mobility.swarmie import swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Pickup node.''' 

def approach():
    global claw_offset_distance
    print ("Attempting a pickup.")

    swarmie.fingers_open()
    rospy.sleep(1)
    swarmie.set_wrist_angle(1.15)

    try:
        block = swarmie.get_nearest_block_location()
    except tf.Exception as e:
        print("Something went wrong and we can't locate the block. ", e)
        swarmie.wrist_up()
        sys.exit(1)

    if block is not None:
        # claw_offset should be a positive distance of how short drive_to needs to be.
        if swarmie.simulator_running():
            swarmie.drive_to(
                block,
                claw_offset=0.1,
                ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR
            )
        else:
            swarmie.drive_to(
                block,
                claw_offset=claw_offset_distance,
                ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR
            )
        # Grab - minimal pickup with sim_check.

        if swarmie.simulator_running():
            finger_close_angle = 0
        else:
            finger_close_angle = 0.5

        swarmie.set_finger_angle(finger_close_angle) #close
        rospy.sleep(1)
        swarmie.wrist_up()
        rospy.sleep(.5)
        # did we succesuflly grab a block?
        if swarmie.has_block():
            swarmie.wrist_middle()
            swarmie.drive(-0.3,
                          ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR)
            return True
        else:
            swarmie.set_wrist_angle(0.55)
            rospy.sleep(1)
            swarmie.fingers_open()
            # Wait a moment for a block to fall out of claw
            rospy.sleep(0.25)
    else:
        print("No legal blocks detected.")
        swarmie.wrist_up()
        sys.exit(1)

    # otherwise reset claw and return Falase
    swarmie.wrist_up()
    return False

def recover():
    global claw_offset_distance
    claw_offset_distance -= 0.02
    print ("Missed, trying to recover.")
    try:
        swarmie.drive(-0.15,
                      ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR)
        # Wait a moment to detect tags before possible backing up further
        rospy.sleep(0.25)
        try:
            block = swarmie.get_nearest_block_location()
        except tf.Exception as e:
            print("Something went wrong recovering and we can't locate the block. ", e)
            swarmie.wrist_up()
            sys.exit(1)
        if block is not None:
            pass
        else:
            swarmie.drive(-0.15,
                          ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR)

        #swarmie.turn(math.pi/2)
        #swarmie.turn(-math.pi)
        #swarmie.turn(math.pi/2)
    except: 
        print("Oh no, we have an exception!")

def main(**kwargs):
    global claw_offset_distance
    
    claw_offset_distance = 0.24 
    if(swarmie.simulator_running()):
        claw_offset_distance -= 0.02

    print ('Waiting for camera/base_link tf to become available.')
    swarmie.xform.waitForTransform(swarmie.rover_name + '/base_link', swarmie.rover_name + '/camera_link', rospy.Time(), rospy.Duration(10))

    for i in range(3): 
        if approach():
            print ("Got it!")
            sys.exit(0)        
        recover()
        
    print ("Giving up after too many attempts.")
    return 1

if __name__ == '__main__' : 
    swarmie.start(node_name='pickup')
    sys.exit(main())
