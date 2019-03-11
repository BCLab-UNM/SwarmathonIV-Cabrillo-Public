#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy
import tf
import math
import random 

import dynamic_reconfigure.client
from geometry_msgs.msg import Point
from swarmie_msgs.msg import Obstacle

from mobility.planner import Planner
from mobility.swarmie import swarmie, TagException, HomeException, ObstacleException, PathException, AbortException, MoveResult

'''Searcher node.''' 

def turnaround():
    #TODO: should this be ignoring TAG_TARGET's???
    swarmie.turn(
        random.gauss(math.pi/2, math.pi/4),
        ignore=Obstacle.IS_SONAR | Obstacle.VISION_SAFE
    )
    
def wander():
    try :
        rospy.loginfo("Wandering...")
        swarmie.turn(random.gauss(0, math.pi/6))
        swarmie.drive(random.gauss(2.5, 1))

        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround()


def search_exit(code):
    global planner, found_tag
    
    reset_speeds()
    
    if found_tag:
        print('Found a tag! Trying to get a little closer.')
        planner.face_nearest_block()
    
    if code == 0:
        swarmie.print_infoLog('Setting search exit poses.')
        set_search_exit_poses()
    sys.exit(code)


def reset_speeds():
    global initial_config, param_client
    param_client.update_configuration(initial_config)


def set_search_exit_poses():
    swarmie.set_search_exit_poses()

################################# Working area
def set_search_start_theta(theta=None):
        '''Remember the search start theta.'''
        if theta == None: 
            theta =  swarmie.get_odom_location().get_pose().theta
        rospy.set_param('search_start_theta', theta)
        
def get_search_start_theta():
        '''
        Will return invalid poses (containing all zeroes) if search exit
        location hasn't been set yet.

        Examples:
        >>> swarmie.set_heading(
        >>>     odom_pose.theta,
        >>>     ignore=Obstacle.TAG_HOME|Obstacle.TAG_TARGET|Obstacle.IS_SONAR
        >>> )
        '''
        theta = rospy.get_param('search_start_poses')
        return theta

def has_search_start_theta():
    '''Check to see if the search start location parameter is set.
    Returns:
    * (`bool`): True if the parameter exists, False otherwise.
    '''
    return rospy.has_param('search_start_poses')
#################################

def main(**kwargs):
    global planner, found_tag
    global initial_config, param_client

    found_tag = False
    SEARCH_SPEEDS = {
         'DRIVE_SPEED': 0.25,
         'TURN_SPEED': 0.7
    }

    planner = Planner()

    swarmie.fingers_open()
    swarmie.wrist_middle()

    # Change drive and turn speeds for this behavior, and register shutdown
    # hook to reset them at exit.
    if not rospy.has_param('search/speeds'):
        speeds = SEARCH_SPEEDS
        rospy.set_param('search/speeds', speeds)
    else:
        speeds = rospy.get_param('search/speeds',
                                 default=SEARCH_SPEEDS)

    param_client = dynamic_reconfigure.client.Client('mobility')
    config = param_client.get_configuration()
    initial_config = {
        'DRIVE_SPEED': config['DRIVE_SPEED'],
        'TURN_SPEED': config['TURN_SPEED']
    }
    param_client.update_configuration(speeds)

    if not planner.sees_home_tag():
        try:
            swarmie.drive(0.5, ignore=Obstacle.IS_SONAR)
        except HomeException:
            swarmie.turn(math.pi,
                         ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR)
        except TagException:
            rospy.sleep(0.3)  # build the buffer a little
            try:
                if swarmie.get_nearest_block_location() is not None:
                    found_tag = True
                    # print('Found a tag! Turning to face.')
                    # planner.face_nearest_block()
                    search_exit(0)  # found a tag?
            except tf.Exception:
                pass
    else:
        swarmie.turn(math.pi, ignore=Obstacle.VISION_SAFE | Obstacle.IS_SONAR)
        
    if has_search_start_theta():
        theta = swarmie.get_odom_location().get_pose().theta + (math.pi/10)
        set_search_start_theta(theta)
    else:
        set_search_start_theta()
        theta = swarmie.get_odom_location().get_pose().theta
    
    swarmie.set_heading( theta, ignore=Obstacle.VISION_HOME )
    try:
        swarmie.drive(50) #going for an exeption
    except TagException:
        rospy.sleep(0.3)  # build buffer a little
        # too risky to stop for targets if home is in view too
        if not planner.sees_home_tag():
            # success!
            found_tag = True
            # print('Found a tag! Turning to face.')
            # planner.face_nearest_block()
            search_exit(0) 
    except ObstacleException:
        print("Hit a will or Obstacle bouceing back to home")
        search_exit(1)# Must have hit a wall going back home
        
    print("Drive alot or crashed unexpectly")
    search_exit(1)

if __name__ == '__main__' : 
    swarmie.start(node_name='search')
    sys.exit(main())
