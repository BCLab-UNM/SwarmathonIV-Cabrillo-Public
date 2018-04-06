#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie

def look_for_tags():
    '''Looks pi/6 in either direction, then oreient to corner if it exisits or to area with the most amount of home tags '''
    global swarmie
    start_heading = swarmie.get_odom_location().get_pose().theta
    turnTheta = math.pi/6
    targets = []
    swarmie.set_heading(start_heading + turnTheta, ignore=-1)
    rospy.sleep(.4)
    swarmie.set_heading(start_heading - turnTheta, ignore=-1)
    rospy.sleep(.4)
    targets = get_center_pose_list(256) # this should be everything seen in the sweep
    if (len(targets) == 0):
        raise IndexError("There are no home tags seen!") 
    return(targets)


def convert_to_Pose2D(t):
    global swarmie
    swarmie.xform.waitForTransform(swarmie.rover_name + '/odom', t.pose.header.frame_id, t.pose.header.stamp, rospy.Duration(5.0))
    odom_pose = swarmie.xform.transformPose(swarmie.rover_name + '/odom', t.pose)
    quat = [odom_pose.pose.orientation.x, odom_pose.pose.orientation.y,
            odom_pose.pose.orientation.z, odom_pose.pose.orientation.w,
            ]
    (_r, _p, y) = tf.transformations.euler_from_quaternion(quat)
    pose = Pose2D()
    pose.x = odom_pose.pose.position.x
    pose.y = odom_pose.pose.position.y
    pose.theta = y
    return(pose)


def get_center_pose_list(id):
    return [convert_to_Pose2D(tag) for tag in swarmie.get_targets_buffer().detections if tag.id is id ]
 
      
def get_furthest_side_hometags_location(tags):
        #seperate by the y value so left and right side 
        loc = swarmie.get_odom_location().get_pose()
        #print([t.y for t in tags])
        homeTags1 = [t for t in tags if t.y-loc.y > 0 ]
        homeTags2 = [t for t in tags if t.y-loc.y < 0 ]
        if (len(homeTags1) == 0): #if there are no tags on the left? side
            l = sorted(homeTags2, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.y)**2))
            homeTag1 = l[0]
            homeTag2 = l[-1]
        elif (len(homeTags2) == 0): #if there are no tags on the right? side
            l = sorted(homeTags1, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.y)**2))
            homeTag1 = l[0]
            homeTag2 = l[-1]
        else:
            homeTag1 = sorted(homeTags1, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.y)**2))[0]          #reverse=True
            homeTag2 = sorted(homeTags2, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.y)**2))[0]          #reverse=True
        return(homeTag1,homeTag2)       


def theta_int(theta):
    ''' same math needed in multiple functions to clean up the theta to make it easier to compair '''
    return(abs(round(theta/math.pi*2,0)))


def mid_point(t1, t2):
    pose = Pose2D()
    pose.x = (t1.x + t2.x) /2 #right now just averaging the value
    pose.y = (t1.y + t2.y) /2
    pose.theta = (t1.theta + t2.theta) /2
    return(pose)


def find_center(tags):
    global swarmie
    swarmie.set_heading(tags[0].theta+math.pi/2, ignore=-1) #if on the side this orients to the center of home
    rospy.sleep(.3)
    tags = get_center_pose_list(256) #just incase we now see a corner after squaring up
    if (len(tags) == 0):
        swarmie.print_infoLog(swarmie.rover_name + "There was a home tag here, but now its gone")
        print("There was a home tag here, but now its gone")
        raise IndexError("There are no home tags seen!") 
    swarmie.print_infoLog(swarmie.rover_name + "dropping off on side:" + str(theta_int(tags[0].theta)))
    print(swarmie.rover_name, " dropping off on side:", theta_int(tags[0].theta))
    return(mid_point(*get_furthest_side_hometags_location(tags))) #this will return the middle of the 2 furthest tags on one side


def main():
    '''Dropoff throws IndexError when no tags near swarmie '''
    global swarmie 
    
    if len(sys.argv) < 2:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    swarmie = Swarmie(sys.argv[1])

    rospy.sleep(.5)
    
    try:
        swarmie.targets_timeout = 0.1
        rospy.sleep(0.2)
        swarmie.targets_timeout = 9 # so they stay around for the decision making, should time it and reduce this
        tags = look_for_tags()
        if(swarmie.simulator_running()):
            swarmie.drive_to(find_center(tags), ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
        else:
            swarmie.drive_to(find_center(tags), claw_offset = 0.15, ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
        swarmie.targets_timeout = 3 #put it back
    except:
        print("Somthing broke")
        swarmie.targets_timeout = 3 #make sure it makes it back
        raise
    
    try:
        import os
        import random

        from gazebo_msgs.srv import GetModelState

        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(swarmie.rover_name, 'world')
        offset = .23
        x = resp_coordinates.pose.position.x
        y = resp_coordinates.pose.position.y
        
        #this is a total hack, it is possible in matlab and in python just have to spend more time
        os.system('rosrun gazebo_ros spawn_model -file /home/robot/rover_workspace/object.urdf -urdf'
        +' -x '+ str(x - math.copysign(offset, x)) 
        +' -y '+ str(y) #str(y - math.copysign(offset, y))
        +' -z 0 -model c' #on the ground
        +str(random.randint(1,99999))) #to avoid name collisions
        
        swarmie.drive(-.45, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    except: 
        swarmie.drive(-.45, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR) #make sure to get out of home
        raise

if __name__ == '__main__' :
    main()

