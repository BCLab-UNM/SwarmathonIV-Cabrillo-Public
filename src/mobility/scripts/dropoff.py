#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie

## TODO for corner make a offset to move the block further inside, and/or change to rovername/camera_link from rovername/odom, and/or turn to be pi/4 from the seen tags
## TODO test timeings of turning waiting after a turn if that will give significate more tag detections
## TODO general improvement of location of resource placement

def look_for_tags():
    '''Looks pi/6 in either direction, then oreient to corner if it exisits or to area with the most amount of home tags '''
    global swarmie
    global rovername
    turnTheta = math.pi/6
    targets = []
    targets.append(get_center_pose_list(256))
    swarmie.set_heading(swarmie.get_odom_location().get_pose().theta + turnTheta, ignore=-1)
    rospy.sleep(.3)
    targets.append(get_center_pose_list(256))
    swarmie.set_heading((swarmie.get_odom_location().get_pose().theta) + (turnTheta*-2), ignore=-1)
    rospy.sleep(.3)
    targets.append(get_center_pose_list(256))
    print(len(targets[0]),":",len(targets[1]),":",len(targets[2]))
    if (sum([len(x) for x in targets]) is 0):
        raise IndexError("There are no home tags seen!")
    
    corner_indices = [x for x in range(len(targets)) if is_corner(targets[x])]
    print("corner_indices:",corner_indices)
    if (len(corner_indices) is 0): #if there are no corners the just pick the orentation with the most tags
    	corner_indices = [0,1,2]
    #make a tuple (number of tags,index) then find  the one with the most tags and then get its index
    directionIndex = max([(len(targets[i]),i) for i in corner_indices])[1]
    print("directionIndex:",directionIndex)
    if (directionIndex in [0,1]):
        swarmie.set_heading(swarmie.get_odom_location().get_pose().theta + (directionIndex+1)*turnTheta, ignore=-1)
    	#swarmie.turn((directionIndex+1)*turnTheta, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    	rospy.sleep(.3)
    targets = get_center_pose_list(256) #just incase the tags moved or I have not turned exactly back
    if (len(targets) is 0): #this is just bad but could happen if overshot the turn
        raise IndexError("There are no home tags seen!")
    return(targets)


def convert_to_Pose2D(t):
    global swarmie
    global rovername
    swarmie.xform.waitForTransform(swarmie.rover_name + '/odom', t.pose.header.frame_id, t.pose.header.stamp, rospy.Duration(1.0))
    odom_pose = swarmie.xform.transformPose(rovername + '/odom', t.pose)
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
    return [convert_to_Pose2D(tag) for tag in swarmie.get_latest_targets().detections if tag.id is id ]


def get_furthest_hometags_location(tags):
        '''should only be called if 2 different orenations(thetas) of tags are seen '''
        global swarmie
        global rovername
        loc = swarmie.get_odom_location().get_pose()
        tagThetasSeen = list(set( int(abs(t.theta)) for t in tags))
        homeTags1 = [t for t in tags if int(abs(t.theta)) is tagThetasSeen[0] ]
        homeTags2 = [t for t in tags if int(abs(t.theta)) is tagThetasSeen[1] ]
        homeTag1 = sorted(homeTags1, key=lambda x : math.sqrt(x.x**2 + x.y**2))[0]          #reverse=True
        homeTag2 = sorted(homeTags2, key=lambda x : math.sqrt(x.x**2 + x.y**2))[0]          #reverse=True
        return(homeTag1,homeTag2)


def is_corner(tags):
    return(len(set( int(abs(t.theta)) for t in tags)) > 1)


def find_center(tags):
    global swarmie
    #if 2+ corner tags have been seen, theta values seen 0,1,3
    if is_corner(tags):
        print(rovername, "dropping off in Corner:", set(int(abs(t.theta)) for t in tags))
        t1, t2 = get_furthest_hometags_location(tags)
        pose = Pose2D()
        pose.x = (t1.x + t2.x) /2 #right now just averaging the value
        pose.y = (t1.y + t2.y) /2
        pose.theta = (t1.theta + t2.theta) /2
        return(pose)
    else: #hopefuly pointing to the middle of home so just squareup? and drive in
        print(rovername, "dropping off on side:", tags[0].theta)
        return(tags[0])


def main():
    '''Dropoff throws IndexError when no tags near swarmie '''
    global swarmie
    global rovername

    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    
    #move wrist down but not so down it hits the ground
    swarmie.set_wrist_angle(.3)
    rospy.sleep(.5)
    
    try:
        tags = look_for_tags()
        swarmie.drive_to(find_center(tags), claw_offset = 0.15, ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
    except:
        raise

    # Recalibrate the home location because we're here.
    swarmie.set_home_odom_location(swarmie.get_odom_location())
    swarmie.set_home_gps_location(swarmie.get_gps_location())

    #what happened to GPS? look at github

    swarmie.putdown() 
    swarmie.drive(-.5, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

if __name__ == '__main__' :
    main()
'''
        #old code from wait_for_tag_transform
        seen_time = targets[0].pose.header.stamp
        try:
            swarmie.xform.waitForTransform(rovername + '/odom', rovername + '/camera_link', seen_time, rospy.Duration(1))
            return
        except Exception as e:
            print ('Waiting for tf.')
    # Uh-oh. No transform after 10 seconds. There must be a problem
    # somewhere else in the rover.
    raise(e)

#notes https://plot.ly/python/linear-fits/ 
import dropoff
import matplotlib.pyplot as plt
x = [ t.x for t in dropoff.get_center_pose_list(swarmie,swarmie.rover_name)]
y = [ t.y for t in dropoff.get_center_pose_list(swarmie,swarmie.rover_name)]
x,y,theta = zip(*list(dropoff.get_center_pose_list(swarmie,swarmie.rover_name)))
x,y,theta = zip(iter(list(dropoff.get_center_pose_list(swarmie,swarmie.rover_name))))
plt.scatter(x,y)
plt.show()

plt.scatter(zip(*list(dropoff.get_center_pose_list(swarmie,swarmie.rover_name))))
#plt.show()

import matplotlib.pyplot as plt
xs = [1,2]
ys = [2,1]
sum(xs)/2
plt.scatter(xs,yw)
plt.show()
'''
