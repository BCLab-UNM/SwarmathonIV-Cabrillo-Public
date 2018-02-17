#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D
from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie


def wait_for_tag_transform():
    global swarmie
    global rovername

    # When the program first starts the tf buffer is empty
    # making it impossible to do transforms. This just waits
    # until tf says it can complete the transform on the latest
    # block detections.
    for _i in range(10):
        targets = [tag for tag in swarmie.get_latest_targets().detections if tag.id is 256 ] #only get the home tags
        if len(targets) == 0 :
            raise IndexError("There are no home tags seen!")
        seen_time = targets[0].pose.header.stamp
        try:
            swarmie.xform.waitForTransform(rovername + '/odom', rovername + '/camera_link', seen_time, rospy.Duration(1))
            return
        except Exception as e:
            print ('Waiting for tf.')
    # Uh-oh. No transform after 10 seconds. There must be a problem
    # somewhere else in the rover.
    raise(e)


def get_center_pose_list():
    global swarmie
    global rovername

    #[tag for tag in swarmie.get_latest_targets().detections if tag.id is 256 ]
    pose_list = []
    targets = swarmie.get_latest_targets()
    for t in targets.detections :
        if t.id == 256 :
            swarmie.xform.waitForTransform(swarmie.rover_name + '/odom', t.pose.header.frame_id, t.pose.header.stamp, rospy.Duration(1.0))
            odom_pose = swarmie.xform.transformPose(rovername + '/odom', t.pose)
            quat = [odom_pose.pose.orientation.x,
                odom_pose.pose.orientation.y,
                odom_pose.pose.orientation.z,
                odom_pose.pose.orientation.w,
            ]
            (_r, _p, y) = tf.transformations.euler_from_quaternion(quat)
            pose = Pose2D()
            pose.x = odom_pose.pose.position.x
            pose.y = odom_pose.pose.position.y
            pose.theta = y
            pose_list.append(pose)

    return pose_list

def sufficient_tags_seen(tags):
    ''' returns True if 2+ tags have a differnt orientation "theta", False otherwise '''
    global swarmie
    uniqTagTheta = round(abs(tags[0].theta),1) 
    for tag in tags: #yeah i'm going to iterate an extra time
        pass
        #print("Tag: x:",tag.x,"y:",tag.y,"theta:",round(abs(tag.theta),1))
        if uniqTagTheta != round(abs(tag.theta),1):
            return(True)
    return(False)

def find_center():
    global swarmie

    tags = get_center_pose_list()
    print(tags)
    if not sufficient_tags_seen(tags):
        pass #turn and if that fails drive around or just drop it inside?
        
    # TODO: Figure out where center is from the tag poses. 
    
    #print (tags)
    
    ''' START Temp drop just past if not on hometag'''
    if len(tags) > 0:
        return(tags[0])
    ''' END Temp drop ust past if not on hometag'''

def main():
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
    
    wait_for_tag_transform()
    swarmie.drive_to(find_center(), ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)

    # Recalibrate the home location because we're here.
    odom_location = swarmie.get_odom_location()
    swarmie.set_home_odom_location(odom_location)

    swarmie.putdown() 
    swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)


if __name__ == '__main__' :
    main()
