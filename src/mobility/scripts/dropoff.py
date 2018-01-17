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
        targets = swarmie.get_latest_targets()
        if len(targets.detections) == 0 :
            raise IndexError("There are no tags seen!")
        seen_time = targets.detections[0].pose.header.stamp
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

    pose_list = []
    targets = swarmie.get_latest_targets()
    for t in targets.detections :
        if t.id == 256 :
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


def find_center():
    global swarmie

    tags = get_center_pose_list()
    
    # TODO: Figure out where center is from the tag poses. 
    
    print (tags)


def main():
    global swarmie
    global rovername

    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    wait_for_tag_transform()

    find_center()

    # swarmie.putdown()
    # swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    # swarmie.turn(math.pi, ignore=Obstacle.IS_SONAR)

    # Recalibrate the home location because we're here.
    odom_location = swarmie.get_odom_location()
    swarmie.set_home_odom_location(odom_location)


if __name__ == '__main__' :
    main()
