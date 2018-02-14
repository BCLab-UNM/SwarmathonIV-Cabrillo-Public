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
        
        # If I dont see a hometag I turn to both sides and if I still dont see a tag I drop the block and throw a ServiceException
        if len(targets) == 0:
            swarmie.turn(math.pi/8, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
            tags = get_center_pose_list()
            if len(targets) == 0:
                swarmie.turn(-math.pi/4, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
                tags = get_center_pose_list()
                if len(targets) == 0:
                    swarmie.putdown() 
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


def get_center_pose_list(): #changed so i can import and use with rdb swarmie
    global swarmie #this is changed  
    global rovername #put back after testing
    #rovername = 'achilles' #just for testing

    pose_list = []
    for t in [tag for tag in swarmie.get_latest_targets().detections if tag.id is 256 ] :
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
        pose_list.append(pose)
    return pose_list

def find_center():
    global swarmie
    tags = get_center_pose_list()
    print(tags)
    
    #if 2+ corner tags have been seen, theta values seen 0,1,3
    if len(set( int(abs(t.theta)) for t in tags)) > 1:
        #do a triangle and take the mid point of the hypotenuse
        pass
    else: #hopefuly pointing to the middel of home so just squareup and drive in
        pass
        #notes https://plot.ly/python/linear-fits/ 
        '''
        import matplotlib.pyplot as plt
        xs = [ t.x for t in dropoff.get_center_pose_list(swarmie) ]
        ys = [ t.y for t in dropoff.get_center_pose_list(swarmie) ]
        plt.scatter(xs, ys)
        plt.show()
        '''
    
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
