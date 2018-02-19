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
## TODO change quick_look_for_tags to return when faceing the area with the most tags and keep track if corner tags were seen then keep that orenation and return
## TODO test timeings of turning waiting after a turn if that will give significate more tag detections
## TODO general improvement of location of resource placement

def quick_look_for_tags():
    '''If I dont see a hometag I turn to both sides and if I still dont see a tag I drop the block and throw a ServiceException'''
    global swarmie
    global rovername
    
    targets = [tag for tag in swarmie.get_latest_targets().detections if tag.id is 256 ] #only get the home tags
    if len(targets) == 0:
        swarmie.turn(math.pi/8, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        targets = get_center_pose_list()
        if len(targets) == 0:
            swarmie.turn(-math.pi/4, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
            targets = get_center_pose_list()
            if len(targets) == 0:
                swarmie.putdown() 
                raise IndexError("There are no home tags seen!")
            else: #if tags were seen on the right side
                return;
        else: #if tags were seen on the left side
            return;          
    else: #if tags were seen ahead
            return;


def get_center_pose_list():
    global swarmie
    global rovername

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


def find_center():
    global swarmie
    
    tags = get_center_pose_list()
    #if 2+ corner tags have been seen, theta values seen 0,1,3
    if len(set( int(abs(t.theta)) for t in tags)) > 1:
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
        quick_look_for_tags()
        swarmie.drive_to(find_center(), ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
    except:
        swarmie.putdown() #best attempt to get it close to home or should I keep it and keep let search find home?
        swarmie.drive(-.3, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        raise

    # Recalibrate the home location because we're here.
    
    odom_location = swarmie.get_odom_location()
    swarmie.set_home_odom_location(odom_location)

    swarmie.putdown() 
    swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
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
