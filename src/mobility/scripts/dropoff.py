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
    '''
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
    '''


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
    
    '''
    global hometag_pose_from_swarmie
    swarmie_pose = swarmie.get_odom_location().get_pose()
    hometag_pose_from_swarmie = Pose2D()
    hometag_pose_from_swarmie.theta = y - swarmie_pose.theta
    hometag_pose_from_swarmie.x = odom_pose.pose.position.x - swarmie_pose.x
    hometag_pose_from_swarmie.y = odom_pose.pose.position.y - swarmie_pose.y
    print("pose:\n", pose)
    print("hometag_pose_from_swarmie:\n",hometag_pose_from_swarmie)
    '''
    #convert Pose2D to PoseStamped and publish topic for rviz
    ''' Notes
    From Darren
        set the frame_id to rovername + '/odom'
    Current searches
        https://answers.ros.org/question/231941/how-to-create-orientation-in-geometry_msgsposestamped-from-angle/
        https://answers.ros.org/question/41233/how-to-understand-robot-orientation-from-quaternion-yaw-angle/
    '''
    return(pose)

def get_center_pose_list(id):
    return [convert_to_Pose2D(tag) for tag in swarmie.get_latest_targets().detections if tag.id is id ]

def get_furthest_corner_hometags_location(tags):
        '''should only be called if 2 different orenations(thetas) of tags are seen '''
        global swarmie
        loc = swarmie.get_odom_location().get_pose()
        #tagThetasSeen = list(set(int(abs(t.theta)) for t in tags))
        tagThetasSeen = list(set(theta_int(t.theta) for t in tags)) 
        homeTags1 = [t for t in tags if theta_int(t.theta) is tagThetasSeen[0] ]
        homeTags2 = [t for t in tags if theta_int(t.theta) is tagThetasSeen[1] ]
        
        if (len(homeTags1) == 0):
            l = sorted(tags, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))
            homeTag1 = l[0]
            homeTag2 = l[-1]
        elif (len(homeTags2) == 0):
            l = sorted(tags, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2)) #reverse=True
            homeTag1 = l[0]
            homeTag2 = l[-1]
        else:
            homeTag1 = sorted(homeTags1, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))[0]          #reverse=True
            homeTag2 = sorted(homeTags2, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))[0]          #reverse=True
        return(homeTag1,homeTag2)
        
def get_furthest_side_hometags_location(tags):
        #seperate by the y value so left and right side 
        loc = swarmie.get_odom_location().get_pose()
        #print([t.y for t in tags])
        homeTags1 = [t for t in tags if t.y-loc.y > 0 ]
        homeTags2 = [t for t in tags if t.y-loc.y < 0 ]
        if (len(homeTags1) == 0): #if there are no tags on the left? side
            l = sorted(homeTags2, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))
            homeTag1 = l[0]
            homeTag2 = l[-1]
        elif (len(homeTags2) == 0): #if there are no tags on the right? side
            l = sorted(homeTags1, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))
            homeTag1 = l[0]
            homeTag2 = l[-1]
        else:
            homeTag1 = sorted(homeTags1, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))[0]          #reverse=True
            homeTag2 = sorted(homeTags2, key=lambda x : math.sqrt((x.x-loc.x)**2 + (x.y-loc.x)**2))[0]          #reverse=True
        return(homeTag1,homeTag2)       

def is_corner(tags):
    return (len(set(theta_int(t.theta) for t in tags)) > 1)

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
    #if 2+ corner tags have been seen, so differnt theta values
    if is_corner(tags):
        swarmie.print_infoLog(swarmie.rover_name + " dropping off in corner:" + str(set(theta_int(t.theta) for t in tags)))
        print(swarmie.rover_name, "dropping off in corner:", set(theta_int(t.theta) for t in tags))
        return(mid_point(*get_furthest_corner_hometags_location(tags)))
    else: #only see one tag of given orntaion squareup with the tag 
        swarmie.set_heading(tags[0].theta+math.pi/2, ignore=-1) 
        #do a second check for the corner 
        rospy.sleep(.3)
        tags = get_center_pose_list(256) #just incase we now see a corner after squaring up
        if (len(tags) == 0):
            print("There was a home tag here, but now its gone")
            raise IndexError("There are no home tags seen!") 
        if is_corner(tags): #this is a second chance for a corner
            swarmie.print_infoLog(swarmie.rover_name + " dropping off in corner:" + str(set(theta_int(t.theta) for t in tags)))
            print(swarmie.rover_name, "dropping off in corner:", set(theta_int(t.theta) for t in tags))
            return(mid_point(*get_furthest_corner_hometags_location(tags)))
        
        #otherwise just drive into the side of home
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
    
    #move wrist down but not so down it hits the ground
    swarmie.set_wrist_angle(.3)
    rospy.sleep(.5)
    
    try:
        swarmie.targets_timeout = 9 # so they stay around for the decision making
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

    # Recalibrate the home location because we're here.
    swarmie.set_home_odom_location(swarmie.get_odom_location())
    swarmie.set_home_gps_location(swarmie.get_gps_location())

    swarmie.putdown() 
    
    swarmie.drive(-.25, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    #swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    #swarmie.turn(math.pi/2, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

if __name__ == '__main__' :
    main()

