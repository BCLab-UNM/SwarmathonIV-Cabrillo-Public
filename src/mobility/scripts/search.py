#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import time
import angles
import random 
import message_filters
import tf

from swarmie_msgs.msg import Obstacle
from geometry_msgs.msg import Vector3, Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException
from Tkconstants import FIRST
from numpy import angle
from asyncore import poll

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def avoid():
    global swarmie
    head = swarmie.get_odom_location().get_pose()
    print(swarmie.get_obstacle_condition(), Obstacle.SONAR_LEFT, Obstacle.SONAR_RIGHT)
    while swarmie.get_obstacle_condition() & 1 == Obstacle.SONAR_LEFT or swarmie.get_obstacle_condition() & 2 == Obstacle.SONAR_RIGHT or swarmie.get_obstacle_condition() & 4 == Obstacle.SONAR_CENTER :
        print(swarmie.get_obstacle_condition(), Obstacle.IS_SONAR)
        while swarmie.get_obstacle_condition() & 4 == Obstacle.SONAR_CENTER :
            swarmie.turn(math.pi/10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            
        try :
            swarmie.drive(1, ingnore=Obstacle.SONAR_RIGHT)
        except ObstacleException :
            swarmie.turn(math.pi/8, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            print("obstacle still present")
            
            
    swarmie.set_heading(head.theta, ignore=Obstacle.IS_SONAR)
           
def wander():
    global swarmie
    try :
        rospy.loginfo("Wandering...")
        swarmie.turn(random.gauss(0, math.pi/8))
        swarmie.drive(2)
        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround()
        
def triangle():
    global swarmie
    dist = 0
    swarmie.set_heading(swarmie.get_odom_location().get_pose().theta - math.pi/8, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION )
    while dist < 4 :
        odom = swarmie.get_odom_location().get_pose()
        home = swarmie.get_home_odom_location()
        dist = math.hypot(home.y - odom.y,
                    home.x - odom.x)
        try :
            rospy.loginfo("triangle...")
            swarmie.drive(1)
        
        except ObstacleException :
            print ("I saw an obstacle!")
            avoid()

    print("made it")
    
    try:
        swarmie.drive(50)
        
    except ObstacleException :
        try:
            odom = swarmie.get_odom_location().get_pose()
            #print("odom:",odom/math.pi * 180 )#,"new:",(math.floor((math.pi/2 + odom.theta)/(math.pi/2)) * math.pi/2)/math.pi * 180)
            swarmie.drive(-.2, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            #swarmie.set_heading(math.floor((math.pi/2 + odom.theta)/(math.pi/2)) * math.pi/2, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            swarmie.turn(-math.pi * 3/4, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            swarmie.drive(1, ignore=Obstacle.SONAR_RIGHT)
        except ObstacleException :
            avoid()
           
    try:        
        try:        
            home = swarmie.get_home_odom_location() 
            swarmie.drive_to(home)
        except ObstacleException:
                avoid()
    except HomeException:
        swarmie.set_heading(math.floor(swarmie.get_odom_location().get_pose.theta / (math.pi / 2)))

        
def orbit(home):
    global swarmie
    try :
        rospy.loginfo("fibring...")
        odom = swarmie.get_odom_location().get_pose()
        xval = math.fabs(odom.x - home.x)
        yval = math.fabs(odom.y - home.y)
        if math.fabs(xval - yval) > 1 : 
            grid = swarmie.get_home_odom_location()
            grid.x = xval 
            grid.y = yval + .5
            swarmie.drive_to(grid) 
        dist = xval * 2 + .5
        print("1l x:",xval,"y:",yval, "dist:", dist)
        head = math.floor((odom.theta + math.pi / 2) / (math.pi/2) + .5) * math.pi / 2
        print("facing:", odom.theta / math.pi * 180, "heading:", head / math.pi * 180)
        
        swarmie.set_heading(head)
        swarmie.drive(dist)
        #swarmie.drive_to(home)
        
    except ObstacleException :
        print ("I saw an obstacle!")
        avoid()
        
def get_angle(msg):
    global angle 
    quat = [    msg.orientation.x, 
                msg.orientation.y, 
                msg.orientation.z, 
                msg.orientation.w, 
                ] 
    angle = tf.transformations.euler_from_quaternion(quat)[2]
   
    #print(info/math.pi * 180, "\n", swarmie.get_odom_location().get_pose().theta / math.pi * 180, "\n", max(math.fabs(info/math.pi * 180 - swarmie.get_odom_location().get_pose().theta / math.pi * 180)), "\n")

#def max(largest):
#    global maximum
#    if largest > maximum and largest < 350:
#        maximum = largest
#    return maximum

def aprox_angle(poll):
    global angle
    avg = 0
    if not swarmie.is_moving() :
        for i in range(0,poll):
            avg = avg + angle
            rospy.sleep(.1)
        avg = avg / poll
    else :
        avg = angle
    return (math.floor(avg / (math.pi/4) + 4.5) - 4) * math.pi 

def main():
    global swarmie 
    global rovername 
    global lhome
    global angle
    
    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.fingers_open()
    swarmie.wrist_middle()
    print("search start...")
   
    rospy.Subscriber(rovername + '/imu', Imu, get_angle)
     
            
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
        
    try :
        swarmie.drive(1)
    except HomeException :
        swarmie.drive(-.5 , ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        print(angle/ math.pi * 180, aprox_angle(5) / math.pi * 180)
        #swarmie.set_heading(angles.shortest_angular_distance(0 , angle + math.pi), ignore=Obstacle.IS_VISION)
        swarmie.set_heading(angle + math.pi/2, ignore=Obstacle.IS_VISION)
        print(angle/ math.pi * 180, aprox_angle(5) / math.pi * 180)
        swarmie.set_heading(angle + math.pi/2, ignore=Obstacle.IS_VISION)
        print(angle/ math.pi * 180, aprox_angle(5) / math.pi * 180)


    try: 
        for move in range(5) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                triangle()
                #wander()
            
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location().get_pose()
                print(odom_location.y)
                swarmie.set_home_odom_location(odom_location)
                #turnaround()
                
    except TagException : 
        print("I found a tag!")
        # Let's drive there to be helpful.
        number = swarmie.get_latest_targets()
        print(number, "map", swarmie.get_target_map())
        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.3, ignore=Obstacle.IS_VISION)
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

