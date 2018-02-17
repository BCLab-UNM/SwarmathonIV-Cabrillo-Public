#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import time
import angles
import numpy
import random 
import message_filters
import tf

from swarmie_msgs.msg import Obstacle
from geometry_msgs.msg import Vector3, Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException
from Tkconstants import FIRST
from numpy import angle #not sure this is still needed
from asyncore import poll

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def avoid():
    global swarmie
    global angle
    global sonar_left
    global sonar_right
    global sonar_center
    global sVar
    global sRPer
    global sCPer
    global pVar
    #if swarmie.get_obstacle_condition() & 3 == Obstacle.SONAR_LEFT:
        
    #elif swarmie.get_obsticle_condition() & 3 == Obstacle.SONAR_RIGHT:
    
    nrpath = True
    head = swarmie.get_odom_location().get_pose()
    print(swarmie.get_obstacle_condition(), Obstacle.SONAR_LEFT, Obstacle.SONAR_RIGHT)
    while nrpath :
        print(sonar_left, sonar_center, sonar_right, Obstacle.IS_SONAR)
        while sonar_right < sRPer - sVar / 2 or sonar_center < sCPer - sVar / 2 :
            swarmie.set_heading(angle + math.pi/10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            
        try:
            swarmie.drive(1, ignore=Obstacle.SONAR_RIGHT)
        except ObstacleException:
            if swarmie.get_obstacle_condition() & 2 == Obstacle.SONAR_RIGHT:
                print("this is still broken")
                print(sonar_left, sonar_center, sonar_right, Obstacle.IS_SONAR)
            else:
                print("obstacle still present")
        
        print("after obstacle present")        
        while sonar_right > sRPer + sVar or sonar_center > sCPer + sVar :
            swarmie.set_heading(angle - math.pi / 10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
        #conditional apon allignment being of path
        cpos = swarmie.get_odom_location().get_pose()
        hyp = math.sqrt(math.pow(cpos.x - head.x,2) + math.pow(cpos.y - head.y,2))
        if math.floor(math.sin(head.theta) * hyp - cpos.y) < pVar and math.floor(math.cos(head.theta) * hyp - cpos.x) < pVar:
            swarmie.set_heading(head.theta, ignore=Obstacle.IS_SONAR)
            if swarmie.get_obstacle_condition() == 0:
                rpath = False
            else:
                swarmie.set_heading(cpos.theta, ignore=Obstacle.IS_SONAR)
    swarmie.set_heading(head.theta, ignore=Obstacle.IS_SONAR)
    try: 
        drive(0.5)
    except ObstacleException:
        avoid()
        
def avoid_wall():
    global swarmie
    global angle
    global sonar_left
    global sonar_right
    global sonar_center
    global sVar
    global sRPer
    global sCPer
    global pVar
    #if swarmie.get_obstacle_condition() & 3 == Obstacle.SONAR_LEFT:
        
    #elif swarmie.get_obsticle_condition() & 3 == Obstacle.SONAR_RIGHT:
    
    nrpath = True
    head = swarmie.get_odom_location().get_pose()
    print(swarmie.get_obstacle_condition(), Obstacle.SONAR_LEFT, Obstacle.SONAR_RIGHT)
    while nrpath :
        print(sonar_left, sonar_center, sonar_right, Obstacle.IS_SONAR)
        while sonar_right < sRPer - sVar / 2 or sonar_center < sCPer - sVar / 2 :
            swarmie.set_heading(angle + math.pi/10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            
        try:
            swarmie.drive(1, ignore=Obstacle.SONAR_RIGHT)
        except ObstacleException:
            if swarmie.get_obstacle_condition() & 2 == Obstacle.SONAR_RIGHT:
                print("this is still broken")
                print(sonar_left, sonar_center, sonar_right, Obstacle.IS_SONAR)
            else:
                print("obstacle still present")
        
        print("after obstacle present")        
        while sonar_right > sRPer + sVar or sonar_center > sCPer + sVar :
            swarmie.set_heading(angle - math.pi / 10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
        #conditional apon allignment being of path
        cpos = swarmie.get_odom_location().get_pose()
        hyp = math.sqrt(math.pow(cpos.x - head.x,2) + math.pow(cpos.y - head.y,2))
        if math.floor(math.sin(head.theta) * hyp - cpos.y) < pVar and math.floor(math.cos(head.theta) * hyp - cpos.x) < pVar:
            swarmie.set_heading(head.theta, ignore=Obstacle.IS_SONAR)
            if swarmie.get_obstacle_condition() == 0:
                rpath = False
            else:
                swarmie.set_heading(cpos.theta, ignore=Obstacle.IS_SONAR)
        #placeholder for difference change latter 
        if math.hypot(swarmie.get_odom_location().get_pose().y - head.y, 
                      swarmie.get_odom_location().get_pose().x - head.x) > 2:
            print("test return")
            return
    swarmie.set_heading(head.theta, ignore=Obstacle.IS_SONAR)
    try: 
        drive(0.5)
    except ObstacleException:
        avoid()
           
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
    global angle
    
    swarmie.set_heading(angle + math.pi / 10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION )
    try :
        rospy.loginfo("pieangle...")
        swarmie.drive(10)
        print("wall?")
        swarmie.set_heading(angle - math.pi, ignore=Obstacle.IS_SONAR)
        try:
            swarmie.drive(2)
        except ObstacleException:
            avoid()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        avoid_wall()

    print("made it")
    go_back()
    
def go_back():
    global angle
    try:        
        try:  
            for x in range(20):      
                home = swarmie.get_home_odom_location() 
                odom = swarmie.get_odom_location().get_pose()
                thetah = math.atan2(home.y - odom.y, home.x - odom.x)
                swarmie.set_heading(thetah)
                hyph = math.hypot(odom.y - home.y, 
                                  odom.x - home.x)
                swarmie.drive(hyph + 1)
            exit(1)
        except ObstacleException:
                avoid()
                go_back()
    except HomeException:
        #figure out home vectors (rvis?)
        swarmie.set_heading(angle + math.pi * 0.9 , ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)

        
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

def get_sonar_left(msg):
    global sonar_left 
    sonar_left = msg.range
    
def get_sonar_right(msg):
    global sonar_right
    sonar_right = msg.range
    
def get_sonar_center(msg):
    global sonar_center 
    sonar_center = msg.range
    
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
    #global lhome
    global angle
    global sonar_left
    global sonar_right
    global sonar_center
    global sVar
    global sRPer
    global sCPer
    global pVar
    
    sVar = 0.5
    sRPer = 0.8
    sCPer = 1.6
    pVar =  0.5
    
    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.fingers_open()
    swarmie.wrist_middle()
    print("search start...")
   
    rospy.Subscriber(rovername + '/imu', Imu, get_angle)
    rospy.Subscriber(rovername + '/sonarLeft', Range, get_sonar_left)
    rospy.Subscriber(rovername + '/sonarRight', Range, get_sonar_right)
    rospy.Subscriber(rovername + '/sonarCenter', Range, get_sonar_center)
     
            
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
        
    try :
        print("map:    ", swarmie.get_target_map().stamp)
        swarmie.drive(1)
    except HomeException :
        swarmie.set_home_odom_location(swarmie.get_odom_location())
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
                print("new1")
                triangle()
                #wander()
            
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location().get_pose()
                swarmie.set_home_odom_location(odom_location)
                #turnaround()
                
    except TagException : 
        print("I found a tag!")
        # Let's drive there to be helpful.
        print("here",swarmie.get_odom_location().get_pose(), "block",swarmie.get_latest_targets(), "map:", swarmie.get_target_map().stamp)

        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)

        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

