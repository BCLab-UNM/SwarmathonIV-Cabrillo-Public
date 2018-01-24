#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import angles
import random 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException
from Tkconstants import FIRST

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def avoid():
    global Swarmie
    head = swarmie.get_odom_location().get_pose()
    print(swarmie.get_obstacle_condition(), Obstacle.IS_SONAR)
    while swarmie.get_obstacle_condition() == Obstacle.IS_SONAR :
        print(swarmie.get_obstacle_condition(), Obstacle.IS_SONAR)
        while swarmie.get_obstacle_condition() == Obstacle.SONAR_CENTER :
            swarmie.turn(math.pi/4, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            
        try :
            swarmie.drive(1, ingnore=Obstacle.SONAR_RIGHT)
        except ObstacleException :
            swarmie.turn(math.pi/4, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            
            
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
    swarmie.turn(math.pi - math.pi/8, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION )
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
        
def orbit():
    global swarmie
    try :
        rospy.loginfo("fibring...")
        home = swarmie.get_home_odom_location()
        odom = swarmie.get_odom_location().get_pose()
        dist = math.fabs(odom.x - home.x) * 2 + .5
        head = math.floor((odom.theta + math.pi / 2) / (math.pi/2) + .5) * math.pi / 2
        print("facing:", odom.theta / math.pi * 180, "heading:", head / math.pi * 180)
        
        swarmie.set_heading(head)
        swarmie.drive(dist)
        #swarmie.drive_to(home)
        
    except ObstacleException :
        print ("I saw an obstacle!")
        avoid()

def main():
    global swarmie 
    global rovername 
    global first
    
    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.fingers_open()
    swarmie.wrist_middle()
    print("search start...")
    
    try :
        first
    except NameError :
        first = False
        try:
            swarmie.drive(1)
        except HomeException:
            print("home")
        
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)


    print(swarmie.get_obstacle_condition(), Obstacle.TAG_HOME)   
    if swarmie.get_obstacle_condition() == Obstacle.TAG_HOME :
        print("this is important")
        swarmie.turn(-math.pi / 2, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
        swarmie.drive(1, ignore=Obstacle.IS_VISION)

    try: 
        for move in range(5) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                orbit()
            
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location().get_pose()
                print(odom_location.y)
                swarmie.set_home_odom_location(odom_location)
                #turnaround()
                
    except TagException : 
        print("I found a tag!")
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

