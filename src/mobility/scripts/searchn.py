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
from __builtin__ import True

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def on_wall():
    print("not working")
    return False

        
def check_path(head):
    global swarmie
    global pVar
    cpos = swarmie.get_odom_location().get_pose()
    hyp = math.sqrt(math.pow(cpos.x - head.x,2) + math.pow(cpos.y - head.y,2))
    stang = angles.shortest_angular_distance(0,head.theta)
    print("wtf math. Ys:",math.sin(stang) * hyp + head.y,cpos.y,"Xs:",math.cos(stang) * hyp + head.x,cpos.x)
    if math.fabs(math.sin(stang) * hyp + head.y - cpos.y) < pVar and math.fabs(math.cos(stang) * hyp + head.x - cpos.x) < pVar:
        return True
    else:
        return False
 
        
def align_wall():
    global swarmie
    global anglei
    global sonar_left
    global sonar_right
    global sonar_center
    global sVar
    global sRPer
    global sCPer
    global pVar
    atmin = False
    mina = anglei
    smin = 4
    sprev = 4
    dir = -1
    turnsr = 0
    turnsl = 0
    over = 0
    dmin = 0
    
    while math.fabs(sonar_left - sonar_right) > sVar and not atmin:
        swarmie.turn(dir * math.py / 12, ignore=Obstacle.IS_SONAR)
        rospy.sleep(.1)
        l = left_sonar 
        r = right_sonar
        c = center_sonar
        a = anglei
        if ((l - r) * dir > 0 and c > smin and c > sprev) or turnsr >= 12 or turnsl >= 12: 
            if turnsr >= 12 or turnsl >= 12:
                turnsr = 0
                turnsl = 0
                over = over + 1
            turnsr = turnsr + (-1 * dir) * math.floor((math.fabs(turnsr) + turnsr) / (2 * turnsr))
            turnsl = turnsl + dir * math.floor((math.fabs(turnsl) + turnsl) / (2 * turnsl))
            dir = dir * -1
        if c < smin:
            smin = c
            mina = a
            dmin = 0
        else :
            dmin = dmin + 1
        if dmin > 7 or over >=2:
            swarmie.set_heading(mina + math.pi / 2, ignore=Obstacle.IS_SONAR)
            atmin = True
        sprev = c 
        

def follow_wall():
    global swarmie
    global anglei
    global sonar_left
    global sonar_right
    global sonar_center
    global sRPer
    global sCPer
    global pVar
    
    align_wall()
    npast = True
    dist = 0
    while napst :
        try:
             swarmie.drive(1,ignore=Obstacle.SONAR_RIGHT)
             dist = dist + 1
        except ObstacleException:
            swarmie.turn(math.py / 12,ignore=Obstacle.IS_SONAR)
        rospy.sleep(.1)
        if sonar_right > sRPer: 
            swarmie.set_heading(anglei - math.py / 2, ignore=Obstacle.IS_SONAR)
            rospy.sleep(.1)
            if sonar_center < sCPer:
                napst = True
                align_wall()
            else:
                npast = False
                swarmie.set_heading(angle + math.py / 2, ignore=Obstacle.IS_SONAR)
        if dist > 5:
            npast = False


def avoid(head):
    global swarmie
    global anglei
    global sonar_left
    global sonar_right
    global sonar_center
    global sRPer
    global sCPer
    global pVar
    
    alignp = False
    while not on_wall() and not check_path(head):
        follow_wall()
        swarmie.set_heading(angle - math.py / 2, ignore=Obstacle.IS_SONAR)
        try:
            drive(1,ignore=Obstacle.SONAR_RIGHT)
            swarmie.set_heading(angle - math.py / 2, ignore=Obstacle.IS_SONAR)
            swarmie.drive(dist,ignore=Obstacle.SONAR_RIGHT)
        except ObstacleException:
            print("new collision")
            
    if on_wall():
        return False
    else:
        return True


def fill_wall():
    global swarmie
    while not on_wall():
        try:
            swarmie.drive(10)
        except:
            avoid(swarmie.get_odom_location().get_pose())
        

def search_area(lpoint,rpoint,len):
    global swarmie
    global anglei
    try:
        drive_to(lpoint)
        while not check_path(rpoint):
            swarmie.set_heading(lpoint)
            swarmie.drive(len)
            swarmie.set_heading(lpoint - math.pi + math.pi / 6)
            swarmie.drive(math.sin(math.pi/6) * len)
    except ObstacleException:
        avoid(swarmie.get_odom_location().get_pose())
        return False
    return True


def find_area():
    global swarmie
    global local_map
    #local_map needs to be a dictionary of arrays that contain fr,fl,br,bl that are poses indicating a rectangle with each angle pointing the next point to the right
    fillmap()
    l = local_map[unexplored][0].fl
    r = local_map[unexplored][0].fl
    r.x = r.x + math.cos(r.theta) * 2
    r.y = r.y + math.sin(r.theta) * 2
    d = math.hypot(local_map[unexplored][0].bl.x - l.x,local_map[unexplored][0].bl.y - l.y)
    return (l,r,d)

def branch(tx,ty,delta,goal):
    global path
    unob = True
    atGoal = False
    while (tx > 0 and tx < 50) and (ty > 0 and ty < 50) and unob and not atGoal:
        ty = ty + add
        tx = int(ty * delta)
        if mapt[ty][tx] > prob or mapt[ty + 1][tx] > prob or mapt[ty][tx + 1] > prob or mapt[ty - 1][tx] > prob or mapt[ty][tx - 1] > prob:
            unob = False
        elif numpy.isnan(mapt[ty][tx]) or numpy.isnan(mapt[ty + 1][tx]) or numpy.isnan(mapt[ty][tx + 1]) or numpy.isnan(mapt[ty - 1][tx]) or numpy.isnan(mapt[ty][tx - 1]):
            unob = False
        elif math.fabs(ty / 2 - goal.y) < pVar or math.fabs(ty / 2 - goal.y) < pVar:
            atGoal = true
    
def path_to(goal):
    global swarmie
    global local_map
    global prob
    curpos = swarmie.get_odom_location().get_pose()
    xl = goal.x - curpos.x
    yl = goal.y - curpos.y
    delta = xl / yl
    add = int(math.fabs(yl) / yl)
    mapt = swarmie.get_obstacle_map()['obstacles']
    tx = int(curpos.x * 2 + .45) + 25
    ty = int(curpos.y * 2 + .45) + 25
    branch(tx,ty,delta,goal)
            
    
    #placeholder 
    drive_to(goal)
        
        
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
    global anglei
    
    swarmie.set_heading(anglei + math.pi / 10, ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION )
    try :
        rospy.loginfo("pieanglei...")
        swarmie.drive(10)
        print("wall?")
        swarmie.set_heading(anglei - math.pi, ignore=Obstacle.IS_SONAR)
        try:
            swarmie.drive(2)
        except ObstacleException:
            avoid(swarmie.get_odom_location().get_pose())
        
    except ObstacleException :
        print ("I saw an obstacle!")
        z = 0
        facing = swarmie.get_odom_location().get_pose()
        while z < 1 :
            z = 0
            try : 
                swarmie.drive(10)
            except ObstacleException:
                print("hit things")
            while avoid(facing) and z < 1: 
                print(z + 1, "times trying to avoid")
                z = z + 1
    print("going home")
    go_back()
        
    print("made it")
    if swarmie.get_obstacle_condition() & 512 != 512:
        go_back()
    
def go_back():
    global swarmie
    global anglei
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
                avoid(swarmie.get_odom_location().get_pose())
                go_back()
    except HomeException:
        #figure out home vectors (rvis?)
        swarmie.set_heading(anglei + math.pi * 0.9 , ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)

        
def get_angle(msg):
    global swarmie
    global anglei 
    quat = [    msg.orientation.x, 
                msg.orientation.y, 
                msg.orientation.z, 
                msg.orientation.w, 
                ] 
    anglei = tf.transformations.euler_from_quaternion(quat)[2]
   
   
def get_sonar_left(msg):
    global swarmie
    global sonar_left 
    sonar_left = msg.range
    
    
def get_sonar_right(msg):
    global swarmie 
    global sonar_right
    sonar_right = msg.range
    
def get_sonar_center(msg):
    global sonar_center 
    sonar_center = msg.range
    




def aprox_angle(poll):
    global swarmie 
    global anglei
    avg = 0
    if not swarmie.is_moving() :
        for i in range(0,poll):
            avg = avg + anglei
            rospy.sleep(.1)
        avg = avg / poll
    else :
        avg = anglei
    return (math.floor(avg / (math.pi/4) + 4.5) - 4) * math.pi 

def main():
    global swarmie 
    global rovername 
    #global lhome
    global anglei
    global sonar_left
    global sonar_right
    global sonar_center
    global sVar
    global sRPer
    global sCPer
    global pVar
    global sonVar
    global prob
    global path
    
    sVar = 0.5
    sRPer = .73
    sCPer = 1.5
    pVar =  0.5
    sonVar = 0.5
    prob = 0.6
    path = [] * 5
    
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
        
   
    print(anglei / math.pi * 180, aprox_angle(5) / math.pi * 180)
    swarmie.set_heading(math.pi / 2, ignore=Obstacle.IS_VISION)


    try: 
        for move in range(5) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                try:
                    swarmie.drive(10)
                except ObstacleException:
                    avoid(swarmie.get_odom_location().get_pose())
            
            except HomeException : 
                print ("I saw home!")
                swarmie.set_home_odom_location(swarmie.get_odom_location())
                swarmie.set_home_gps_location(swarmie.get_gps_location())
                turnaround()
                
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