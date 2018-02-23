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
from sensor_msgs.msg import Range

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException
from Tkconstants import FIRST
from numpy import angle
from asyncore import poll


def detectm():
    global angle
    global sonar_left
    global sonar_right
    global sonar_center
    avgl = 0
    mm = angle
    avgr = 0
    avgc = 0
    x = 0
    ml = -9999
    mr = -9999
    mc = -9999
    while x != 1:
        for y in range(20):
            rospy.sleep(.1)
            l = sonar_left
            r = sonar_right
            c =  sonar_center
            if ml < math.fabs(avgl / (y + 1) - l):
                ml = l
            if mr < math.fabs(avgr / (y + 1) - r):
                mr = r
            if mc < math.fabs(avgc / (y + 1) - c):
                mc = c
            avgl = avgl + l
            avgr = avgr + r
            avgc = avgc + c
            avgl = avgl / 20
            avgr = avgr / 20
            avgc = avgc / 20 
        try:
            swarmie.drive(1,ignore= Obstacle.IS_VISION)
        except ObstacleException:
            print("obstacle hit:")
            print("    avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
            print("    avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
            print("    avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            if math.fabs(sonar_left - avgl) > 0.5:
                print("    change in varience left:")
                print("        avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
                print("        avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
                print("        avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            if math.fabs(sonar_right - avgr) > 0.5:
                print("    change in varience right:")
                print("        avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
                print("        avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
                print("        avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            if math.fabs(sonar_center - avgc) > 0.5:
                print("    change in varience center:")
                print("        avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
                print("        avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
                print("        avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            swarmie.set_heading(angle + math.pi/2, ignore=Obstacle.IS_SONAR)
            print("obstacle hit:")
            print("    avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
            print("    avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
            print("    avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            exit(1)    
        
        print("avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
        print("avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
        print("avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
        if math.fabs(sonar_left - avgl) > 0.5:
            print("change in varience left:")
            print("    avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
            print("    avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
            print("    avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            x = 1
        if math.fabs(sonar_right - avgr) > 0.5:
            print("change in varience right:")
            print("    avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
            print("    avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
            print("    avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            x = 1
        if math.fabs(sonar_center - avgc) > 0.5:
            print("change in varience center:")
            print("    avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
            print("    avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
            print("    avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
            x = 1
    
def detect():
    global angle
    global sonar_left
    global sonar_right
    global sonar_center
    avgl = 0
    avgr = 0
    avgc = 0
    x = 0
    ml = -9999
    mr = -9999
    mc = -9999

    for y in range(20):
        rospy.sleep(.1)
        l = sonar_left
        r = sonar_right
        c =  sonar_center
        if ml < math.fabs(avgl / (y + 1) - l):
            ml = l
        if mr < math.fabs(avgr / (y + 1) - r):
            mr = r
        if mc < math.fabs(avgc / (y + 1) - c):
            mc = c
        avgl = avgl + l
        avgr = avgr + r
        avgc = avgc + c
        
    avgl = avgl / 20
    avgr = avgr / 20
    avgc = avgc / 20 
    print("avg left:", avgl, "now:", sonar_left, "max varience:" , math.fabs(avgl - ml))
    print("avg right:", avgr, "now:", sonar_right, "max varience:" , math.fabs(avgr - mr))
    print("avg center:", avgc, "now:", sonar_center, "max varience:" , math.fabs(avgc - mc))
             

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
    
    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.fingers_open()
    swarmie.wrist_middle()
    print("sensor start...")
   
    rospy.Subscriber(rovername + '/imu', Imu, get_angle)
    rospy.Subscriber(rovername + '/sonarLeft', Range, get_sonar_left)
    rospy.Subscriber(rovername + '/sonarRight', Range, get_sonar_right)
    rospy.Subscriber(rovername + '/sonarCenter', Range, get_sonar_center)
     
            
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)
        
    #for do in range(20) :
    detectm()
        
    print ("End of sensing")
    exit(1)

if __name__ == '__main__' : 
    main()

