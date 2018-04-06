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
from numpy import arctan

'''Searcher node.''' 

def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
def on_wall():
    global swarmie
    global linec
    #placeholder
    #if find_wall() :
     #   return True
    #else:
     #   return False
    return False

def find_wall():
     global swarmie
     global prob
     #fix
     mapt = swarmie.get_obstacle_map()['obstacles']
     pointsb = []
     x = -1 
     for i in mapt:
         for j in i:
             x = x + 1
             if j > prob : 
                pointsb.append([])
                pointsb.append(int(x / 50))
                pointsb.append(int(x % 50))
     
     linesb = []
     itt = 0
     for i in pointsb:
         for j in pointsb:
             if i[0] == j[0] + 1 and i[1] == j[1] or i[0] == j[0] + 1 and i[1] == j[1] - 1 or i[0] == j[0] and i[1] == j[1] - 1 or i[0] == j[0] - 1 and i[1] == j[1] - 1 or i[0] == j[0] - 1 and i[1] == j[1] or i[0] == j[0] - 1 and i[1] == j[1] + 1 or i[0] == j[0] and i[1] == j[1] + 1 or i[0] == j[0] + 1 and i[1] == j[1] + 1:
                 if not(j[0] == linesb[itt][0] and j[1] == linesb[itt][1]): 
                     linesb.append([])
                     linesb.append(i[0])
                     linesb.append(i[1])
                     linesb.append(i[0])
                     linesb.append(j[1])
                     itt = itt + 1
     for i in linesb:
        xp = (i[1] - 25) / 2
        yp = (i[0] - 25) / -2
        yd = (i[2] - i[0]) 
        xd = (i[3] - i[1])
        if check_line(xp,yp,xd,yd,mapt,0) and check_line(xp,yp,xd,yd,mapt,1):
            return True
        else:
            return False
    
        
def check_path(head):
    global swarmie
    global pVar
    cpos = swarmie.get_odom_location().get_pose()
    hyp = math.sqrt(math.pow(cpos.x - head.x,2) + math.pow(cpos.y - head.y,2))
    stang = angles.shortest_angular_distance(0,head.theta)
    print("wtf math. Ys:",math.sin(stang) * hyp + head.y,cpos.y,"Xs:",math.cos(stang) * hyp + head.x,cpos.x)
    if math.fabs(math.sin(stang) * hyp + head.y - cpos.y) < pVar and math.fabs(math.cos(stang) * hyp + head.x - cpos.x) < pVar:
        print("found path")
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
            print("give up. mina:",mina,"smin:",smin)
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
        swarmie.drive_to(lpoint)
        while not check_path(rpoint):
            swarmie.set_heading(lpoint.theta)
            swarmie.drive(len)
            swarmie.set_heading(lpoint.theta - math.pi + math.pi / 6)
            swarmie.drive(math.sin(math.pi/6) * len)
    except ObstacleException:
        avoid(swarmie.get_odom_location().get_pose())
        return False
    return True


# 0 = empty, 1 = unexplored, 2 = possibly obstacle, 4 = obstacle
def check_line(xi,yi,xf,yf,mapt,det):
    global swarmie
    global prob
    global linec
    del linec[:]
    delta = (yf - yi) / (xf - xi)
    xf = goal.x * 2 + 25
    yf = -goal.y * 2 + 25
    xi = curpos.x * 2 + 25
    yi = -curpos.y * 2 + 25
    addy = int(math.fabs(yf - yi) / (yf - yi))
    addx = int(math.fabs(xf - xi) / (xf - xi))
    obpos = 0
    pobpos = 0
    empos = 0
    unexpos = 0
    while xi * addx < xf * addx and yi * addy < yf * addy:
        xi = xi + addx
        yi = xi * delta
        #if mapt[ty][tx] > prob or mapt[ty + 1][tx] > prob or mapt[ty][tx + 1] > prob or mapt[ty - 1][tx] > prob or mapt[ty][tx - 1] > prob:
        if mapt[int(ty)][int(tx)] > prob or mapt[int(ty + 0.25)][int(tx)] > prob or mapt[int(ty + 0.25)][int(tx + 0.25)] > prob or mapt[int(ty)][int(tx + 0.25)] > prob or mapt[int(ty - 0.25)][int(tx + 0.25)] > prob or mapt[int(ty - 0.25)][int(tx)] > prob or mapt[int(ty - 0.25)][int(tx - 0.25)] > prob or mapt[int(ty)][int(tx - 0.25)] > prob or mapt[int(ty + 0.25)][int(tx - 0.25)] > prob:
            linec[4].append({"x" : (xi - 25) / 2, "y" : (yi - 25) / 2})
            obpos = obpos + 1
        elif mapt[int(ty)][int(tx)] > 0 or mapt[int(ty + 0.25)][int(tx)] > 0 or mapt[int(ty + 0.25)][int(tx + 0.25)] > 0 or mapt[int(ty)][int(tx + 0.25)] > 0 or mapt[int(ty - 0.25)][int(tx + 0.25)] > 0 or mapt[int(ty - 0.25)][int(tx)] > 0 or mapt[int(ty - 0.25)][int(tx - 0.25)] > 0 or mapt[int(ty)][int(tx - 0.25)] > 0 or mapt[int(ty + 0.25)][int(tx - 0.25)] > 0:
            if (mapt[int(ty)][int(tx)] + mapt[int(ty + 0.25)][int(tx)] + mapt[int(ty + 0.25)][int(tx + 0.25)] + mapt[int(ty)][int(tx + 0.25)] + mapt[int(ty - 0.25)][int(tx + 0.25)] + mapt[int(ty - 0.25)][int(tx)] + mapt[int(ty - 0.25)][int(tx - 0.25)] + mapt[int(ty)][int(tx - 0.25)] + mapt[int(ty + 0.25)][int(tx - 0.25)]) / 9 > prob:
                linec[2].append({"x" : (xi - 25) / 2, "y" : (yi - 25) / 2})
                pobpos = pobpos + 1
            else:
                linec[0].append({"x" : (xi - 25) / 2, "y" : (yi - 25) / 2})
                empos = empos + 1
        elif mapt[ty][tx] == 0.0:
            linec[0].append({"x" : (xi - 25) / 2, "y" : (yi - 25) / 2})
            empos = empos + 1
        else:
            linec[1].append({"x" : (xi - 25) / 2, "y" : (yi - 25) / 2})
            unexpos = unexpos + 1
    if det == 0:
        if empos > 0:
            return False
        else:
            return True
    if det & 1 == 1:
        if unexpos > 0:
            return False
        else:
            return True
    if det & 2 == 2:
        if pobpos > 0:
            return False
        else:
            return True
    if det & 4 == 4:
        if obpos > 0:
            return False
        else:
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
 
    
def line_path_to(goal,avd):
    global swarmie
    global prob
    global linec
    global path
    curpos = swarmie.get_odom_location().get_pose()
    mapc = swarmie.get_obstacle_map()['obstacles']
    atGoal = False
    skip = 0
    cx = curpos.x
    cy = curpos.y 
    dx = goal.x
    dy = goal.y
    cha = math.py / 6
    while not atGoal and skip < 20:
        if not check_line(cx,cy,dx,dy,mapc,avd):
            hy = math.hypot(linec[avd][0]["x"] - cx, linec[avd][0]["y"] - cy)
            anc = angles.shortest_angular_distance(0,math.atan2(linec[avd][0]["y"] - cy,linec[avd][0]["x"] - cx))
        while not check_line(cx,cy,dx,dy,mapc,avd) and angles.shortest_angular_distance(anc,an):
             if hy - math.hypot(linec[avd][0]["x"] - cx, linec[avd][0]["y"] - cy) >= 1.5:
                 hy = math.hypot(linec[avd][0]["x"] - cx, linec[avd][0]["y"] - cy)
                 cha = -1 * cha
             an = angles.shortest_angular_distance(0,math.atan2(linec[avd][0]["y"] - cy,linec[avd][0]["x"] - cx)) + cha
             dx = math.cos(an) * hy
             dy = math.sin(an) * hy
             #unfinished
        skip = skip + 1
    
    if atGoal:
        drive_path()
    else:
        print("could not find known path. Driving dangerously...")
        drive_to(goal)


def path_to(start,goal,avd):
    global swarmie
    global prob
    global path
    curpos = start
    mapt = swarmie.get_obstacle_map()['obstacles']
    xf = -goal.y * 2 + 25
    yf = goal.x * 2 + 25
    xi = -curpos.y * 2 + 25
    yi = curpos.x * 2 + 25
    sqares = []
    fild = 0
    mlist = [[]]*50
    for i in range(50):
        mlist[i] = [1] * 50
    x = -1
    for i in mapt:
        for j in i:
            x = x + 1    
            if mapt[int(x / 50)][x % 50] > prob :
                mlist[int(x / 50)][x % 50] = 4
            elif mapt[int(x / 50)][x % 50] > 0 :
                print("test if")
                if int(x / 50) + 1 < 50 and int(x / 50) - 1 >= 0 and x % 50 + 1 < 50 and x % 50 >= 0 and (mapt[int(x / 50)][x % 50] > prob + mapt[int(x / 50) + 1][x % 50] + mapt[int(x / 50)][x % 50 + 1] + mapt[int(x / 50) - 1][x % 50] + mapt[int(x / 50)][x % 50 - 1]) / 4 > prob:
                    mlist[int(x / 50)][x % 50] = 2
                else:
                    mlist[int(x / 50)][x % 50] = 0
            elif mapt[int(x / 50)][x % 50] == 0.0:
                mlist[int(x / 50)][x % 50] = 0
                fild = fild + 1
            else:
                mlist[int(x / 50)][x % 50] = 1
    for i in mlist:
            print(i)
    sqares.append([])
    sqares[0].append(int(xi))
    sqares[0].append(int(yi))
    print(sqares[0][0] + 1,sqares[0][1])
    curs = 0
    atGoal = False
    while len(sqares) > 0 and not atGoal: 
        cur = 0
        badR = 0
        tR = 0
        print(sqares[curs][0] + 1,sqares[curs][1])
        if sqares[curs][0] + 1 < 50 and sqares[curs][1] < 50 and sqares[curs][1] >= 0 and not mlist[sqares[curs][0] + 1][sqares[curs][1]] & avd == avd:
            sqares.append([])
            cur = cur + 1
            sqares[curs+cur].append(sqares[curs][0] + 1)
            sqares[curs+cur].append(sqares[curs][1])
            mlist[sqares[curs][0] + 1][sqares[curs][1]] = 8 
            if math.fabs(sqares[curs][0] + 1 - xf) <= 1  and  math.fabs(sqares[curs][1] - yf) <= 1:
                atGoal = True
        elif sqares[curs][0] + 1 < 50 and sqares[curs][1] < 50 and sqares[curs][1] >= 0 and not mlist[sqares[curs][0] + 1][sqares[curs][1]] & 8 == 8:
            badR = badR + 1
        else:
            tR = tR + 1
        if sqares[curs][1] + 1 < 50 and sqares[curs][0] < 50 and sqares[curs][0] >= 0 and not mlist[sqares[curs][0]][sqares[curs][1] + 1] & avd == avd:
            sqares.append([])
            cur = cur + 1
            sqares[curs+cur].append(sqares[curs][0])
            sqares[curs+cur].append(sqares[curs][1] + 1)
            mlist[sqares[curs][0]][sqares[curs][1] + 1] = 8
            if math.fabs(sqares[curs][0] - xf) <= 1  and math.fabs(sqares[curs][1] + 1 - yf) <= 1:
                atGoal = True
        elif sqares[curs][1] + 1 < 50 and sqares[curs][0] < 50 and sqares[curs][0] >= 0 and not mlist[sqares[curs][0]][sqares[curs][1] + 1] & 8 == 8:
            badR = badR + 1
        else:
            tR = tR + 1
        if sqares[curs][0] - 1 >= 0 and sqares[curs][1] < 50 and sqares[curs][1] >= 0 and not mlist[sqares[curs][0] - 1][sqares[curs][1]] & avd == avd:
            sqares.append([])
            cur = cur + 1
            sqares[curs+cur].append(sqares[curs][0] - 1)
            sqares[curs+cur].append(sqares[curs][1])
            mlist[sqares[curs][0] - 1][sqares[curs][1]] = 8
            if math.fabs(sqares[curs][0] - 1 - xf) <= 1  and math.fabs(sqares[curs][1] - yf) <= 1:
                atGoal = True
        elif sqares[curs][0] - 1 >= 0 and sqares[curs][1] < 50 and sqares[curs][1] >= 0 and not mlist[sqares[curs][0] - 1][sqares[curs][1]] & 8 == 8:
            badR = badR + 1
        else:
            tR = tR + 1
        if sqares[curs][1] - 1 >= 0 and sqares[curs][0] < 50 and sqares[curs][0] >= 0 and not mlist[sqares[curs][0]][sqares[curs][1] - 1] & avd == avd:
            sqares.append([])
            cur = cur + 1
            sqares[curs+cur].append(sqares[curs][0])
            sqares[curs+cur].append(sqares[curs][1] - 1)
            mlist[sqares[curs][0]][sqares[curs][1] - 1] = 8
            if math.fabs(sqares[curs][0] - xf) <= 1  and math.fabs(sqares[curs][1] - 1 - yf) <= 1:
                atGoal = True
        elif sqares[curs][1] - 1 >= 0 and sqares[curs][0] < 50 and sqares[curs][0] >= 0 and not mlist[sqares[curs][0]][sqares[curs][1] - 1] & 8 == 8:
            badR = badR + 1
        else:
            tR = tR + 1
        if badR + tR >= 4: 
            del squares[curs]
            cur = cur - 1
        curs = curs + cur
        for i in mlist:
            print(i)
        
    if atGoal:
        del path[:]
        itt = 0
        #fix squares is deleated dont do that
        for i in sqares:
            print(itt,i[1],i[0],i)
            path.append([])
            path[itt].append((i[1] - 25) / 2)
            path[itt].append((i[0] - 25) / -2)
            itt = itt + 1
        return True
    else: 
        return False
    
    
def wander():
    global swarmie
    try :
        rospy.loginfo("Wandering...")
<<<<<<< HEAD
        swarmie.turn(random.gauss(0, math.pi/8))
        swarmie.drive(2)
=======
        swarmie.turn(random.gauss(0, math.pi/6))
        swarmie.drive(random.gauss(2.5, 1))

>>>>>>> master
        rospy.loginfo("Circling...")
        swarmie.circle()
        
    except ObstacleException :
        print ("I saw an obstacle!")
        turnaround()
        
         
def Dwander():
    global swarmie
    mapt = swarmie.get_obstacle_map()['obstacles']
    fild = 0
    loc = swarmie.get_odom_location().get_pose()
    xi = -loc.y * 2 + 25
    yi = loc.x * 2 + 25
    mlist = [[]]*50
    for i in range(50):
        mlist[i] = [1] * 50
    x = -1
    for i in mapt:
        for j in i:
            x = x + 1    
            if mapt[int(x / 50)][x % 50] > prob :
                mlist[int(x / 50)][x % 50] = 4
            elif mapt[int(x / 50)][x % 50] > 0 :
                print("test if")
                if int(x / 50) + 1 < 50 and int(x / 50) - 1 >= 0 and x % 50 + 1 < 50 and x % 50 >= 0 and (mapt[int(x / 50)][x % 50] > prob + mapt[int(x / 50) + 1][x % 50] + mapt[int(x / 50)][x % 50 + 1] + mapt[int(x / 50) - 1][x % 50] + mapt[int(x / 50)][x % 50 - 1]) / 4 > prob:
                    mlist[int(x / 50)][x % 50] = 2
                else:
                    mlist[int(x / 50)][x % 50] = 0
            elif mapt[int(x / 50)][x % 50] == 0.0:
                mlist[int(x / 50)][x % 50] = 0
                fild = fild + 1
            else:
                mlist[int(x / 50)][x % 50] = 1
    for i in mlist:
            print(i)
    sumn = 0
    x = 0
    while x < xi:
        for i in mapt[x]:
            sumn = sumn + i
        x = x + 1 
    sumw = 0
    y = 0
    while y < yi:
        for i in range[50]:
            sumw = sumw + mapt[i][y]
        y = y + 1 
    sums = 0
    x = 49
    while x > xi:
        for i in mapt[x]:
            sums = sums + i
        x = x - 1 
    sume = 0
    y = 49
    while y > yi:
        for i in range[50]:
            sume = sume + mapt[i][y]
        y = y - 1 
    try:
        if sumn < sume and sumn < sums and sumn < sumw:
            swarie.set_heading(math.py/2)
        elif sume < sumn and sume < sums and sume < sumw:
            swarie.set_heading(0)
        elif sums < sumn and sums < sume and sums < sumw:
            swarie.set_heading(3*math.py/2)
        elif sumw < sumn and sumw < sums and sumw < sume:
            swarie.set_heading(math.py)
        rospy.loginfo("Wandering...")
        swarmie.turn(random.gauss(-math.pi/4, math.pi/4))
        swarmie.drive(3)
        rospy.loginfo("Circling...")
        
    except ObstacleException :
        print ("I saw an obstacle!")
        avoid(swarmie.get_odom_location.get_pose())
        
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

<<<<<<< HEAD
        
def get_angle(msg):
=======

def escape_home(detections):
    global swarmie, planner

    print('\nGetting out of the home ring!!')
    angle, dist = planner.get_angle_and_dist_to_escape_home(detections)
    swarmie.turn(
        angle,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )
    swarmie.drive(
        dist,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )


def handle_exit():
    global planner, swarmie, found_tag

    reset_speeds()

    if found_tag:
        print('Found a tag! Trying to get a little closer.')
        planner.face_nearest_block()

    swarmie.print_infoLog('Setting search exit poses.')
    set_search_exit_poses()


def reset_speeds():
    global initial_config, param_client
    param_client.update_configuration(initial_config)


def set_search_exit_poses():
>>>>>>> master
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
    global linec
    
    sVar = 0.5
    sRPer = .73
    sCPer = 1.5
    pVar =  0.5
    sonVar = 0.5
    prob = 0.6
    path = []
    linec = []
    
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

<<<<<<< HEAD
        
    try: 
        rospy.sleep(.5)
        print(anglei / math.pi * 180, aprox_angle(5) / math.pi * 180)
        swarmie.set_heading(math.pi, ignore=Obstacle.IS_VISION)
    
        if swarmie.has_search_exit_poses():
            thing = swarmie.get_odom_location()
            odom_pose, gps_pose = swarmie.get_search_exit_poses()
            if path_to(swarmie.get_odom_location().get_pose(),odom_pose,3):
                for i in path:
                    thing.x = i[0]
                    thing.y = i[1]
                    try:
                        swarmie.drive_to(thing)
                        facing = swarmie.get_odom_location().get_pose()
                        facing.x = facing.x - 1
                        facing.y = facing.y - 1
                        parl = swarmie.get_odom_location().get_pose()
                        parl.x = parl.x + 1
                        parl.y = parl.y + 1
                        search_area(facing,parl,2)
                    except ObstacleException:
                        avoid(swarmie.get_odom_location)
                    
        #try:
         #   swarmie.drive(20)
        #except ObstacleException:
         #   fill_wall()
        
        for move in range(5) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                try:
                    Dwander()
                    facing = swarmie.get_odom_location().get_pose()
                    parl = swarmie.get_odom_location().get_pose()
                    parl.x = parl.x + 2
                    search_area(facing,parl,2)
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
        swarmie.set_search_exit_poses()

        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)

        exit(0)
        
=======
    # Change drive and turn speeds for this behavior, and register shutdown
    # hook to reset them at exit.
    if not rospy.has_param('/' + rovername + '/search/speeds'):
        speeds = SEARCH_SPEEDS
        rospy.set_param('/' + rovername + '/search/speeds', speeds)
    else:
        speeds = rospy.get_param('/' + rovername + '/search/speeds',
                                 default=SEARCH_SPEEDS)

    param_client = dynamic_reconfigure.client.Client(rovername + '_MOBILITY')
    config = param_client.get_configuration()
    initial_config = {
        'DRIVE_SPEED': config['DRIVE_SPEED'],
        'TURN_SPEED': config['TURN_SPEED']
    }
    param_client.update_configuration(speeds)
    rospy.on_shutdown(handle_exit)

    if not planner.sees_home_tag():
        try:
            swarmie.drive(0.5, ignore=Obstacle.IS_SONAR)
        except HomeException:
            # get out of from inside home if it ever happens
            detections = swarmie.get_latest_targets().detections
            if planner.is_inside_home_ring(detections):
                escape_home(detections)
            else:
                swarmie.turn(math.pi,
                             ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
        except TagException:
            rospy.sleep(0.3)  # build the buffer a little
            try:
                if swarmie.get_nearest_block_location() is not None:
                    found_tag = True
                    # print('Found a tag! Turning to face.')
                    # planner.face_nearest_block()
                    exit(0)  # found a tag?
            except tf.Exception:
                pass
    else:
        # get out of from inside home if it ever happens
        detections = swarmie.get_latest_targets().detections
        if planner.is_inside_home_ring(detections):
            escape_home(detections)
        else:
            swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)

    # Return to our last search exit pose if possible
    dist = 0
    cur_pose = swarmie.get_odom_location().get_pose()

    if swarmie.has_search_exit_poses():
        last_pose, _gps = swarmie.get_search_exit_poses()
        dist = math.sqrt((last_pose.x - cur_pose.x) ** 2
                         + (last_pose.y - cur_pose.y) ** 2)

    if dist > 1:  # only bother if it was reasonably far away
        print('Driving to last search exit position.')
        swarmie.print_infoLog('Driving to last search exit position.')
        try:
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True)

            cur_loc = swarmie.get_odom_location()
            if not cur_loc.at_goal(last_pose, 0.3):
                print('Getting a little closer to last exit position.')
                swarmie.drive_to(last_pose, throw=False)

        except rospy.ServiceException:
            # try again without map waypoints
            planner.drive_to(last_pose,
                             tolerance=0.5,
                             tolerance_step=0.5,
                             avoid_targets=False,
                             avoid_home=True,
                             use_waypoints=False)

            cur_loc = swarmie.get_odom_location()
            if not cur_loc.at_goal(last_pose, 0.3):
                print('Getting a little closer to last exit position.')
                swarmie.drive_to(last_pose, throw=False)

        except PathException:
            print('PathException on our way to last search exit location.')
            # good enough
            pass

        try:
            # planner.clear(math.pi / 4, ignore=Obstacle.TAG_HOME, throw=True)
            # swarmie.drive(0.2, throw=False)
            # planner.sweep(throw=True)
            swarmie.circle()
            swarmie.set_heading(
                last_pose.theta,
                ignore=Obstacle.TAG_HOME
            )
        except TagException:
            rospy.sleep(0.3)  # build buffer a little
            # too risky to stop for targets if home is in view too
            if not planner.sees_home_tag():
                # success!
                found_tag = True
                # print('Found a tag! Turning to face.')
                # planner.face_nearest_block()
                exit(0)
        except HomeException:
            # Just move onto random search, but this shouldn't really happen
            # either.
            pass
        except ObstacleException:
            print('ObstacleException while finishing return to last search exit location.')
            pass # good enough

    try:
        for move in range(30) :
            if rospy.is_shutdown() :
                exit(-1)
            try:
                wander()

            except HomeException :
                print ("I saw home!")
                planner.set_home_locations()

                # get out of from inside home if it ever happens
                detections = swarmie.get_latest_targets().detections
                if planner.is_inside_home_ring(detections):
                    escape_home(detections)

                turnaround()

    except TagException :
        print("I found a tag!")
        # Let's drive there to be helpful.
        rospy.sleep(0.3)
        if not planner.sees_home_tag():
            found_tag = True
            # print('Found a tag! Turning to face.')
            # planner.face_nearest_block()
            # swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.6, ignore=Obstacle.IS_VISION)
            exit(0)

>>>>>>> master
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()
