#! /usr/bin/env python

from __future__ import print_function

import sys
import rospy 
import math
import random

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

'''Searcher node.''' 
def fib(n):
    '''
    fibananci number
    '''
    a,b =0,1
    while n > 0:
        a,b = b, a + b
        n -= 1
    return a

def fib_move(n):
    global swarmie
    try:
        rospy.loginfo("fibonanci pattern")
        print('turn pi/2')
        swarmie.turn(math.pi/2)
        print ('drive {}'.format(fib(n)))
        swarmie.drive(fib(n))
    except ObstacleException :
        print ("I saw an obstacle!")
        loc = self.get_odom_location().get_pose()
        angle = angles.shortest_angular_distance(loc.theta, -math.pi/4)
        swarmie.turn(angle,ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
        '''
        swarmie.turn(math.pi/2,ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
        swarmie.drive(2)
        
        swarmie.turn(math.pi/2)
        swarmie.drive(1)
        swarmie.turn(-math.pi/2)
        swarmie.drive(1)
        swarmie.turn(-math.pi/2)
        swarmie.drive(1)
        swarmie.turn(math.pi/2)
        '''
def turnaround(): 
    global swarmie
    swarmie.turn(random.gauss(math.pi/2, math.pi/4), ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
    
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


def randomWalk(num_steps):
    '''
    does a 2 dementional random walk in 4 directionsw
    arguments:
       num_steps - number of random steps
       it usees tandom step size listed in "step_sizes
    '''
    global swarmie
    # step for randim walk
    step_sizes = [1.0,1.5,2.0,2.5,3] #possible step sizes
    step_size = random.choice(step_sizes) #choose one of the step size        # Size of step, arbitrary value
   
    # eight possible directions
    moves =[(0,step_size),        ##
            (math.pi/2,step_size), #
            (math.pi,step_size),
            (-math.pi/2,step_size),
            ]
        
    for i in range(num_steps):
        try :
            rospy.loginfo("Random walk...")
            direction,distance = moves[random.randint(0,3)]  
            swarmie.turn(direction)
            swarmie.drive(distance)
            
        except ObstacleException :
            print ("I saw an obstacle!")
            distance = step_size       # bounced back the way we came.
            swarmie.turn(math.pi,ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)
            swarmie.drive(distance,ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION)

def main():
    global swarmie 
    global rovername 
    food_location =[]
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)
    swarmie.fingers_open()
    swarmie.wrist_middle()

    try: 
        '''
        for wander range is 6
        for fib_move about 10.
        for random walk, since step size is small, 10 with 20 steps in random walk
        '''
        for move in range(10) :
            if rospy.is_shutdown() : 
                exit(-1)
            try:
                '''
                wander() # original search
                fib_move(move+2) # path using fibnoccii numbers
                randomWalk()
                '''
                
                #if len(food_location) > 1: #if if found a foud go to last food location
                 #   swarmie.drive_to(food_location.pop())
                randomWalk(20)
            except HomeException : 
                print ("I saw home!")
                odom_location = swarmie.get_odom_location()
                swarmie.set_home_odom_location(odom_location)
                turnaround()
                
    except TagException : 
        print("I found a tag!")
        # Let's drive there to be helpful.
        swarmie.drive_to(swarmie.get_nearest_block_location(), claw_offset=0.3, ignore=Obstacle.IS_VISION)
        #food_location.append(swarmie.get_nearest_block_location())#store latest food location
        exit(0)
        
    print ("I'm homesick!")
    exit(1)

if __name__ == '__main__' : 
    main()

