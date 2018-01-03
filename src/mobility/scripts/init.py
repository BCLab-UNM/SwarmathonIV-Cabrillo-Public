#! /usr/bin/env python

from __future__ import print_function

import sys 
import math 
import rospy 

from swarmie_msgs.msg import Obstacle

from mobility.swarmie import Swarmie

def main():
    global swarmie 
    global rovername 
    
    if len(sys.argv) < 2 :
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername)

    # During a normal startup the rover will be facing the center and
    # close to the nest. But there's no guarantee where we will be if 
    # mobility crashes and is forced to restart. This checks to see 
    # if we've set home location prviously.
    
    if swarmie.has_home_gps_location() : 
        # Oops. This must be a restart. Don't assume we're near the 
        # nest. Just exit and hope... 
        exit(0)

    # Assume the starting position is facing the center. 
    # This should be valid by contest rules. 
    #
    # Drive closer until we can see the center. 
    try:
        swarmie.drive(1) 
    except: 
        # This could happen if we bump into another rover. 
        # Let's just call it good. 
        pass
    
    # Wait up to two minutes for an good GPS fix. 
    # TODO: Can we ever get a fix this good in real life? 
    home = swarmie.wait_for_fix(distance=3, time=120) 
    
    if home is None : 
        swarmie.print_infoLog(rovername + ' failed to get a GPS fix!')        
    else:
        swarmie.set_home_gps_location(home)

    swarmie.turn(math.pi, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    
if __name__ == '__main__' : 
    main()
