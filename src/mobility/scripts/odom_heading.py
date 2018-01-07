#! /usr/bin/env python
from __future__ import print_function
import sys
import rospy
import rosnode
from nav_msgs.msg import Odometry
from mobility.swarmie import Location

def print_odom(msg):
    loc = Location(msg)
    pose = loc.get_pose()
    print(pose)

def main():
    if len(sys.argv) < 2:
        rovers = set()
        nodes = rosnode.get_node_names()
        for node in nodes:
            if 'MOBILITY' in node:
                node = node.lstrip('/')
                rovername = node.split('_')[0]
                rovers.add(rovername)
        if len(rovers) == 0:
            print('\033[91m',"No Rovers Detected",'\033[0m')
            print('usage:', sys.argv[0], '<rovername>')
            exit(-1)
        elif len(rovers) == 1:
            rovername = rovers.pop()
            print('Detected rovers: ', rovername)
            print('\033[92m',"Auto selected:",rovername,'\033[0m')
        else:
            print('Detected rovers:')
            for rover in rovers:
                print(rover)
            rovername = ''
            while rovername not in rovers:
                rovername = raw_input('Which rover would you like to connect to? ')
    else:
        rovername = sys.argv[1]

    rospy.init_node('odom_listener')
    sub = rospy.Subscriber(rovername + '/odom', Odometry, print_odom, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()