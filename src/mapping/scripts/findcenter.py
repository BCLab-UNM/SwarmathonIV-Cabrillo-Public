#! /usr/bin/env python 

from __future__ import print_function

import sys
import rospy 
import angles
import math
import copy
import threading
from rospy.numpy_msg import numpy_msg

import tf
from grid_map_msgs.msg import GridMap
import cv2
import numpy as np

from sensor_msgs.msg import Image


def target_map_sub(target_map):
    global pub
    ndata_f = target_map.data[1].data 

    ndata_f.resize(300,300)    
    ndata = ndata_f * 65535; 
    ndata = ndata.astype('int16')
    im = Image(encoding='mono16')
    im.height, im.width, channels = ndata.shape + (1,)

    contig = np.ascontiguousarray(ndata)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        ndata.dtype.byteorder == '>' or 
        ndata.dtype.byteorder == '=' and sys.byteorder == 'big'
    )
    pub.publish(im)
    
def main() :
    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    global rover    
    rover = sys.argv[1]
    rospy.init_node(rover + '_FINDCENTER')
    
    rospy.Subscriber(rover + '/target_map', numpy_msg(GridMap), target_map_sub)
    
    global pub
    pub = rospy.Publisher(rover + '/home_image', Image, queue_size=1)
    rospy.spin()

if __name__ == '__main__' : 
    main()
    