#! /usr/bin/env python 

from __future__ import print_function

import rospy 
from diagnostics.diags import Diagnostics 
from diagnostics.watcher import TopicWatcher

def main() :
    rospy.init_node('diagnostics')
    
    # Wait for a clock message.
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now > 0:
            break
    
    d = Diagnostics()
    d.run()

if __name__ == '__main__':
    main()
