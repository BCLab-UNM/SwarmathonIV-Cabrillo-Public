#! /usr/bin/env python 

from __future__ import print_function

import rospy 

from rospy.msg import AnyMsg

from diagnostics.watcher import TopicWatcher
    
def main() :
    rospy.init_node('diagnostics')

    diags_pub = rospy.Publisher('diagnostics', String, queue_size=2, latch=True)
    
    r = rospy.Rate(1) # 1hz

    # Wait for a clock message.
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now > 0:
            break

    t = TopicWatcher('wristAngle_fuck')
    
    while not rospy.is_shutdown():
        t.check()
        r.sleep()


if __name__ == '__main__':
    main()
    