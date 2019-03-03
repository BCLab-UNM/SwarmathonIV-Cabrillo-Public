#! /usr/bin/env python
"""Wait in the start queue until it's this rover's turn to do the
init behavior.
"""
from __future__ import print_function

import sys
import rospy

from mobility.srv import Queue, QueueRequest, QueueResponse


def main(**kwargs):
    queue = rospy.ServiceProxy('start_queue/wait', Queue)
    req = QueueRequest()
    try:
        queue(req)
    except rospy.ServiceException:
        pass  # good enough

    return 0


if __name__ == '__main__':
    sys.exit(main())