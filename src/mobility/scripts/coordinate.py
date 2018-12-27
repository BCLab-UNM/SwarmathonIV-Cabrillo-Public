#! /usr/bin/env python
"""Coordinate between rovers"""
try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import Any, Callable
        from genpy import Message as Msg
except ImportError:
    pass

import math
import rospy
import threading

from nav_msgs.msg import Odometry
from control_msgs.srv import QueryCalibrationState

from mobility import sync
from mobility.swarmie import Location

from swarmie_msgs.msg import StartQueuePriority


coord_lock = threading.Lock()


class Coordinator(rospy.SubscribeListener):
    """Coordinate between rovers while they're stationary before entering
    autonomous mode.

    Also implements SubscribeListener interface to publish messages to new
    subscribers.
    """

    def __init__(self):
        super(Coordinator, self).__init__()

        self._initial_pose = Location(None)
        self._queue_priority = StartQueuePriority()
        self._start_queues = [[], []]  # Rovers sorted by their priority.

        rospy.init_node('coordinate')

        rospy.Subscriber('/start_queue_priorities', StartQueuePriority,
                         self._insert_to_start_queue, queue_size=10)

        self._wait_for_initial_odometry()
        self._set_start_queue_priority()

        self.pub = rospy.Publisher(
            '/start_queue_priorities', StartQueuePriority,
            subscriber_listener=self, queue_size=10
        )

    def _wait_for_initial_odometry(self):
        """Wait until a message comes in from the IMU/Wheel Encoder EKF."""
        while self._initial_pose.Odometry is None and not rospy.is_shutdown():
            # TODO: Should there be a backup if this message doesn't arrive within 10-15 sec?
            try:
                self._initial_pose.Odometry = rospy.wait_for_message(
                    'odom/filtered', Odometry, timeout=2
                )

            except rospy.ROSException:
                rospy.logwarn(rospy.get_namespace().strip('/') +
                              ': timed out waiting for filtered odometry data.')

    def _set_start_queue_priority(self):
        """Set this rover's start queue priority and fill out the
        message fields."""
        self._queue_priority.rover_name = (
            self._initial_pose.Odometry.child_frame_id.split('/')[0]
        )

        initial_heading = self._initial_pose.get_pose().theta

        if initial_heading > 0:
            self._queue_priority.queue_index = StartQueuePriority.QUEUE_ONE
            self._queue_priority.priority = math.pi - initial_heading
        else:
            self._queue_priority.queue_index = StartQueuePriority.QUEUE_TWO
            self._queue_priority.priority = abs(initial_heading)

    @sync(coord_lock)
    def _insert_to_start_queue(self, msg):
        # type: (StartQueuePriority) -> None
        """Insert rover into the appropriate start queue and re-sort the queue
        by priorities.
        """
        self._start_queues[msg.queue_index].append(
            (msg.priority, msg.rover_name)
        )
        self._start_queues[msg.queue_index].sort(key=lambda tup: tup[0])

        log_msg = []

        for index, start_queue in enumerate(self._start_queues):
            log_msg.append('Queue {}: '.format(index + 1))
            for rover in start_queue:
                log_msg.append('{}: priority: {}'.format(rover[1],
                                                         round(rover[0], 3)))

        rospy.loginfo('Current start order: ' + str(log_msg))

    @sync(coord_lock)
    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        # type: (str, Callable[[Msg], None], Callable[[Msg], None]) -> None
        """Callback when a peer has subscribed to a topic. Send a single message
        to that peer.

        Args:
            topic_name: topic name. NOTE: topic name will be resolved/remapped
            topic_publish: method to publish message data to all subscribers
            peer_publish: method to publish message data to new subscriber.
                NOTE: behavior for the latter is transport-dependent as some
                transports may be broadcast only.

        Returns:
            None
        """
        # rospy.loginfo('New subscriber to: {}'.format(topic_name))
        peer_publish(self._queue_priority)

    @sync(coord_lock)
    def peer_unsubscribe(self, topic_name, num_peers):
        # type: (str, int) -> None
        """Callback when a peer has unsubscribed from a topic.

        Args:
            topic_name: topic name. NOTE: topic name will be resolved/remapped
            num_peers: number of remaining peers subscribed to topic

        Returns:
            None
        """
        pass  # TODO: is this callback needed?


def main():
    coord = Coordinator()
    rospy.spin()


if __name__ == '__main__':
    main()
