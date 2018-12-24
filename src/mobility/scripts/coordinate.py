#! /usr/bin/env python
"""Coordinate between rovers"""
try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import Any, Callable
        from genpy import Message as Msg
except ImportError:
    pass

import angles
import rospy
import threading

from nav_msgs.msg import Odometry

from mobility import sync
from mobility.swarmie import Location

from swarmie_msgs.msg import SwarmieHeading


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
        self._initial_headings = []  # Accrue a list of all rovers' initial headings.
        self._start_queue = []  # Rovers sorted by their initial heading.

        rospy.init_node('coordinate')

        rospy.Subscriber('/initial_heading', SwarmieHeading,
                         self._initial_heading_cb, queue_size=10)

        while self._initial_pose.Odometry is None and not rospy.is_shutdown():
            # TODO: Should there be a backup if this message doesn't arrive within 10-15 sec?
            try:
                self._initial_pose.Odometry = rospy.wait_for_message(
                    'odom/filtered',
                    Odometry,
                    timeout=2
                )
            except rospy.ROSException:
                rospy.logwarn(rospy.get_namespace().strip('/') +
                              ': timed out waiting for filtered odometry data.')

        self.pub = rospy.Publisher('/initial_heading', SwarmieHeading,
                                   subscriber_listener=self, queue_size=10)

    @sync(coord_lock)
    def _initial_heading_cb(self, msg):
        # type: (SwarmieHeading) -> None
        self._initial_headings.append(msg)
        self._start_queue = sorted(
            self._initial_headings,
            key=lambda x: angles.normalize_angle_positive(x.theta),
            reverse=True
        )

        log_msg = []

        for rover in self._start_queue:  # type: SwarmieHeading
            log_msg.append('{}: theta: {}'.format(rover.rover_name,
                                                  round(rover.theta, 3)))

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
        msg = SwarmieHeading()
        msg.rover_name = self._initial_pose.Odometry.child_frame_id.split('/')[0]
        msg.theta = self._initial_pose.get_pose().theta
        peer_publish(msg)

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
