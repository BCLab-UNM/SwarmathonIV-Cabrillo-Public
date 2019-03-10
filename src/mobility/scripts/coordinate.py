#! /usr/bin/env python
"""Coordinate between rovers.
TODO: Put StartQueuePriority.msg into mobility instead of swarmie_msgs?
"""
try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import Any, Callable, List, Set, Tuple
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
from mobility.srv import (Queue, QueueRequest, QueueResponse,
                          QueueRemove, QueueRemoveRequest, QueueRemoveResponse,
                          GetRoverNames, GetRoverNamesRequest,
                          GetRoverNamesResponse)


coord_lock = threading.Lock()


class Coordinator(rospy.SubscribeListener):
    """Coordinate between rovers while they're stationary before entering
    autonomous mode.

    Also implements SubscribeListener interface to publish messages to new
    subscribers.
    """

    # Amount priority will be increased if a rover has to perform an IMU
    # calibration at the start of a round. This is an appropriate number
    # because rover priorities by initial heading are in [0, PI], so increasing
    # this value by 10 is sure to drop the rover to the back of the queue.
    CAL_PRIORITY_PENALTY = 10

    def __init__(self):
        super(Coordinator, self).__init__()

        rospy.init_node('coordinate')

        self._rover_name = rospy.get_namespace().strip('/')
        self._initial_pose = Location(None)
        self._queue_priority = StartQueuePriority()
        self._queue_priority.is_finished = False

        # Rovers sorted by their priority.
        self._start_queues = [[], []]  # type: List[List[Tuple[float, str]], List[Tuple[float, str]]]
        self._remove_proxies = []  # type: List[rospy.ServiceProxy]
        self._rovers = {self._rover_name}  # type: Set[str]

        # Service proxies to query the IMU node.
        self._imu_is_finished_validating = rospy.ServiceProxy(
            'imu/is_finished_validating', QueryCalibrationState
        )
        self._imu_needs_calibration = rospy.ServiceProxy(
            'imu/needs_calibration', QueryCalibrationState
        )

        self.sub = rospy.Subscriber('/start_queue/priorities',
                                    StartQueuePriority,
                                    self._insert_to_start_queue, queue_size=10)

        self._wait_for_initial_odometry()
        self._set_start_queue_priority()

        self.pub = rospy.Publisher(
            '/start_queue/priorities', StartQueuePriority,
            subscriber_listener=self, queue_size=10
        )

        self.queue = rospy.Service('start_queue/wait', Queue, self._queue)
        self.remove = rospy.Service('start_queue/remove', QueueRemove,
                                    self._remove_from_queue)
        self.list_rovers = rospy.Service('get_rover_names', GetRoverNames,
                                         self._list_rovers)

    def _wait_for_initial_odometry(self):
        """Wait until a message comes in from the IMU/Wheel Encoder EKF."""
        while self._initial_pose.Odometry is None and not rospy.is_shutdown():
            # TODO: Should there be a backup if this message doesn't arrive within 10-15 sec?
            try:
                self._initial_pose.Odometry = rospy.wait_for_message(
                    'odom/filtered', Odometry, timeout=2
                )

                if self._initial_pose.get_pose().theta == 0:
                    # This can happen occasionally in the sim. It's probably
                    # because the robot_localization node hasn't received IMU
                    # data yet.
                    rospy.logwarn(
                        ('{}: This initial odometry message has a heading of ' +
                         'exactly 0. Dropping this message and waiting for a ' +
                         'better one.').format(self._rover_name)
                    )
                    self._initial_pose.Odometry = None

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

        # Only call the IMU services if the simulation isn't running.
        if not rospy.get_param('/use_sim_time', False):
            while not self._imu_is_finished_validating().is_calibrated:
                rospy.sleep(0.5)

            if self._imu_needs_calibration().is_calibrated:
                self._queue_priority.priority += (
                    Coordinator.CAL_PRIORITY_PENALTY
                )

    @sync(coord_lock)
    def _insert_to_start_queue(self, msg):
        # type: (StartQueuePriority) -> None
        """Insert a rover into the appropriate start queue, if it says it hasn't
        finished queueing yet, and re-sort the queue by priorities.
        """
        if msg.rover_name != self._queue_priority.rover_name:
            self._remove_proxies.append(
                rospy.ServiceProxy(
                    '/{}/start_queue/remove'.format(msg.rover_name),
                    QueueRemove
                )
            )
            self._rovers.add(msg.rover_name)

        if msg.is_finished:
            rospy.loginfo(('{}: {} has already finished queueing. Not adding ' +
                           'them to my start queue.').format(self._rover_name,
                                                             msg.rover_name))
            return

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

    def _queue(self, req):
        # type: (QueueRequest) -> QueueResponse
        """ROS service callback to start waiting in the start queue.

        Args:
            req: the request to start waiting. Includes a timout per rover,
                which is multiplied by the number of rovers ahead of this one
                in the start queue to determine the total timeout length.

        Returns:
            The service QueueResponse, including the result, whether the service
                timed out, or completed successfully.
        """
        timeout_per_rover = QueueRequest.TIMEOUT_DEFAULT
        if req.timeout_per_rover > 1e-5:
            timeout_per_rover = req.timeout_per_rover

        num_rovers_ahead = 0

        for rover in self._start_queues[self._queue_priority.queue_index]:
            if rover[1] == self._queue_priority.rover_name:
                break
            num_rovers_ahead += 1

        timeout_time = (rospy.Time().now()
                        + rospy.Duration(timeout_per_rover * num_rovers_ahead))

        done_once = False
        while not done_once or rospy.Time().now() < timeout_time:
            if len(self._start_queues[self._queue_priority.queue_index]) == 0:
                rospy.logwarn(("{}: This start queue is empty, so there's " +
                               "nothing to wait for.").format(self._rover_name))
                return QueueResponse(result=QueueResponse.SUCCESS)

            elif (self._start_queues[self._queue_priority.queue_index][0][1]
                  == self._queue_priority.rover_name):
                return QueueResponse(result=QueueResponse.SUCCESS)

            done_once = True

        return QueueResponse(result=QueueResponse.TIMEOUT)

    @sync(coord_lock)
    def _remove_from_queue(self, req):
        # type: (QueueRemoveRequest) -> QueueRemoveResponse
        """Remove a rover from the start queue. This provides a service to both
        local and remote nodes. A local node, like the local task manager, for
        example, can call with req.notify_others=True, and this service will
        then call the remote services to update everyone's start queues before
        returning.

        Args:
            req: the request containing the rover to remove.

        Returns:
            The empty service response.
        """
        if req.rover_name == self._rover_name:
            self._queue_priority.is_finished = True

        for queue in self._start_queues:
            for index, rover in enumerate(queue):
                if rover[1] == req.rover_name:
                    rospy.loginfo('{}: removing {} from queue'.format(
                        rospy.get_name(),
                        rover[1]
                    ))
                    queue.pop(index)

        if req.notify_others:
            rospy.loginfo('notifying others.')
            for service in self._remove_proxies:
                service(QueueRemoveRequest(rover_name=req.rover_name))

        return QueueRemoveResponse()

    @sync(coord_lock)
    def _list_rovers(self, req):
        # type: (GetRoverNamesRequest) -> GetRoverNamesResponse
        """Return a list of the rovers currently online.

        Args:
            req: the service request.

        Returns:
            the service response containing the list of rover names.
        """
        response = GetRoverNamesResponse()
        response.rovers = list(self._rovers)
        return response


def main():
    coord = Coordinator()
    rospy.spin()


if __name__ == '__main__':
    main()
