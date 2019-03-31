#! /usr/bin/env python
"""Find a corner of the home plate.

This module provides an importable function that can be used in multiple
behaviors if necessary.

TODO: Ideally, the rover will drive counter-clockwise around the home ring when
 it looks for a corner. This means it should generally be turning right. There
 are currently still cases where the rover will start going in the wrong
 direction. This is bad because another rover might be to the left of this
 rover, coming straight toward it.
"""
from __future__ import print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List, Optional, Tuple
except ImportError:
    pass

import angles
import math
import rospy
import tf.transformations

from mobility.swarmie import swarmie, HomeCornerException, PathException
from mobility import utils

from geometry_msgs.msg import PoseStamped

from swarmie_msgs.msg import Obstacle


def recover():
    """Recover from the scenario where no home tags are in view anymore."""
    # TODO: should sonar be ignored when recovering?
    # TODO: is simply backing up a little bit a reliable recovery move?
    ignore = (Obstacle.TAG_TARGET | Obstacle.TAG_HOME |
              Obstacle.INSIDE_HOME | Obstacle.IS_SONAR)
    swarmie.drive(-.1, ignore=ignore)


def find_rightmost_home_tag(transform_timeout=0.15):
    # type: (float) -> Optional[PoseStamped]
    """Find the rightmost home tag to drive toward, transformed to the
    odometry frame.

    Args:
        transform_timeout: How long to wait for each transform.

    Returns:
        The PoseStamped to drive to, or None if no tag could be found.
    """
    targets = swarmie.get_targets_buffer(id=256, age=8)
    targets_xformed = []  # type: List[Tuple[PoseStamped, PoseStamped]]
    now = rospy.Time.now()

    for target in targets:
        try:
            ps_odom = swarmie.transform_pose('odom', target.pose,
                                             timeout=transform_timeout)
            ps_odom.header.stamp = now
            ps_base_link = swarmie.transform_pose('base_link', ps_odom,
                                                  timeout=transform_timeout)
            targets_xformed.append((ps_odom, ps_base_link))

        except tf.Exception as e:
            rospy.logwarn_throttle(1.0,
                                   ('Transform exception in' +
                                    'find_rightmost_home_tag(): {}').format(e))

    targets_sorted = sorted(targets_xformed,
                            key=lambda tup: tup[1].pose.position.y)

    if len(targets_sorted) > 0:
        # Filter targets_sorted to contain a group of the rightmost tags
        targets_sorted = filter(
            lambda tup: abs(tup[1].pose.position.y
                            - targets_sorted[0][1].pose.position.y) < 0.07,
            targets_sorted
        )
        # Of the rightmost tags, return one that's far away.
        return max(targets_sorted, key=lambda tup: tup[1].pose.position.x)[0]

    targets_sorted = utils.sort_tags_left_to_right(
        swarmie.get_targets_buffer(id=256, age=0.5)
    )

    if len(targets_sorted) == 0:
        return None

    rightmost_target = targets_sorted[-1]
    try:
        return swarmie.transform_pose('odom', rightmost_target.pose)
    except tf.Exception as e:
        rospy.logwarn(
            'Transform exception in find_rightmost_home_tag(): {}'.format(e)
        )

    return None

def find_home_corner(max_fail_count=3):
    # type: (int) -> bool
    """Drive around and find a corner of the home plate.

    Args:
        max_fail_count: How many times the rover will stop with no home tags in
            view, backing up to try to find them again, before giving up.

    Returns:
        True if a corner was found.

    Raises:
        PathException: If the fails to find a home tag max_fail_count times.
    """
    # The rover drives repeatedly to the rightmost home tag in view. However,
    # swarmie.drive_to() drives to a point such that the rover is on top of that
    # point. If the rover did this here, it's likely no home tags would be in
    # view anymore. The claw_offset argument allows the rover to stop short of
    # the goal point by this distance in meters.
    claw_offset = 0.3

    # How many times the rover has stopped with no home tags in view.
    fail_count = 0

    rightmost_tag = None

    # TODO: should sonar be ignored all the time?
    ignore = (Obstacle.TAG_TARGET | Obstacle.TAG_HOME |
              Obstacle.INSIDE_HOME | Obstacle.IS_SONAR)

    # Because this function's caller has likely only just seen a home tag,
    # waiting for a brief moment here helps to ensure the first call to
    # swarmie.get_targets_buffer() returns with home tags in the list.
    rospy.sleep(0.1)

    for _ in range(10):
        try:
            prev_rightmost_tag = rightmost_tag
            rightmost_tag = find_rightmost_home_tag()

            if rightmost_tag is None:
                if prev_rightmost_tag is not None:
                    rightmost_tag = prev_rightmost_tag
                else:
                    fail_count += 1
                    if fail_count >= max_fail_count:
                        raise PathException(('Unable to find a home corner, ' +
                                             'no home tags are in view.'))

                    recover()
                    continue

            # TODO: pull helpers out of home_transform.py and into utils.py
            #  so you can use yaw_from_quaternion() here.
            _r, _p, home_yaw = tf.transformations.euler_from_quaternion(
                [rightmost_tag.pose.orientation.x,
                 rightmost_tag.pose.orientation.y,
                 rightmost_tag.pose.orientation.z,
                 rightmost_tag.pose.orientation.w]
            )

            # Using set heading before driving is useful when the rover is
            # mostly facing the corner to its left (which isn't the one we
            # want to drive to). Given the home tag's orientation on the
            # home ring, this gets the rover facing to the right, and keeps
            # it that way. Otherwise, it's possible for the rover to be
            # facing a corner to its left, and the simple "drive to the
            # rightmost tag" logic here will fail; the rover won't move at
            # all because its already where it should be according to
            # drive_to().
            # TODO: this if statement's logic could be improved. The rover
            #  frequently wastes time turning to the right when it arrives
            #  oriented directly at one of the corners. It just needs to
            #  drive a little bit forward in that case and it would see the
            #  corner and get the corner exception.
            rover_yaw = swarmie.get_odom_location().get_pose().theta

            if (abs(angles.shortest_angular_distance(rover_yaw, home_yaw))
                    > math.pi / 2):
                swarmie.set_heading(home_yaw + math.pi / 3, ignore=ignore)
            else:
                swarmie.drive_to(rightmost_tag.pose.position,
                                 claw_offset=claw_offset, ignore=ignore)

        except HomeCornerException:
            # success!!
            return True

    raise PathException('Unable to find a home corner, I gave up.')


def main():
    if find_home_corner():
        print('Success!! We found a corner of home!!!')


if __name__ == '__main__':
    # For testing this as a standalone script.
    swarmie.start(node_name='find_corner')
    main()
