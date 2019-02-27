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

from mobility.swarmie import swarmie, HomeCornerException, PathException
from mobility import utils

from swarmie_msgs.msg import Obstacle


def recover():
    """Recover from the scenario where no home tags are in view anymore."""
    # TODO: should sonar be ignored when recovering?
    # TODO: is simply backing up a little bit a reliable recovery move?
    ignore = (Obstacle.TAG_TARGET | Obstacle.TAG_HOME |
              Obstacle.INSIDE_HOME | Obstacle.IS_SONAR)
    swarmie.drive(-.1, ignore=ignore)


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

    # TODO: should sonar be ignored all the time?
    ignore = (Obstacle.TAG_TARGET | Obstacle.TAG_HOME |
              Obstacle.INSIDE_HOME | Obstacle.IS_SONAR)

    while True:
        try:
            sorted_tags = utils.sort_tags_left_to_right(
                swarmie.get_latest_targets(id=256)
            )
            if len(sorted_tags) == 0:
                fail_count += 1
                if fail_count >= max_fail_count:
                    # TODO: is it useful to raise a PathException here? Or would
                    #  a sensible return value be better? An exception is likely
                    #  to crash the caller, forcing the task manager to handle
                    #  it appropriately.
                    raise PathException(
                        'Unable to find a home corner, no home tags are in view.'
                    )

                recover()
            else:
                rightmost_tag = sorted_tags[-1]
                tag_xformed = swarmie.transform_pose('odom', rightmost_tag.pose)

                swarmie.drive_to(tag_xformed.pose.position,
                                 claw_offset=claw_offset, ignore=ignore)

        except HomeCornerException:
            # success!!
            return True


def main():
    swarmie.start(node_name='find_corner')
    if find_home_corner():
        print('Success!! We found a corner of home!!!')


if __name__ == '__main__':
    # For testing this as a standalone script.
    main()
