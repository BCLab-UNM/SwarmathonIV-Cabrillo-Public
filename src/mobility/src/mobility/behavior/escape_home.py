#! /usr/bin/env python
"""Escape from the inside of home if the rover ever finds itself there when
it shouldn't be!
"""
import sys
import math
import angles
import rospy
import tf

from mobility.swarmie import swarmie, Obstacle


def get_angle_and_dist_to_escape_home(detections):
    """Return the angle to turn and distance to drive in order to get
    out of the home area if the rover is trapped in there. Should be
    used in conjuction with Planner.is_inside_home_ring()

    Args:
    * detections - the list of AprilTagDetections

    Returns:
    * angle - the angle in radians to turn.
    * distance - the distance in meters to drive.

    Raises:
    * tf.Exception if the transform into the base_link frame fails.
    """
    OVERSHOOT_DIST = 0.4  # meters, distance to overshoot target by
    result = {
        'angle': sys.maxint,
        'dist': None
    }
    see_home_tag = False

    for detection in detections:
        if detection.id == 256:
            see_home_tag = True
            home_detection = swarmie.transform_pose('/base_link',
                                                    detection.pose)

            quat = [home_detection.pose.orientation.x,
                    home_detection.pose.orientation.y,
                    home_detection.pose.orientation.z,
                    home_detection.pose.orientation.w]
            _r, _p, y = tf.transformations.euler_from_quaternion(quat)
            y -= math.pi / 2
            y = angles.normalize_angle(y)

            if abs(y) < result['angle']:
                result['angle'] = y
                result['dist'] = \
                    (math.sqrt(home_detection.pose.position.x ** 2
                               + home_detection.pose.position.y **2)
                     + OVERSHOOT_DIST)

    if not see_home_tag:
        # doesn't make sense to turn or drive if no home tags were seen
        return 0, 0

    return result['angle'], result['dist']


def escape_home(detections):
    rospy.logwarn('Getting out of the home ring!!')

    angle, dist = get_angle_and_dist_to_escape_home(detections)
    swarmie.turn(
        angle,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )
    swarmie.drive(
        dist,
        ignore=Obstacle.IS_SONAR | Obstacle.IS_VISION
    )


def main(**kwargs):
    """Get out of home!"""
    obstacles = swarmie.get_obstacle_condition()
    if obstacles & Obstacle.INSIDE_HOME != Obstacle.INSIDE_HOME:
        return
    escape_home(swarmie.get_latest_targets(id=256))


if __name__ == '__main__':
    swarmie.start(node_name='escape_home')
    sys.exit(main())
