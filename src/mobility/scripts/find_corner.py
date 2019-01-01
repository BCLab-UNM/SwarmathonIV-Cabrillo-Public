#! /usr/bin/env python
"""Find a corner of the home plate."""
from __future__ import print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List, Optional, Tuple
except ImportError:
    pass

import angles
import copy
import math
import numpy as np
import rospy
import sys
import tf
import tf.transformations

from geometry_msgs.msg import (PointStamped, Pose, Pose2D, PoseStamped,
                               PoseArray, Quaternion)

from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray


# Globals
# ROS publishers
closest_pub = None
rotated_pub = None
intersect_pub = None
corner_pub = None
# base_link_frame parameter
base_link_frame = None


def euler_from_quaternion(quat):
    # type: (Quaternion) -> Tuple[float, float, float]
    """Return roll, pitch, yaw representation of the Quaternion."""
    r, p, y = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w]
    )
    return r, p, y


def yaw_from_quaternion(quat):
    # type: (Quaternion) -> float
    """Extract the yaw component of a quaternion."""
    return euler_from_quaternion(quat)[2]


def ps_to_p2d(ps):
    # type: (PoseStamped) -> Pose2D
    """Convert a PoseStamped to a Pose2D."""
    return Pose2D(x=ps.pose.position.x, y=ps.pose.position.y,
                  theta=yaw_from_quaternion(ps.pose.orientation))


def pnt_dist(p1, p2):
    # type: (Tuple[float, float], Tuple[float, float]) -> float
    """Return the distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def pose_dist(pose1, pose2):
    # type: (PoseStamped, PoseStamped) -> float
    """Return the 2D distance between two poses (using only x and y)."""
    return pnt_dist((pose1.pose.position.x, pose1.pose.position.y),
                    (pose2.pose.position.x, pose2.pose.position.y))


def rotate_quaternion(q, angle, axis):
    # type: (Quaternion, float, Tuple[float, float, float]) -> Quaternion
    """Rotate the quaternion by the specified angle around the given
    axis (x, y, z).
    """
    rotated = tf.transformations.quaternion_multiply(
        [q.x, q.y, q.z, q.w],
        tf.transformations.quaternion_about_axis(angle, axis)
    )

    return Quaternion(*rotated)


def rotate(anchor, pose, angle):
    # type: (PoseStamped, PoseStamped, float) -> Tuple[PoseStamped, PoseStamped]
    """Rotate pose and anchor about the anchor's position by the
    specified angle.
    """
    o = copy.deepcopy(anchor)
    p = copy.deepcopy(pose)

    cosa = math.cos(angle)
    sina = math.sin(angle)

    # translate
    p.pose.position.x -= o.pose.position.x
    p.pose.position.y -= o.pose.position.y

    # rotate
    x = p.pose.position.x * cosa - p.pose.position.y * sina
    y = p.pose.position.x * sina + p.pose.position.y * cosa

    # translate back
    p.pose.position.x = x + o.pose.position.x
    p.pose.position.y = y + o.pose.position.y

    # rotate the pose orientations
    o.pose.orientation = rotate_quaternion(o.pose.orientation, angle, (0, 0, 1))
    p.pose.orientation = rotate_quaternion(p.pose.orientation, angle, (0, 0, 1))

    return o, p


def intersect(line1,  # type: Tuple[float, float, float]
              line2   # type: Tuple[float, float, float]
              ):
    # type: (...) -> Optional[Tuple[float, float]]
    """Return the point of intersection between two lines.

    Args:
        line1: the first line, described in a point, slope form as (x, y, slope)
        line2: the second line, described in a point, slope form as (x, y, slope)

    Returns:
        The point of intersection (x, y), or None if there isn't one.
    """
    # To find the point of intersection, we need to solve a system of 2 linear
    # equations in the form ax + by = c. The two lines are defined by a point
    # and a slope, so moving from point-slope form to the form ax + by = c
    # yields:
    # (y - y1) = m * (x - x1)
    # y - y1 = m * x - m * x1)
    # - m * x + y = y1 - m * x1 (ax + by = c)

    # Solve for x, y where x1, y1, m1, x2, y2, and m2 are known.
    # - m1 * x + y = y1 - m1 * x1
    # - m2 * x + y = y2 - m2 * x2
    a = np.array([[-line1[2], 1.],
                  [-line2[2], 1.]])
    b = np.array([[line1[1] - line1[2] * line1[0]],
                  [line2[1] - line2[2] * line2[0]]])

    try:
        int_ = np.linalg.solve(a, b)
    except np.linalg.LinAlgError:
        # This is unlikely to happen, but could if lines were parallel.
        return None

    return int_.item(0), int_.item(1)


# min_d = sys.maxint
def intersected(pose1, pose2, epsilon=0.03):
    # type: (PoseStamped, PoseStamped, float) -> Optional[PoseStamped]
    """Given a pair of poses, return the pose that is approximately intersected
    by the other.

    A pose is intersected by the other if the pose lies approximately on the
    line described by the other pose's position and orientation in 2D space.

    Args:
        pose1: the first pose
        pose2: the second pose
        epsilon: A small value used to determine whether two points are close
            enough to each other.

    Returns:
        A copy of the intersected pose, or None if neither pose intersects
            the other.
    """
    # global min_d

    p1 = (pose1.pose.position.x, pose1.pose.position.y)
    p2 = (pose2.pose.position.x, pose2.pose.position.y)

    line1 = (p1[0], p1[1],
             math.tan(yaw_from_quaternion(pose1.pose.orientation)))
    line2 = (p2[0], p2[1],
             math.tan(yaw_from_quaternion(pose2.pose.orientation)))

    p = intersect(line1, line2)
    if p is None:
        return None

    # min_ = min(pnt_dist(p1, p), pnt_dist(p2, p))
    # if min_ < min_d:
    #     min_d = min_
    #     print('min dist seen: {}'.format(min_))
    # print('min pnt dist: {}'.format(min(pnt_dist(p1, p), pnt_dist(p2, p))))

    # If the intersection point is very close two one of the two poses, then
    # that pose lies approximately on the line described by the other pose.
    if pnt_dist(p1, p) < epsilon:
        return copy.deepcopy(pose1)
    elif pnt_dist(p2, p) < epsilon:
        return copy.deepcopy(pose2)

    return None


def are_inline(pose1, pose2):
    # type: (PoseStamped, PoseStamped) -> bool
    """Return True if either pose is oriented directly toward or away from
    the other, within a tolerance.
    """
    pose = intersected(pose1, pose2)
    if pose is not None:
        return True

    return False


def closest_inline_pair(poses1,  # type: List[PoseStamped]
                        poses2  # type: List[PoseStamped]
                        ):
    # type: (...) -> Optional[Tuple[PoseStamped, PoseStamped]]
    """Return the closest pair of poses that are inline with each other, one
    pose from each list. Each list should contain at least one pose.

    Args:
        poses1: the list of poses in one orientation
        poses2: the list of poses in the other orientation (turned 90 deg)

    Returns:
        The pair of poses, or None, if no pair of poses are inline.
    """
    # TODO: return a deep copy of the pair of poses?
    closest = (sys.maxint, PoseStamped(), PoseStamped())
    found_inline = False

    for p1 in poses1:
        for p2 in poses2:
            if are_inline(p1, p2):
                found_inline = True
                dist = pose_dist(p1, p2)
                if dist < closest[0]:
                    closest = (dist, p1, p2)

    if found_inline:
        return closest[1:]

    return None


def find_corner_tags(detections):
    # type: (List[PoseStamped]) -> Optional[Tuple[PoseStamped, PoseStamped]]
    """Find the closest pair of tags whose orientations in 2D space are
    approximately perpendicular and are inline with each other.

    Detections should not be empty and should only contain home tags (id ==
    256).
    """
    global closest_pub, rotated_pub, intersect_pub

    pose_buckets = [[], []]  # type: List[List[PoseStamped], List[PoseStamped]]
    pose_buckets[0].append(detections[0])

    for detection in detections[1:]:  # type: PoseStamped
        angle_between = abs(angles.shortest_angular_distance(
            yaw_from_quaternion(pose_buckets[0][0].pose.orientation),
            yaw_from_quaternion(detection.pose.orientation)
        ))

        if angle_between < 0.25:  # ~15 deg
            pose_buckets[0].append(detection)
        elif angle_between > math.pi / 2 - 0.25:  # PI/2 minus a nominal ~15 deg.
            pose_buckets[1].append(detection)

    if len(pose_buckets[1]) > 0:  # We found 2 different orientations of tags.
        pair = closest_inline_pair(*pose_buckets)
        if pair is None:
            return None

        pose1, pose2 = pair

        # are_inline(pose1, pose2)
        # pose3 = Pose()

        # angle_between = angles.shortest_angular_distance(
        #     yaw_from_quaternion(pose1.pose.orientation),
        #     yaw_from_quaternion(pose2.pose.orientation)
        # )
        # q = tf.transformations.quaternion_multiply(
        #     [pose1.pose.orientation.x, pose1.pose.orientation.y,
        #      pose1.pose.orientation.z, pose1.pose.orientation.w],
        #     tf.transformations.quaternion_about_axis(angle_between / 2.,
        #                                              (0, 0, 1))
        # )
        # pose3.position.x = (pose1.pose.position.x + pose2.pose.position.x) / 2.
        # pose3.position.y = (pose1.pose.position.y + pose2.pose.position.y) / 2.
        # pose3.position.z = (pose1.pose.position.z + pose2.pose.position.z) / 2.
        # pose3.orientation.x = q[0]
        # pose3.orientation.y = q[1]
        # pose3.orientation.z = q[2]
        # pose3.orientation.w = q[3]

        # TODO: only publish if debugging?
        msg = PointStamped()
        msg.header = pose1.header
        int_ = intersected(pose1, pose2)
        if int_ is not None:
            msg.point.x = int_.pose.position.x
            msg.point.y = int_.pose.position.y
            msg.point.z = int_.pose.position.z
        intersect_pub.publish(msg)

        msg = PoseArray()
        msg.header = pose1.header
        msg.poses = [pose1.pose, pose2.pose]  # , pose3]
        closest_pub.publish(msg)

        return pose1, pose2
    else:
        # Publish empty PoseArray's so RViz doesn't show old data.
        msg = PoseArray()
        msg.header = detections[0].header
        closest_pub.publish(msg)
        rotated_pub.publish(msg)  # TODO: move into new function

        msg = PointStamped()
        msg.header = detections[0].header
        intersect_pub.publish(msg)

    return None


def corner_pose(intersected_pose, other_pose):
    # type: (PoseStamped, PoseStamped) -> PoseStamped
    """Return the calculated corner pose, given two inline, perpendicular home
    tag poses.
    """
    result = copy.deepcopy(intersected_pose)
    result.pose.position.x = ((intersected_pose.pose.position.x
                               + other_pose.pose.position.x) / 2.)
    result.pose.position.y = ((intersected_pose.pose.position.y
                               + other_pose.pose.position.y) / 2.)
    result.pose.position.z = ((intersected_pose.pose.position.z
                               + other_pose.pose.position.z) / 2.)

    # Rotate the other pose about the intersected pose by the angle required
    # to give the intersected pose a zero heading in the current frame of
    # reference.
    int_rot, other_rot = rotate(
        intersected_pose, other_pose,
        -yaw_from_quaternion(intersected_pose.pose.orientation)
    )

    # Type 2 corner
    if yaw_from_quaternion(other_rot.pose.orientation) > 0:
        result.pose.orientation = rotate_quaternion(result.pose.orientation,
                                                    math.pi / 2, (0, 0, 1))

    return result


def targets_cb(msg, listener):
    # type: (AprilTagDetectionArray, tf.TransformListener) -> None
    """Find corner of home."""
    global base_link_frame, rotated_pub, corner_pub

    detections = []  # type: List[PoseStamped]
    for detection in msg.detections:  # type: AprilTagDetection
        if detection.id == 256:
            try:
                xpose = listener.transformPose(base_link_frame, detection.pose)
                r, p, y = euler_from_quaternion(xpose.pose.orientation)

                if r < 0.25 and p < 0.25:  # ~15 degrees
                    # TODO: is this a good number?
                    # TODO: could filtering ever be a problem, since the
                    #  transform is going to the base link frame, if the rover
                    #  was yawed or pitched while driving over a cube? I don't
                    #  know if transforming into the odom frame would help in
                    #  that case either, because robot_localization is in 2D
                    #  mode.
                    # Occasionally a detection's pose estimate isn't calculated
                    # very well, and its orientation has a very large roll
                    # and/or pitch component, when we know the tags are flat
                    # on the ground.
                    detections.append(xpose)

            except tf.Exception:
                pass

    # If a home tag is in view
    if len(detections) > 0:
        corner = find_corner_tags(detections)
        if corner is not None:
            # print(ps_to_p2d(corner[0]), ps_to_p2d(corner[1]))
            pose1, pose2 = corner
            int_ = intersected(pose1, pose2)

            if int_ == pose1:
                cor_pose = corner_pose(pose1, pose2)
                rotated_poses = rotate(
                    pose1, pose2, -yaw_from_quaternion(pose1.pose.orientation)
                )
                # p1_rot, p2_rot = rotated_poses
            else:
                cor_pose = corner_pose(pose2, pose1)
                rotated_poses = rotate(
                    pose2, pose1, -yaw_from_quaternion(pose2.pose.orientation)
                )
                # p2_rot, p1_rot = rotated_poses

            corner_pub.publish(cor_pose)

            msg = PoseArray()
            msg.header = pose1.header
            msg.poses = [ps.pose for ps in rotated_poses]
            rotated_pub.publish(msg)
        else:
            # Publish empty corner pose
            msg = PoseStamped()
            msg.header = detections[0].header
            corner_pub.publish(msg)


def main():
    global base_link_frame, closest_pub, rotated_pub, intersect_pub, corner_pub

    rospy.init_node('find_corner')

    listener = tf.TransformListener(cache_time=rospy.Duration(secs=30))

    base_link_frame = rospy.get_param('base_link_frame')
    print(base_link_frame)

    sub = rospy.Subscriber('targets', AprilTagDetectionArray,
                           targets_cb, callback_args=listener)
    closest_pub = rospy.Publisher('targets/closest_pair', PoseArray,
                                  queue_size=10)
    rotated_pub = rospy.Publisher('targets/closest_pair/rotated', PoseArray,
                                  queue_size=10)
    intersect_pub = rospy.Publisher('targets/closest_pair/intersect',
                                    PointStamped, queue_size=10)
    corner_pub = rospy.Publisher('targets/corner', PoseStamped, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()
