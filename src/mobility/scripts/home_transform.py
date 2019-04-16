#! /usr/bin/env python
"""Find a corner of the home plate and generate the transform from the home
coordinate frame to the odometry frame.
TODO: Using the voting scheme to converge on a single set of corner orientation
 bounds, it's possible for a single rover to spoil the entire group's choice.
 This could happen by publishing a high authority message containing a choice
 that will never work using the home plate's current orientation. As it stands,
 this would prevent all of the rovers from being able to identify a corner's
 number and calculate the home frame's orientation.
"""
from __future__ import division, print_function

try:
    from typing import TYPE_CHECKING
    if TYPE_CHECKING:
        from typing import List, Optional, Tuple
except ImportError:
    pass

import angles
from collections import OrderedDict
import copy
import math
import numpy as np
import pprint
import random
import rospy
import threading
import sys
import tf
import tf.transformations
import traceback

from geometry_msgs.msg import (PointStamped, Pose, Pose2D, PoseStamped,
                               PoseArray, Quaternion, TransformStamped)
from nav_msgs.msg import Odometry
from std_msgs.msg import Time

from mobility.utils import is_moving
from mobility import sync

from apriltags2to1.msg import AprilTagDetection, AprilTagDetectionArray
from mobility.msg import HomeTransformAuthority
from swarmie_msgs.msg import Obstacle


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


def ps_to_pt_stamped(ps):
    # type: (PoseStamped) -> PointStamped
    """Convert a PoseStamped to a PointStamped, dropping the orientation data."""
    pnt = PointStamped(header=copy.deepcopy(ps.header))
    pnt.point.x = ps.pose.position.x
    pnt.point.y = ps.pose.position.y
    pnt.point.z = ps.pose.position.z

    return pnt


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
    p1 = (pose1.pose.position.x, pose1.pose.position.y)
    p2 = (pose2.pose.position.x, pose2.pose.position.y)

    line1 = (p1[0], p1[1],
             math.tan(yaw_from_quaternion(pose1.pose.orientation)))
    line2 = (p2[0], p2[1],
             math.tan(yaw_from_quaternion(pose2.pose.orientation)))

    p = intersect(line1, line2)
    if p is None:
        return None

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


def needs_positve_norm(lo, hi):
    """Given a low bound and a high bound for a range of angles, determine
    whether you need to normalize the angles to be [0, 2*PI].
    """
    return (angles.normalize_angle_positive(lo)
            < math.pi
            < angles.normalize_angle_positive(hi))


home_xform_lock = threading.Lock()


class BoundsStillUnsetError(Exception):
    """Raised when HomeTransformGen failed to set its corner orientation
    bounds.
    """
    pass


class HomeTransformGen:
    """Generate the transform from the home frame at the center of the home
    plate to the odom frame for this rover.

    Tag orientations on the home plate are::

             Corner 1                                       Corner 4
             <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <--

             <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <-- <--
            -------------                               -------------
     Type 1 |<-- <-- <--|<-- <-- <-- <-- <-- <-- <-- <--|<-- <-- <--| Type 2
     Corner |           |                               |           | Corner
            | |   |   | |                               | ^   ^   ^ |
            | v   v   v |                               | |   |   | |
            -------------                               -------------
              |   |   |                                   ^   ^   ^
              v   v   v                                   |   |   |
                                        |x
              |   |   |                 |                 ^   ^   ^
              v   v   v             ____|                 |   |   |
                                    y
              |   |   |                                   ^   ^   ^
              v   v   v                                   |   |   |
            -------------                               -------------
     Type 2 | |   |   | |                               | ^   ^   ^ | Type 1
     Corner | v   v   v |                               | |   |   | | Corner
            |           |                               |           |
            |--> --> -->|--> --> --> --> --> --> --> -->|--> --> -->|
            -------------                               -------------
             --> --> --> --> --> --> --> --> --> --> --> --> --> -->

             --> --> --> --> --> --> --> --> --> --> --> --> --> -->
             Corner 2                                       Corner 3

    Corners are numbered by the quadrant they lie within. The two corner types
    are numbered by the first quadrant they appear in.


    The two corner type's are defined to have different orientations::

            -----------------
     Type 1 |<--   <--   <--|
     Corner |      _|x      |
            |      y        |
            | |     |     | |
            | v     v     v |
            -----------------

            -----------------
     Type 2 | |     |     | |
     Corner | v     v     v |
            |     x__       |
            |       |y      |
            |-->   -->   -->|
            -----------------

    To determine which corner (1, 2, 3, 4) is in view when the rover sees home
    tags of 2 different orientations, we identify the type of corner and
    calculate its pose in the odometry coordinate frame. Those two pieces of
    information are enough to estimate the home plate's current pose in the
    odometry frame, allowing us to calculate the transform from the home frame
    to the odometry frame.
    """

    CORNER_TYPE_1 = 1
    CORNER_TYPE_2 = 2

    # Rotations in radians to orient each type of corner toward the center of
    # home. This is used to calculate the translation between the corner's pose
    # and the center of home. These are calculated geometrically using the home
    # plate dimensions (1m x 1m), the number of tags per side (21), and the
    # assumption that the calculated corner position is at the center of the 6
    # home tags defining a corner.
    TRANS_ROT = {
        CORNER_TYPE_1: -2.266,
        CORNER_TYPE_2: -2.447
    }

    # A corner's distance from the center of home, in meters. This is used to
    # calculate the translation to the center of home along with the angles
    # above. It's calculated geometrically using the same assumptions as the
    # rotation angles above.
    CORNER_DIST = 0.558

    # Angle by which each type of corner must be rotated to have the same
    # orientation as the home plate. This is used to calculate the home plate's
    # orientation.
    CORNER_ROT = {
        1: 0,
        2: -math.pi / 2,
        3: math.pi,
        4: math.pi / 2
    }

    # Data structures holding the 2 Options for determining a corner number.
    # These hold the bounds that a corner's orientation must lie within for it
    # to be identified. The bounds will be shortened at each end by a term,
    # phi, representing the error in compass headings between rovers that are
    # facing exactly the same direction.

    BOUNDARY_OPT_1 = 1
    BOUNDARY_OPT_2 = 2

    # Primary option. A rover is most likely to vote for this option, rotated
    # so each corner's orientation lies near the middle of each set of bounds.
    _BOUNDS_1 = {
        1: {  # type 1 corner
            1: {  # corner 1
                # Does tag orientation need normalization to [0, 2PI]?
                'norm_pos': False,
                # (low bound, high bound)
                'bounds': (-math.pi / 2, math.pi / 2)
            },
            3: {  # corner 3
                'norm_pos': True, 'bounds': (math.pi / 2, 3 * math.pi / 2)
            }
        },
        2: {  # type 2 corner
            2: {  # corner 2
                'norm_pos': False, 'bounds': (0, math.pi)
            },
            4: {  # corner 4
                'norm_pos': False, 'bounds': (-math.pi, 0)
            }
        }
    }

    # Secondary option. A rover is less likely to vote for this option, but will
    # do so if a corner's orientation using the primary option lies in the
    # ambiguous region after reducing the bounds by phi radians at each end.
    # Because the rovers rotate the bounds to put the corner's orientations near
    # the middle, a rover voting for Option 2 could also use a rotated Option 1,
    # if a higher authority rover votes for Option 1. It's most important that
    # the rovers all agree on the set of bounds to identify corners.
    _BOUNDS_2 = {
        1: {  # type 1 corner
            # corner 1
            1: _BOUNDS_1[2][4],
            # corner 3
            3: _BOUNDS_1[2][2]
        },
        2: {  # type 2 corner
            # corner 2
            2: _BOUNDS_1[1][1],
            # corner 4
            4: _BOUNDS_1[1][3]
        }
    }

    # BOUNDS is defined as an OrderedDict because it's important to iterate
    # through the options in the order they're defined below. This way Option 1
    # will be the most likely option to be used.
    BOUNDS = OrderedDict((
        (BOUNDARY_OPT_1, _BOUNDS_1),
        (BOUNDARY_OPT_2, _BOUNDS_2)
    ))

    def __init__(self):
        """Initialization."""
        rospy.init_node('home_transform')

        self.rover_name = rospy.get_namespace().strip('/')

        # It's important to wait for a message on the /clock topic so the
        # rostime API doesn't return time 0 below. If a message comes in here,
        # it should be safe to assume a message was received inside of rostime.
        if rospy.get_param('/use_sim_time', False):
            try:
                rospy.wait_for_message('/clock', Time, timeout=10.0)
            except rospy.ROSException:
                rospy.logwarn(("{}: timed out waiting for a message on " +
                               "/clock. This node's start time is likely " +
                               "to be 0.").format(self.rover_name))

        self._xform_vote = HomeTransformAuthority()
        self._xform_vote.rover_name = self.rover_name
        self._xform_vote.start_time = rospy.Time.now()
        self._xform_vote.random_priority = random.randrange(0, sys.maxint)
        self._xform_vote.boundary_option = (
            HomeTransformAuthority.BOUNDARY_OPT_UNSET
        )

        # Wait to set the bounds until a corner is seen or another rover
        # sets them.
        self._bounds = None

        # Keep track of previous obstacle message so it's only published once
        # when it changes.
        self._prev_obst_status = Obstacle.PATH_IS_CLEAR
        self._cur_odom = None  # type: Odometry
        self._home_point = None  # type: PointStamped

        # Transform listener and broadcaster
        self._xform_l = tf.TransformListener(cache_time=rospy.Duration(secs=30))
        self._xform_b = tf.TransformBroadcaster()

        self._home_xform = None

        self._base_link_frame = rospy.get_param('base_link_frame')
        self._odom_frame = rospy.get_param('odom_frame')
        self._home_frame = rospy.get_param('home_frame')
        self._xform_rate = rospy.get_param('~transform_publish_rate', 0.1)
        self._vote_rate = rospy.get_param('~vote_rate', 1.0)
        self._log_rate = rospy.get_param('~log_rate', 5.0)
        self._inside_home_threshold = rospy.get_param('~inside_home_threshold',
                                                      0.6)

        # TODO: calculate a good number for Phi.
        # When oriented in exactly the same direction, two rovers may have
        # slightly different compass headings. This discrepancy means it's
        # important to define the corner orientation boundary regions carefully
        # and to make sure that all the rovers are using the same boundary
        # option to identify corners.
        # Default value of ~15 degrees, Where 15 degrees is the avg error
        self._phi = rospy.get_param('~phi', 0.25)

        # Publishers
        self._xform_vote_pub = rospy.Publisher('/home_xform_vote',
                                               HomeTransformAuthority,
                                               queue_size=10)
        self._obstacle_pub = rospy.Publisher('obstacle',
                                             Obstacle,
                                             queue_size=1,
                                             latch=True)
        self._home_point_pub = rospy.Publisher('home_point',
                                               PointStamped,
                                               queue_size=10,
                                               latch=True)
        self._home_point_approx_pub = rospy.Publisher('home_point/approx',
                                                      PointStamped,
                                                      queue_size=10,
                                                      latch=True)

        # Additional publishers, for visualizing intermediate steps in RViz.
        self._closest_pub = rospy.Publisher('targets/closest_pair',
                                            PoseArray, queue_size=10)
        self._rotated_pub = rospy.Publisher('targets/closest_pair/rotated',
                                            PoseArray, queue_size=10)
        self._intersect_pub = rospy.Publisher('targets/closest_pair/intersect',
                                              PointStamped, queue_size=10)
        self._corner_pub = rospy.Publisher('targets/corner',
                                           PoseStamped, queue_size=10)
        self._home_pose_pub = rospy.Publisher('home_pose',
                                              PoseStamped, queue_size=10)

        # Subscribers
        self._odom_sub = rospy.Subscriber('odom/filtered',
                                          Odometry,
                                          self._odom_cb)
        self._targets_sub = rospy.Subscriber('targets',
                                             AprilTagDetectionArray,
                                             self._targets_cb)
        self._xform_auth_sub = rospy.Subscriber('/home_xform_vote',
                                                HomeTransformAuthority,
                                                self._home_xform_vote_cb)

        # Timers
        self._xform_timer = rospy.Timer(rospy.Duration(self._xform_rate),
                                        self._send_xform)
        self._xform_vote_timer = rospy.Timer(rospy.Duration(self._vote_rate),
                                             self._send_xform_vote)

    @sync(home_xform_lock)
    def _odom_cb(self, msg):
        # type: (Odometry) -> None
        """Store the most recent Odometry message."""
        self._cur_odom = msg

    def _log_vote(self, vote, action, reason):
        # type: (HomeTransformAuthority, str, str) -> None
        """Log an accepted/ignored transform authority message."""
        rospy.loginfo_throttle(self._log_rate,
                               ("[{}]: {} home transform vote from: {}.\n" +
                                "Reason: {}.\n" +
                                "My vote:\n{}\n" +
                                "Their vote:\n{}\n").format(self.rover_name,
                                                            action,
                                                            vote.rover_name,
                                                            reason,
                                                            self._xform_vote,
                                                            vote))

    def _log_accepted_vote(self, vote, reason):
        # type: (HomeTransformAuthority, str) -> None
        """Log an accepted transform authority message."""
        self._log_vote(vote, "Accepting", reason)

    def _log_ignored_vote(self, vote, reason):
        # type: (HomeTransformAuthority, str) -> None
        """Log an ignored transform authority message."""
        self._log_vote(vote, "Ignoring", reason)

    def _has_higher_authority_than(self, vote):
        # type: (HomeTransformAuthority) -> bool
        """Return true if this rover has higher authority than the rover sending
        the vote.
        """
        if vote.start_time == self._xform_vote.start_time:
            return self._xform_vote.random_priority < vote.random_priority

        return self._xform_vote.start_time < vote.start_time

    def _accept_vote(self, vote, reason):
        # type: (HomeTransformAuthority, str) -> None
        """Accept the vote and set or update this rover's bounds accordingly."""
        # TODO: log the accepted vote every time?
        self._log_accepted_vote(vote, reason)

        if self._are_bounds_set():
            if vote.boundary_option != self._xform_vote.boundary_option:
                rospy.logwarn(("{}: Switching home transform boundary option " +
                               "from Option {} to Option {}.").format(
                    self.rover_name, self._xform_vote.boundary_option,
                    vote.boundary_option
                ))

            elif (abs(vote.boundary_theta - self._xform_vote.boundary_theta)
                    > math.pi / 4):
                rospy.logwarn(("{}: New home transform boundaries are " +
                               "rotated {} radians. This is significantly " +
                               "different from old rotation of " +
                               "{} radians.").format(
                    self.rover_name, vote.boundary_theta,
                    self._xform_vote.boundary_theta
                ))

        self._set_bounds(vote.boundary_option, vote.boundary_theta)

    @sync(home_xform_lock)
    def _home_xform_vote_cb(self, msg):
        # type: (HomeTransformAuthority) -> None
        """Process a vote received from another rover."""
        if msg.rover_name == self.rover_name:
            # Ignore votes received from the local publisher.
            return

        if msg.boundary_option == HomeTransformAuthority.BOUNDARY_OPT_UNSET:
            # Ignore votes from rovers with unset bounds.
            self._log_ignored_vote(msg, reason="This vote's bounds are unset")
            return

        if (self._xform_vote.boundary_option ==
                HomeTransformAuthority.BOUNDARY_OPT_UNSET):
            # Accept this vote and set this rover's bounds using the vote data.
            # This is vote is accepted regardless of the voting rover's
            # authority compared to this rover. This way the group's highest
            # authority rover can begin sending votes with bounds information as
            # soon as any rover sees a corner of home, allowing the group to
            # converge on a single choice as quickly as possible.
            self._accept_vote(msg, reason="My vote bounds are unset")
            return

        if self._has_higher_authority_than(msg):
            # Ignore votes from rovers with lower authority than this rover.
            self._log_ignored_vote(msg,
                                   reason="Vote authority is lower than mine")
            return

        # Accept the vote and set this rover's bounds
        self._accept_vote(msg, reason="Vote authority is higher than mine")

    @sync(home_xform_lock)
    def _send_xform_vote(self, _event):
        """Publish this rover's transform authority if it's the current leader,
        or if it hasn't heard from the leader recently.
        """
        self._xform_vote_pub.publish(self._xform_vote)

    @staticmethod
    def _find_corner_tags(pose_bucket1,  # type: List[PoseStamped]
                          pose_bucket2,  # type: List[PoseStamped]
                          ):
        # type: (...) -> Optional[Tuple[PoseStamped, PoseStamped]]
        """Find the closest pair of tags whose orientations in 2D space are
        approximately perpendicular and are inline with each other.
        """
        if pose_bucket1 and pose_bucket2:
            # We found 2 different orientations of tags.
            return closest_inline_pair(pose_bucket1, pose_bucket2)

        return None

    def _classify_corner(self, intersected_pose, other_pose):
        # type: (PoseStamped, PoseStamped) -> Tuple[int, PoseStamped]
        """Return a corner type and it's calculated pose, given two inline,
        perpendicular home tag poses.
        """
        c_type = HomeTransformGen.CORNER_TYPE_2

        # The angle to rotate the intersected pose by in order to get the
        # corner's orientation.
        theta = math.pi

        ps = copy.deepcopy(intersected_pose)
        ps.pose.position.x = ((intersected_pose.pose.position.x
                               + other_pose.pose.position.x) / 2.)
        ps.pose.position.y = ((intersected_pose.pose.position.y
                               + other_pose.pose.position.y) / 2.)
        ps.pose.position.z = ((intersected_pose.pose.position.z
                               + other_pose.pose.position.z) / 2.)

        # Rotate the other pose about the intersected pose by the angle required
        # to give the intersected pose a zero heading in the current frame of
        # reference.
        int_rot, other_rot = rotate(
            intersected_pose, other_pose,
            -yaw_from_quaternion(intersected_pose.pose.orientation)
        )

        # Type 1 corner
        if yaw_from_quaternion(other_rot.pose.orientation) > 0:
            c_type = HomeTransformGen.CORNER_TYPE_1
            theta = -math.pi / 2

        ps.pose.orientation = rotate_quaternion(ps.pose.orientation,
                                                theta, (0, 0, 1))

        return c_type, ps

    def _id_corner(self, corner_type, corner_pose):
        # type: (int, PoseStamped) -> Optional[int]
        """Given a corner's type and its pose in the odometry frame, identify
        the corner number (1, 2, 3, 4).

        self._bounds must be set before calling.

        Args:
            corner_type: the type (1, 2) of corner.
            corner_pose: the corner pose in the odometry frame.

        Returns:
            The corner number (1, 2, 3, 4) or None if the corner can't be
                identified (shouldn't happen).
        """
        yaw = yaw_from_quaternion(corner_pose.pose.orientation)

        for corner_num, bounds in self._bounds[corner_type].items():
            ang = yaw

            if bounds['norm_pos']:
                ang = angles.normalize_angle_positive(ang)

            if bounds['bounds'][0] < ang < bounds['bounds'][1]:
                return corner_num

        return None

    def _are_bounds_set(self):
        """Return true if the this rover's bounds have been set."""
        return (self._xform_vote.boundary_option
                != HomeTransformAuthority.BOUNDARY_OPT_UNSET)

    def _set_bounds(self, opt_num, theta=0):
        # type: (int, float) -> None
        """Given an option number, set the corner orientation bounds dictionary
        for this rover, rotated by theta radians.
        """
        self._xform_vote.boundary_option = opt_num
        self._xform_vote.boundary_theta = theta
        self._bounds = copy.deepcopy(HomeTransformGen.BOUNDS[opt_num])

        rospy.loginfo_throttle(
            self._log_rate,
            '{}: using home transform Option {}, rotated {} rad'.format(
                self.rover_name, opt_num, theta
            )
        )

        for _corner_type, corners in self._bounds.items():
            for _corner_num, bounds in corners.items():
                # Constrain the bounds by Phi radians on both ends.
                lo_bound = bounds['bounds'][0] + self._phi
                hi_bound = bounds['bounds'][1] - self._phi

                # Rotate the constrained bounds by theta radians.
                lo_bound += theta
                hi_bound += theta

                # Re-determine whether positive normaliazation (0 to 2*PI) is
                # necessary after the rotation.
                if needs_positve_norm(lo_bound, hi_bound):
                    bounds['norm_pos'] = True
                    bounds['bounds'] = (
                        angles.normalize_angle_positive(lo_bound),
                        angles.normalize_angle_positive(hi_bound)
                    )
                else:
                    bounds['norm_pos'] = False
                    bounds['bounds'] = (
                        angles.normalize_angle(lo_bound),
                        angles.normalize_angle(hi_bound)
                    )

    def _set_bounds_option(self, corner_type, corner_pose, opt_num):
        # type: (int, PoseStamped, int) -> bool
        """Given the corner type, it's pose in the odometry frame, and the
        option number to use, set the corner orientation bounds.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the odometry frame.
            opt_num: Which Option Number (1, 2) to try to use.

        Returns:
            Whether the bounds option has been set.
        """
        yaw = yaw_from_quaternion(corner_pose.pose.orientation)

        for (_corner_num,
             bounds) in HomeTransformGen.BOUNDS[opt_num][corner_type].items():
            ang = yaw

            if bounds['norm_pos']:
                ang = angles.normalize_angle_positive(ang)

            # Constrain the bounds by Phi radians on both ends. This is also
            # how large the bounds regions will be once they're chosen.
            lo_bound = bounds['bounds'][0] + self._phi
            hi_bound = bounds['bounds'][1] - self._phi

            if lo_bound < ang < hi_bound:
                # Calculate theta, the angle to rotate the bounds, such that
                # the corner's orientation is in their center.
                midpt = (lo_bound + hi_bound) / 2.
                theta = angles.shortest_angular_distance(midpt, ang)

                self._set_bounds(opt_num, theta=theta)
                return True

        return False

    def _choose_bounds_option(self, corner_type, corner_pose):
        # type: (int, PoseStamped) -> None
        """Given the corner pose in the odometry frame, choose which bounds
        option to use.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the odometry frame.

        Raises:
            BoundsStillUnsetError: If we failed to set a bounds option.
        """
        for opt_num, bounds_opt in HomeTransformGen.BOUNDS.items():
            if self._set_bounds_option(corner_type, corner_pose, opt_num):
                return

        raise BoundsStillUnsetError(
            'Unable to set the bounds using either Bounds Option'
        )

    @staticmethod
    def _translate_to_center(corner_type, corner_pose):
        # type: (int, PoseStamped) -> PoseStamped
        """Given a corner type and its pose in the odometry frame, translate
        the corner's position to the center of home.
        """
        home_pose = copy.deepcopy(corner_pose)

        theta = HomeTransformGen.TRANS_ROT[corner_type]

        angle_to_center = yaw_from_quaternion(rotate_quaternion(
            corner_pose.pose.orientation, theta, (0, 0, 1)
        ))
        home_pose.pose.position.x = (home_pose.pose.position.x
                                     + HomeTransformGen.CORNER_DIST
                                     * math.cos(angle_to_center))
        home_pose.pose.position.y = (home_pose.pose.position.y
                                     + HomeTransformGen.CORNER_DIST
                                     * math.sin(angle_to_center))

        return home_pose

    @sync(home_xform_lock)
    def _home_pose(self, corner_type, corner_pose):
        # type: (int, PoseStamped) -> Tuple[PointStamped, Optional[PoseStamped]]
        """Given a corner pose in the base_link frame, calculate the home
        plate's pose in the odometry frame.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the base_link frame.

        Returns:
            A tuple containing the home plate's position in the odometry frame
                and it's Pose in the odometry frame, if it could be calculated.

        Raises:
            tf.Exception: If the internal transform here fails.
            BoundsStillUnsetError: If the bounds were unset prior to this call
                and they couldn't be set here.
        """
        # This function acquires a lock to ensure the vote callback doesn't
        # modify self._bounds after its value is checked below, but before it's
        # set inside self._set_bounds(). Without the lock, it would be possible
        # for this rover to be in the process of setting everything up to use
        # Option 1, when a higher authority rover votes to use Option 2, briefly
        # changing self._xform_vote.boundary_option to the correct value before
        # it gets overridden in self._set_bounds(). This would allow this rover
        # to temporarily ignore the vote for Option 2, although it would receive
        # another vote shortly from that rover or another rover who properly
        # accepted the vote.
        self._xform_l.waitForTransform(self._odom_frame,
                                       corner_pose.header.frame_id,
                                       corner_pose.header.stamp,
                                       rospy.Duration(0.1))
        c_pose_odom = self._xform_l.transformPose(self._odom_frame,
                                                  corner_pose)

        if not self._are_bounds_set():
            self._choose_bounds_option(corner_type, c_pose_odom)

        home_pose = self._translate_to_center(corner_type, c_pose_odom)
        home_point = ps_to_pt_stamped(home_pose)

        corner_num = self._id_corner(corner_type, c_pose_odom)
        if corner_num is None:
            rospy.logerr_throttle(
                self._log_rate,
                ("{}: Couldn't identify a Type {} corner with pose:\n{}\n" +
                 "Current bounds are:\n{}").format(
                    self.rover_name, corner_type,
                    ps_to_p2d(c_pose_odom),
                    pprint.pformat(self._bounds, width=70)
                )
            )
            return home_point, None

        rospy.loginfo_throttle(self._log_rate,
                               '{}: looking at corner type {}, num {}'.format(
                                   self.rover_name, corner_type, corner_num
                               ))

        home_pose.pose.orientation = rotate_quaternion(
            home_pose.pose.orientation,
            HomeTransformGen.CORNER_ROT[corner_num],
            (0, 0, 1)
        )

        return home_point, home_pose

    @sync(home_xform_lock)
    def _gen_home_xform(self, home_pose):
        # type: (PoseStamped) -> None
        """Given the home_plate's pose in the odometry frame, calculate the
        transform from the home frame to the odometry frame.
        """
        # Considering the odometry frame as the parent frame, the odom -> home
        # transform is the same as the home plate's pose in the odometry frame.
        # However, we need the home frame to be the parent frame for tf. We need
        # to calculate the home -> odom transform. This transform is the inverse
        # of the implied transform contained in the home plate's pose.
        transform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix([home_pose.pose.position.x,
                                                   home_pose.pose.position.y,
                                                   home_pose.pose.position.z]),
            tf.transformations.quaternion_matrix([home_pose.pose.orientation.x,
                                                  home_pose.pose.orientation.y,
                                                  home_pose.pose.orientation.z,
                                                  home_pose.pose.orientation.w])
        )

        inverse = tf.transformations.inverse_matrix(transform)
        trans = tf.transformations.translation_from_matrix(inverse)
        q = tf.transformations.quaternion_from_matrix(inverse)

        xform = TransformStamped()
        xform.header.frame_id = self._home_frame
        xform.child_frame_id = home_pose.header.frame_id

        xform.transform.translation.x = trans[0]
        xform.transform.translation.y = trans[1]

        xform.transform.rotation.x = q[0]
        xform.transform.rotation.y = q[1]
        xform.transform.rotation.z = q[2]
        xform.transform.rotation.w = q[3]

        self._home_xform = xform

    def _send_xform(self, _event):
        # type: (rospy.timer.TimerEvent) -> None
        """Callback for the Timer. Send the current home transform. Using a
        Timer makes sure the transform is broadcast even when a corner of home
        isn't visible.
        """
        if self._home_xform is None:
            return

        # Stamp the transform here so everyone thinks it's up to date.
        self._home_xform.header.stamp = rospy.Time().now()
        self._xform_b.sendTransformMessage(self._home_xform)

    @staticmethod
    def _is_bad_orientation(tag_yaw):
        # type: (float) -> bool
        """Return True if the AprilTag's orientation in the base_link frame is
        such that the rover may be inside of home.

        Args:
            tag_yaw: The tag's yaw in the base_link frame.
        """
        yaw_threshold = 1.3  # radians

        tag_yaw -= math.pi / 2
        tag_yaw = angles.normalize_angle(tag_yaw)

        return abs(tag_yaw) < yaw_threshold

    @sync(home_xform_lock)
    def _is_rover_inside_home(self, good_yaw_count, bad_yaw_count):
        # type: (float, float) -> bool
        """Return True if the rover appears to be inside of home.

        This method first compares the good and bad counts, and if they're
        suspicious, compares the rover's current position with the home
        position.

        Using the home tag orientations is useful so the rover can only think
        it's inside of home if a home tag is in view, but it tends to generate
        false positives. Comparing the rover's and home's locations helps reduce
        the number of false positives.

        Args:
            good_yaw_count: The count of acceptably-oriented home tags.
            bad_yaw_count: The count of dangerously-oriented home tags.
        """
        if bad_yaw_count == 0 or bad_yaw_count < good_yaw_count:
            return False

        if self._cur_odom is not None and self._home_point is not None:
            x_dist = (self._cur_odom.pose.pose.position.x
                      - self._home_point.point.x)
            y_dist = (self._cur_odom.pose.pose.position.y
                      - self._home_point.point.y)

            return (abs(x_dist) < self._inside_home_threshold and
                    abs(y_dist) < self._inside_home_threshold)

        return True

    @staticmethod
    def _get_bucketed_tags(detections  # type: List[PoseStamped]
                           ):
        # type: (...) -> Tuple[List[PoseStamped], List[PoseStamped]]
        """Place tag poses into buckets by their orientation.

        Detections should not be empty and should only contain home tags (id ==
        256).
        """
        pose_buckets = [[], []]  # type: List[List[PoseStamped], List[PoseStamped]]
        pose_buckets[0].append(detections[0])

        for detection in detections[1:]:  # type: PoseStamped
            angle_between = abs(angles.shortest_angular_distance(
                yaw_from_quaternion(pose_buckets[0][0].pose.orientation),
                yaw_from_quaternion(detection.pose.orientation)
            ))

            if angle_between < 0.25:  # ~15 deg
                pose_buckets[0].append(detection)
            elif angle_between > math.pi / 2 - 0.25:
                # PI/2 minus a nominal ~15 deg.
                pose_buckets[1].append(detection)

        return pose_buckets[0], pose_buckets[1]

    def _find_approx_home_pos(self, pose_bucket1, pose_bucket2):
        # type: (List[PoseStamped], List[PoseStamped]) -> None
        """Find the approximate home location, given two lists of home tag
        poses, bucketed by their orientation.

        Buckets should contain only home tags (id == 256), and at least one
        bucket should not be empty.
        """
        if self._cur_odom is not None and is_moving(self._cur_odom):
            return

        approx_corner = False

        if pose_bucket1 and pose_bucket2:
            approx_corner = True
            if (angles.shortest_angular_distance(
                    yaw_from_quaternion(pose_bucket1[0].pose.orientation),
                    yaw_from_quaternion(pose_bucket2[0].pose.orientation))
                    < 0):
                detections = pose_bucket1
            else:
                detections = pose_bucket2
        else:
            detections = pose_bucket1
            if len(pose_bucket2) > len(detections):
                detections = pose_bucket2

        if approx_corner:
            sum_ = reduce(lambda p, n: (p[0] + n.pose.position.x,
                                        p[1] + n.pose.position.y),
                          detections,
                          (0, 0))

            pose = copy.deepcopy(detections[0])
            pose.pose.position.x = sum_[0] / len(detections)
            pose.pose.position.y = sum_[1] / len(detections)
            pose.pose.orientation = rotate_quaternion(pose.pose.orientation,
                                                      math.pi, (0, 0, 1))
        else:
            pose = max(detections, key=lambda d: d.pose.position.y)

        pose.pose.position.z = 0

        try:
            self._xform_l.waitForTransform(self._odom_frame,
                                           pose.header.frame_id,
                                           pose.header.stamp,
                                           rospy.Duration(0.15))
            xpose = self._xform_l.transformPose(self._odom_frame, pose)
        except tf.Exception as e:
            rospy.logwarn_throttle(
                self._log_rate,
                ('{}: Transform exception in ' +
                 'HomeTransformGen._find_approx_home_pos():\n{}').format(
                    self.rover_name, e
                )
            )
            return

        home_pt = PointStamped()
        home_pt.header.frame_id = self._odom_frame
        home_pt.header.stamp = rospy.Time.now()

        if approx_corner:
            home_pose = self._translate_to_center(
                HomeTransformGen.CORNER_TYPE_1,
                xpose
            )
            home_pt.point = home_pose.pose.position
        else:
            yaw = yaw_from_quaternion(xpose.pose.orientation) + math.pi / 2
            home_pt.point.x = xpose.pose.position.x + 0.5 * math.cos(yaw)
            home_pt.point.y = xpose.pose.position.y + 0.5 * math.sin(yaw)

        home_pt.point.z = 0.0

        self._home_point_approx_pub.publish(home_pt)

    def _targets_cb(self, msg):
        # type: (AprilTagDetectionArray) -> None
        """Find corner of home."""
        next_obst_status = Obstacle.PATH_IS_CLEAR

        # Counts of acceptable and not-acceptable orientations of home tags in
        # the base_link frame. If there are more bad orientations than good
        # ones, it's likely the rover is inside of the home ring, and we'll
        # consider more closely using the rover's current position in the
        # home frame.
        good_yaw_count = 0
        bad_yaw_count = 0

        detections = []  # type: List[PoseStamped]
        for detection in msg.detections:  # type: AprilTagDetection
            if detection.id == 256:
                try:
                    xpose = self._xform_l.transformPose(self._base_link_frame,
                                                        detection.pose)
                    r, p, y = euler_from_quaternion(xpose.pose.orientation)

                    if abs(r) < 0.25 and abs(p) < 0.25:  # ~15 degrees
                        # TODO: is this a good number?
                        # TODO: could filtering ever be a problem, since the
                        #  transform is going to the base link frame, if the
                        #  rover was yawed or pitched while driving over a cube?
                        #  I don't know if transforming into the odom frame
                        #  would help in that case either, because
                        #  robot_localization is in 2D mode.
                        # Occasionally a detection's pose estimate isn't
                        # calculated very well, and its orientation has a very
                        # large roll and/or pitch component, when we know the
                        # tags are flat on the ground.
                        if self._is_bad_orientation(y):
                            bad_yaw_count += 1
                        else:
                            good_yaw_count += 1

                        detections.append(xpose)

                except tf.Exception:
                    pass

        # If a home tag is in view
        if len(detections) > 0:
            if self._is_rover_inside_home(good_yaw_count, bad_yaw_count):
                next_obst_status |= Obstacle.INSIDE_HOME

            pose_buckets = self._get_bucketed_tags(detections)
            self._find_approx_home_pos(*pose_buckets)

            corner = self._find_corner_tags(*pose_buckets)
            if corner is not None:
                next_obst_status |= Obstacle.HOME_CORNER

                pose1, pose2 = corner
                int_ = intersected(pose1, pose2)

                if int_ == pose1:
                    cor_type, cor_pose = self._classify_corner(pose1, pose2)
                    rotated_poses = rotate(
                        pose1, pose2,
                        -yaw_from_quaternion(pose1.pose.orientation)
                    )
                else:
                    cor_type, cor_pose = self._classify_corner(pose2, pose1)
                    rotated_poses = rotate(
                        pose2, pose1,
                        -yaw_from_quaternion(pose2.pose.orientation)
                    )

                try:
                    self._home_point, home_pose = self._home_pose(cor_type,
                                                                  cor_pose)
                except tf.Exception as e:
                    rospy.logwarn(
                        ('{}: Transform exception raised by ' +
                         'HomeTransformGen._home_pose():\n{}').format(
                            self.rover_name, e
                        )
                    )
                    return
                except BoundsStillUnsetError:
                    rospy.logerr_throttle(self._log_rate,
                                          'BoundsStillUnsetError:\n{}'.format(
                                              traceback.format_exc()
                                          ))
                    return

                if home_pose is not None:
                    # generate transform
                    self._gen_home_xform(home_pose)
                    self._home_pose_pub.publish(home_pose)

                self._home_point_pub.publish(self._home_point)

                # Publish intermediate steps
                self._corner_pub.publish(cor_pose)

                msg = PoseArray()
                msg.header = pose1.header
                msg.poses = [pose1.pose, pose2.pose]
                self._closest_pub.publish(msg)

                msg.poses = [ps.pose for ps in rotated_poses]
                self._rotated_pub.publish(msg)

                msg = PointStamped()
                msg.header = pose1.header
                msg.point.x = int_.pose.position.x
                msg.point.y = int_.pose.position.y
                msg.point.z = int_.pose.position.z
                self._intersect_pub.publish(msg)
            else:
                # If no corner is in view, publish empty PoseArrays so RViz
                # doesn't show old data.
                self._closest_pub.publish(header=detections[0].header)
                self._rotated_pub.publish(header=detections[0].header)
        else:
            # If no home tags are in view, publish empty PoseArrays.
            msg = PoseArray()
            msg.header.stamp = rospy.Time().now()
            msg.header.frame_id = self._base_link_frame
            self._closest_pub.publish(msg)
            self._rotated_pub.publish(msg)

        # Publish obstacle messages for the home plate's corner, if necessary.
        if next_obst_status != self._prev_obst_status:
            msg = Obstacle()
            msg.msg = next_obst_status
            msg.mask = Obstacle.HOME_CORNER | Obstacle.INSIDE_HOME
            self._obstacle_pub.publish(msg)

        self._prev_obst_status = next_obst_status


def main():
    home_xform = HomeTransformGen()
    rospy.spin()


if __name__ == '__main__':
    main()
