#! /usr/bin/env python
"""Find a corner of the home plate and generate the transform from the home
coordinate frame to the odometry frame.
TODO: register a shutdown handler to set parameters for the transform option
 being used if this node ever dies?
"""
from __future__ import print_function

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
import rospy
import threading
import sys
import tf
import tf.transformations

from geometry_msgs.msg import (PointStamped, Pose, Pose2D, PoseStamped,
                               PoseArray, Quaternion)

from mobility import sync

from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from mobility.srv import (GetRoverNames, GetRoverNamesRequest,
                          GetRoverNamesResponse, SetHomeTransformOption,
                          SetHomeTransformOptionRequest,
                          SetHomeTransformOptionResponse)


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


home_xform_lock = threading.Lock()


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
     Type 1 | ^     ^     ^ |
     Corner | |     |     | |
            |      _|x      |
            |      y        |
            |-->   -->   -->|
            -----------------

            -----------------
     Type 2 |<--   <--   <--|
     Corner |     x__       |
            |       |y      |
            | ^     ^     ^ |
            | |     |     | |
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
        CORNER_TYPE_1: 0.876,
        CORNER_TYPE_2: 0.695
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
        1: math.pi,
        2: math.pi / 2,
        3: 0,
        4: -math.pi / 2
    }

    # Data structures holding the 2 Options for determining a corner number.
    # These hold the bounds that a corner's orientation must lie within for it
    # to be identified. The bounds will be shortened at each end by a term,
    # phi, representing the error in compass headings between rovers that are
    # facing exactly the same direction.

    BOUNDARY_OPT_1 = 1
    BOUNDARY_OPT_2 = 2

    # BOUNDS is defined as an OrderedDict because it's important to iterate
    # through the options in the order they're defined below. This way Option 1
    # will be the most likely option to be used.
    BOUNDS = OrderedDict({
        # Primary option, this is the most likely to be used, but a rover using
        # this option will switch to option 2 if another rover needs to use
        # option 2.
        BOUNDARY_OPT_1: {
            1: {  # type 1 corner
                1: {  # corner 1
                    # Does tag orientation need normalization to [0, 2PI]?
                    'norm_pos': True,
                    # (low bound, high bound)
                    'bounds': (math.pi / 2, 3 * math.pi / 2)
                },
                3: {  # corner 3
                    'norm_pos': False, 'bounds': (-math.pi / 2, math.pi / 2)
                }
            },
            2: {  # type 2 corner
                2: {  # corner 2
                    'norm_pos': False, 'bounds': (-math.pi, 0)
                },
                4: {  # corner 4
                    'norm_pos': False, 'bounds': (0, math.pi)
                }
            }
        },

        # The secondary option, this is less likely to be used, but all rovers
        # will switch to it if any rover needs to used this option.
        BOUNDARY_OPT_2: {
            1: {  # type 1 corner
                1: {  # corner 1
                    'norm_pos': False, 'bounds': (0, math.pi)
                },
                3: {  # corner 3
                    'norm_pos': False, 'bounds': (-math.pi, 0)
                }
            },
            2: {  # type 2 corner
                2: {  # corner 2
                    'norm_pos': True, 'bounds': (math.pi / 2, 3 * math.pi / 2)
                },
                4: {  # corner 4
                    'norm_pos': False, 'bounds': (- math.pi / 2, math.pi / 2)
                }
            }
        }
    })

    def __init__(self):
        # TODO: calculate a good number for Phi and make this accessible as
        #  a ROS parameter.
        # When oriented in exactly the same direction, two rovers may have
        # slightly different compass headings. This discrepancy, or error, makes
        # it important to define the corner orientation boundary regions
        # carefully and to make sure that all the rovers are using the same
        # boundary option to identify corners.
        self._phi = 0.25  # ~15 degrees, Where 15 degrees is the avg error

        # Wait until a corner is seen or another rover requests to use option 2
        # before making a choice.
        self._bounds_set = False
        self._bounds_option = None
        self._bounds = None

        rospy.init_node('find_corner')

        self.rover_name = rospy.get_namespace().strip('/')

        # Transform listener
        self._xform_l = tf.TransformListener(cache_time=rospy.Duration(secs=30))

        self._base_link_frame = rospy.get_param('base_link_frame')
        self._odom_frame = rospy.get_param('odom_frame')

        # Subscribers
        self._sub = rospy.Subscriber('targets',
                                     AprilTagDetectionArray, self._targets_cb)

        # Services
        self._opt_srv = rospy.Service('home_transform/set_option_num',
                                      SetHomeTransformOption,
                                      self._process_opt_req)

        # Publishers, for visualizing intermediate steps in RViz.
        self._closest_pub = rospy.Publisher('targets/closest_pair',
                                            PoseArray, queue_size=10)
        self._rotated_pub = rospy.Publisher('targets/closest_pair/rotated',
                                            PoseArray, queue_size=10)
        self._intersect_pub = rospy.Publisher('targets/closest_pair/intersect',
                                              PointStamped, queue_size=10)
        self._corner_pub = rospy.Publisher('targets/corner',
                                           PoseStamped, queue_size=10)
        self._home_pub = rospy.Publisher('home_pose',
                                         PoseStamped, queue_size=10)

        # ServiceProxy
        # This is used the first time a corner is seen to get a list of rovers
        # currently online. Then this rover can send a request to each remote
        # rover to use its preferred transform option. It should be safe to
        # assume this list is up to date by the time this rover sees a corner
        # of home for the first time.
        self.get_rovers = rospy.ServiceProxy('get_rover_names',
                                             GetRoverNames)

    @sync(home_xform_lock)
    def _process_opt_req(self,
                         req  # type: SetHomeTransformOptionRequest
                         ):
        # type: (...) -> SetHomeTransformOptionResponse
        """Process a request from a remote rover to set the bounds option for
        the group.

        This rover will accept or reject the request. Requests to use Option 2
        take priority.

        It will accept the request if:
            - The request is Option 1:

                - This rover is using Option 1 OR
                - This rover hasn't picked an option yet.

            - The request is Option 2:

                - This rover is using Option 1 OR
                - This rover is using Option 2 OR
                - This rover hasn't picked an option yet.

        It will reject the request if:
            - The request is Option 1 and this rover is using Option 2
        """
        response = SetHomeTransformOptionResponse()
        response.success = False

        rospy.loginfo('{}: transform option request received from {}'.format(
            self.rover_name, req.rover_name
        ))

        if req.option_num == HomeTransformGen.BOUNDARY_OPT_1:
            if not self._bounds_set or self._bounds_option == req.option_num:
                response.success = True

        elif req.option_num == HomeTransformGen.BOUNDARY_OPT_2:
            response.success = True

            # Use Option 2, but wait until the next time a corner is seen
            # to set the bounds, in case they need to be rotated slightly.
            if not self._bounds_set:
                msg = ('{}: request received from {} to use transform ' +
                       'Option 2. Will use Option 2 when a corner ' +
                       'is seen.').format(self.rover_name, req.rover_name)

                rospy.loginfo(msg)
                self._bounds_set = True
                self._bounds_option = HomeTransformGen.BOUNDARY_OPT_2
                self._bounds = None

            elif self._bounds_option == HomeTransformGen.BOUNDARY_OPT_1:
                # TODO: clear any data associated with the now invalid old
                #  transform if moving from Option 1 to Option 2?
                msg = ('{}: request received from {} to use transform ' +
                       'Option 2. Switching to Option 2.').format(
                    self.rover_name, req.rover_name
                )
                rospy.loginfo(msg)
                self._bounds_set = True
                self._bounds_option = HomeTransformGen.BOUNDARY_OPT_2
                self._bounds = None

        return response

    def _find_corner_tags(self, detections):
        # type: (List[PoseStamped]) -> Optional[Tuple[PoseStamped, PoseStamped]]
        """Find the closest pair of tags whose orientations in 2D space are
        approximately perpendicular and are inline with each other.

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

        if len(pose_buckets[1]) > 0:
            # We found 2 different orientations of tags.
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
            # pose3.position.x = (pose1.pose.position.x
            #                     + pose2.pose.position.x) / 2.
            # pose3.position.y = (pose1.pose.position.y
            #                     + pose2.pose.position.y) / 2.
            # pose3.position.z = (pose1.pose.position.z
            #                     + pose2.pose.position.z) / 2.
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
            self._intersect_pub.publish(msg)

            msg = PoseArray()
            msg.header = pose1.header
            msg.poses = [pose1.pose, pose2.pose]  # , pose3]
            self._closest_pub.publish(msg)

            return pose1, pose2
        else:
            # Publish empty PoseArray's so RViz doesn't show old data.
            msg = PoseArray()
            msg.header = detections[0].header
            self._closest_pub.publish(msg)
            self._rotated_pub.publish(msg)  # TODO: move into new function

            msg = PointStamped()
            msg.header = detections[0].header
            self._intersect_pub.publish(msg)

        return None

    def _classify_corner(self, intersected_pose, other_pose):
        # type: (PoseStamped, PoseStamped) -> Tuple[int, PoseStamped]
        """Return a corner type and it's calculated pose, given two inline,
        perpendicular home tag poses.
        """
        c_type = HomeTransformGen.CORNER_TYPE_2

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
            ps.pose.orientation = rotate_quaternion(ps.pose.orientation,
                                                    math.pi / 2, (0, 0, 1))

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

    def _set_bounds(self, opt_num, theta=0):
        # type: (int, float) -> None
        """Given an option number, set the corner orientation bounds dictionary
        for this rover, rotated by theta radians.
        """
        self._bounds_set = True
        self._bounds_option = opt_num
        self._bounds = copy.deepcopy(HomeTransformGen.BOUNDS[opt_num])

        rospy.loginfo(
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
                bounds['bounds'] = (lo_bound + theta, hi_bound + theta)

    def _set_bounds_option(self, corner_type, corner_pose, opt_num,
                           req_others=False):
        # type: (int, PoseStamped, int, bool) -> bool
        """Given the corner type, it's pose in the odometry frame, and the
        option number to use, attempt to set the corner orientation bounds,
        making the request to other rovers if necessary.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the odometry frame.
            opt_num: Which Option Number (1, 2) to try to use.
            req_others: Whether to make requests to remote rovers. It isn't
                necessary to make requests when another rover has already been
                instructed to switch to Option 2.

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
                theta = 0

                if ang - lo_bound < self._phi:
                    # Corner orientation is a little too close to the lower
                    # bound, so the rover will use this option, but rotate the
                    # bounds by -Phi radians, so there is a little more margin
                    # for error.
                    theta = -self._phi

                elif hi_bound - ang < self._phi:
                    # Corner orientation is a little too close to the upper
                    # bound, so the rover will use this option, but rotate the
                    # bounds by Phi radians, so there is a little more margin
                    # for error.
                    theta = self._phi

                if req_others:
                    # Make remote requests to other rovers to use this opt_num
                    # If any rover rejects return the result of calling this
                    # function again with req_others=False, and opt_num 2
                    rovers = self.get_rovers(GetRoverNamesRequest())
                    # Whether at least one remote service responded
                    made_contact = False

                    srvs = []  # type: List[Tuple[str, rospy.ServiceProxy]]
                    for rover in rovers.rovers:
                        if rover != self.rover_name:
                            srv = '/{}/home_transform/set_option_num'.format(
                                rover
                            )
                            srvs.append(
                                (rover,
                                 rospy.ServiceProxy(srv,
                                                    SetHomeTransformOption))
                            )

                    for srv in srvs:
                        req = SetHomeTransformOptionRequest()
                        req.option_num = opt_num
                        req.rover_name = self.rover_name

                        try:
                            response = srv[1](req)

                            if response.success:
                                made_contact = True
                            else:
                                msg = ('{}: {} rejected request to use home ' +
                                       'transform option number {}').format(
                                    self.rover_name, srv[0], opt_num
                                )
                                rospy.loginfo(msg)

                                return self._set_bounds_option(
                                    corner_type, corner_pose,
                                    HomeTransformGen.BOUNDARY_OPT_2,
                                    req_others=False
                                )
                        except rospy.ServiceException:
                            rospy.logerr(
                                ("{}: Couldn't make transform option service " +
                                 "request to {}").format(self.rover_name,
                                                         srv[0])
                            )
                            # TODO: should the rover have to successfully
                            #  call all the other rovers' services? How reliable
                            #  are service calls to remote machines?
                            # Hopefully the rover we couldn't connect to here
                            # has already made a request to us or will do so
                            # soon, and we can switch options then if necessary.
                            pass

                    if len(srvs) > 0 and not made_contact:
                        # Don't do anything if there are other rovers running
                        # and you couldn't reach at least one of them.
                        return False

                self._set_bounds(opt_num, theta=theta)
                return True

        return False

    def _choose_bounds_option(self, corner_type, corner_pose):
        # type: (int, PoseStamped) -> bool
        """Given the corner pose in the odometry frame, choose which bounds
        option to use and make sure all other rovers are ok with that choice.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the odometry frame.

        Returns:
            Whether the bounds options was set.
        """
        for opt_num, bounds_opt in HomeTransformGen.BOUNDS.items():
            if self._set_bounds_option(corner_type, corner_pose, opt_num,
                                       req_others=True):
                return True

        return False

    @sync(home_xform_lock)
    def _home_pose(self, corner_type, corner_pose):
        # type: (int, PoseStamped) -> Optional[PoseStamped]
        """Given a corner pose in the base_link frame, calculate the home
        plate's pose in the odometry frame.

        Args:
            corner_type: The type (1, 2) of corner.
            corner_pose: The corner's pose in the base_link frame.

        Returns:
            The home plate's pose in the odometry frame, or None, if it couldn't
                be calculated.
        """
        # This function acquires a lock when it's called to ensure the service
        # callback doesn't modify self._bounds_set and self._bounds after their
        # values are checked below, but before they're set inside
        # self._set_bounds(). Without the lock, it would be possible for this
        # rover to be in the process of setting everything up to use Option 1,
        # when another rover requests to use Option 2, briefly changing
        # self._bounds_option to the correct value before it gets overridden in
        # _set_bounds(). If that was the only rover requesting Option 2, this
        # rover would have missed the opportunity to switch, and would possibly
        # be generating transforms that don't agree with the rest of the group
        # (The transforms might also be fine, it depends on the home plate's
        # exact orientation relative to magnetic north, but it's not worth
        # the risk).
        try:
            self._xform_l.waitForTransform(self._odom_frame,
                                           corner_pose.header.frame_id,
                                           corner_pose.header.stamp,
                                           rospy.Duration(0.1))
            c_pose_odom = self._xform_l.transformPose(self._odom_frame,
                                                      corner_pose)
        except tf.Exception:
            # We can't do anything without having the transformed pose.
            rospy.logwarn(
                ('{}: Transform exception in ' +
                 'HomeTransformGen._home_pose().').format(self.rover_name)
            )
            return None

        if not self._bounds_set:
            if not self._choose_bounds_option(corner_type, c_pose_odom):
                return None
        elif self._bounds is None:
            # This can happen if another rover requested Option 2, which takes
            # priority, but this rover hasn't seen a corner yet, perhaps because
            # it's still waiting in the start queue.
            if not self._set_bounds_option(corner_type, c_pose_odom,
                                           self._bounds_option,
                                           req_others=False):
                return None

        home_pose = copy.deepcopy(c_pose_odom)

        theta = HomeTransformGen.TRANS_ROT[corner_type]

        angle_to_center = yaw_from_quaternion(rotate_quaternion(
            c_pose_odom.pose.orientation, theta, (0, 0, 1)
        ))
        home_pose.pose.position.x = (home_pose.pose.position.x
                                     + HomeTransformGen.CORNER_DIST
                                     * math.cos(angle_to_center))
        home_pose.pose.position.y = (home_pose.pose.position.y
                                     + HomeTransformGen.CORNER_DIST
                                     * math.sin(angle_to_center))

        corner_num = self._id_corner(corner_type, c_pose_odom)
        if corner_num is None:
            # TODO: put this log statement temporarily outside the if, just to
            #  make sure it works.
            rospy.logerr(("{}: couldn't identify corner number from corner " +
                          "with pose \n{}").format(self.rover_name,
                                                   c_pose_odom))
            return None

        rospy.loginfo_throttle(0.5,
                               '{}: looking at corner type {}, num {}'.format(
                                   self.rover_name, corner_type, corner_num
                               ))

        home_pose.pose.orientation = rotate_quaternion(
            home_pose.pose.orientation,
            HomeTransformGen.CORNER_ROT[corner_num],
            (0, 0, 1)
        )
        self._home_pub.publish(home_pose)

        return home_pose

    def _home_xform(self, home_pose):
        # type: (PoseStamped) -> TransformStamped
        """Given the home_plate's pose in the odometry frame, calculate the
        transform from the home frame to the odometry frame.
        """
        pass

    def _targets_cb(self, msg):
        # type: (AprilTagDetectionArray) -> None
        """Find corner of home."""
        detections = []  # type: List[PoseStamped]
        for detection in msg.detections:  # type: AprilTagDetection
            if detection.id == 256:
                try:
                    xpose = self._xform_l.transformPose(self._base_link_frame,
                                                        detection.pose)
                    r, p, y = euler_from_quaternion(xpose.pose.orientation)

                    if r < 0.25 and p < 0.25:  # ~15 degrees
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
                        detections.append(xpose)

                except tf.Exception:
                    pass

        # If a home tag is in view
        if len(detections) > 0:
            corner = self._find_corner_tags(detections)
            if corner is not None:
                # print(ps_to_p2d(corner[0]), ps_to_p2d(corner[1]))
                pose1, pose2 = corner
                int_ = intersected(pose1, pose2)

                if int_ == pose1:
                    cor_type, cor_pose = self._classify_corner(pose1, pose2)
                    rotated_poses = rotate(
                        pose1, pose2,
                        -yaw_from_quaternion(pose1.pose.orientation)
                    )
                    # p1_rot, p2_rot = rotated_poses
                else:
                    cor_type, cor_pose = self._classify_corner(pose2, pose1)
                    rotated_poses = rotate(
                        pose2, pose1,
                        -yaw_from_quaternion(pose2.pose.orientation)
                    )
                    # p2_rot, p1_rot = rotated_poses

                home_pose = self._home_pose(cor_type, cor_pose)
                if home_pose is not None:
                    # generate transform
                    pass

                self._corner_pub.publish(cor_pose)

                msg = PoseArray()
                msg.header = pose1.header
                msg.poses = [ps.pose for ps in rotated_poses]
                self._rotated_pub.publish(msg)
            else:
                # Publish empty corner pose
                msg = PoseStamped()
                msg.header = detections[0].header
                self._corner_pub.publish(msg)


def main():
    home_xform = HomeTransformGen()
    rospy.spin()


if __name__ == '__main__':
    main()
