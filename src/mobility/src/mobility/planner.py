#! /usr/bin/env python
"""
Local Planner
todo: raise HomeExceptions from Planner.drive_to()?
todo: is reusing PathException in Planner.drive_to() ok?
todo: is using swarmie.drive(), turn, etc with throw=False and just geting the MoveResult ok?
todo: refactor to make target avoidance optional?
todo: is setting x pos = 0 good in _get_angle_and_dist_to_avoid() when radius is too small?
"""
from __future__ import print_function

import argparse
import sys
import math
import rospy
import tf
import angles

from geometry_msgs.msg import PoseStamped, Pose2D, Point
from mapping.srv import GetNavPlanResponse

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import (swarmie, Location, HomeException, TagException,
                              PathException, ObstacleException,
                              InsideHomeException)
from mobility import utils


class Planner:
    """Local Planner

    Uses Swarmie API and map/get_plan service to navigate the rover to a
    goal position, within a given tolerance, without hitting blocks or sonar
    obstacles. Hopefully, anyway.
    """

    # Global driving state variables
    STATE_IDLE = 0
    STATE_DRIVE = 1
    STATE_AVOID_LEFT = 2
    STATE_AVOID_RIGHT = 3
    STATE_AVOID_REVERSE = 4

    DISTANCE_OK = 0.5
    PATHWAY_EDGE_DIST = 0.33  # meters

    # Number of consecutive times the rover can drive or turn and receive a
    # MoveResult other than SUCCESS before giving up. This is a relatively
    # large number because the first waypoint in a nav plan can be 5 or 10
    # meters away, and any time the rover comes upon a sonar or tag obstacle,
    # its fail_count is incremented. It's not uncommon for the rover to come
    # across a scattering of cubes that it must spend some time avoiding.
    # During that time, it needs some leeway to drive slightly off course
    # as a part of its avoidance behavior. It seems to work for the most part.
    # FAIL_COUNT_LIMIT = 10

    def __init__(self, use_rviz_nav_goal=False,
                 avoid_targets=False, avoid_home=False):
        """Create a new Planner.
        Args:
        * swarmie - the Swarmie object you've already instantiated
        * use_rviz_nav_goal - for testing/debugging. Subscribes to RViz
          nav_goals published on the topic /rovername/goal, and navigates to
          those goals using Planner.drive_to()
        * avoid_targets - for testing/debugging. Whether the rover should avoid
          targets in the RViz nav_goal callback. Not used unless
          use_rviz_nav_goal == True
        * avoid_home - for testing/debugging. Whether the rover should avoid
          home in the RViz nav_goal callback. Not used unless
          use_rviz_nav_goal == True
        """
        self.rovername = swarmie.rover_name
        self.current_state = Planner.STATE_IDLE
        self.prev_state = Planner.STATE_IDLE
        self.cur_loc = Location(None)
        self.goal = Pose2D()
        self.plan = GetNavPlanResponse()
        self.tolerance = 0.0
        self.result = MoveResult.SUCCESS
        self.fail_count = 0
        self.avoid_targets = True
        self.avoid_home = False
        # self.ignore = Obstacle.IS_SONAR  # used for turns and _clear()

        # Subscribers
        if use_rviz_nav_goal:
            print('Using RViz 2D Nav Goals')
            if avoid_home:
                print('Avoiding home during this run.')
            if avoid_targets:
                print('Avoiding targets during this run.')
            self._nav_goal_sub = rospy.Subscriber(
                'goal',
                PoseStamped,
                self._rviz_nav_goal_cb,
                callback_args=(avoid_targets, avoid_home),
                queue_size=1
            )

    def _get_angle_and_dist_to_avoid(self, detection, direction='left'):
        """The safe driving pathway is defined to be the space between two
        lines running parallel to the rover's x-axis, at y=0.33m and y=-0.33m.
        Driving between these lines gives the wheels about 10cm of clearance on
        either side. These lines also conveniently intersect the upper-left and
        upper-right corners of the camera's field of view, but are not visible
        in the frame. So, to avoid driving over a cube, the rover should make
        sure it drives within these lines (drives without a tag in view).
        This function helps decide which direction to turn and how far to
        drive to get a tag out of view and go around it efficiently.

        Returns the angle the rover should turn in order to get the tag out
        of its field of view.

        |     tag->*   .|<-path_edge_point
        |         /   . |
        |        /   .  |
        |       /   .   |<- path edge
        |       | <-theta, the angle to turn. Rover would turn left(+theta)
        |     __|_.     |                     in this instance.
        |  []|  |. |[]  |
        |    |  .  |    |
        |    |rover|    |
        |  []|_____|[]  |

                |
                |+x
          +y____|

        Args:
        * detection - the PoseStamped detection in the /camera_link frame.
          This detection should be the detection whose position it makes the
          most sense to make a turn decision relative to.
        * direction - the direction you want to turn to avoid the tag.
          'left' turn counterclockwise, returns a positive angle.
          'right' turn clockwise, returns a positive angle

        Returns:
        * angle - the angle in radians to turn.
        * distance - the distance in meters to drive.
        """
        OVERSHOOT_DIST = 0.20  # meters, distance to overshoot target by
        base_link_pose = swarmie.transform_pose('/base_link', detection.pose)
        radius = math.sqrt(base_link_pose.pose.position.x ** 2
                           + base_link_pose.pose.position.y ** 2)
        tag_point = Point(x=base_link_pose.pose.position.x,
                          y=base_link_pose.pose.position.y)

        path_edge_point = Point()
        # solve for x given the radius and y-coord of a point on a circle
        # Just set x to zero if radius is too small (if tag is too close to
        # the rover. Protects math.sqrt() from evaluating a negative number.
        if radius > Planner.PATHWAY_EDGE_DIST:
            path_edge_point.x = math.sqrt(radius ** 2
                                          - Planner.PATHWAY_EDGE_DIST ** 2)
        else:
            path_edge_point.x = 0
        path_edge_point.y = Planner.PATHWAY_EDGE_DIST
        if direction == 'left':
            path_edge_point.y *= -1

        return (-self._angle_between(tag_point, path_edge_point),
                path_edge_point.x + OVERSHOOT_DIST)

    def get_angle_to_face_detection(self, detection):
        """Get the angle required to turn and face a detection to put it in
        the center of the camera's field of view.

        Args:
        * detection - the PoseStamped detection in the /camera_link frame.
          This detection should be the detection whose position it makes the
          most sense to make a turn decision relative to.

        Returns:
        * angle - the angle in radians to turn.
        """
        base_link_pose = swarmie.transform_pose('/base_link', detection.pose)
        radius = math.sqrt(base_link_pose.pose.position.x ** 2
                           + base_link_pose.pose.position.y ** 2)
        tag_point = Point(x=base_link_pose.pose.position.x,
                          y=base_link_pose.pose.position.y)

        center_of_view_point = Point()
        # solve for x given the radius and y-coord of a point on a circle
        # y-coord is zero in this special case
        center_of_view_point.x = math.sqrt(radius ** 2)
        center_of_view_point.y = 0

        return -self._angle_between(tag_point, center_of_view_point)

    def face_home_tag(self):
        """Turn and face the home tag nearest the center of view if we
        see one. Does nothing if no home tag is seen."""
        home_detections = self._sort_tags_nearest_center(
            swarmie.get_latest_targets(id=256)
        )
        if len(home_detections) > 0:
            angle = self.get_angle_to_face_detection(home_detections[0])
            current_heading = swarmie.get_odom_location().get_pose().theta
            swarmie.set_heading(
                current_heading + angle,
                ignore=Obstacle.IS_SONAR | Obstacle.VISION_SAFE
            )

    def sees_home_tag(self):
        """Returns true if the rover can see a home tag.
        Returns false otherwise.
        """
        return bool(swarmie.get_latest_targets(id=256))

    def _angle_between(self, point_1, point_2):
        """Returns the angle from point_1 on a circle to point_2 on a circle.
        Circle is centered at the origin.

        Angle is positive if point_2 is counterclockwise from point_1
        Angle is negative if point_2 is clockwise from point_1

        Args:
        * point_1 - geometry_msgs/Point the start point
        * point_2 - geometry_msgs/Point the end point

        Returns:
        * angle - the angle, in radians, between point_1 and point_2
        """
        angle_1 = math.atan2(point_1.y, point_1.x)
        angle_2 = math.atan2(point_2.y, point_2.x)
        return angles.shortest_angular_distance(angle_1, angle_2)

    def _sort_tags_nearest_center(self, detections):
        """Sort args tags by their distance from the center
        of the camera's field of view.

        Args:
        * detections - the list of apriltags2to1/AprilTagDetections.

        Returns:
        * sorted detections  sorted list of AprilTagDetections in view. Will
          be empty if no tags are in view.
        """
        return sorted(detections,
                      key=lambda x: abs(x.pose.pose.position.x))

    def _get_speeds(self, linear=None, angular=None):
        """Return the speed dictionary to use."""
        speeds = swarmie.speed_normal

        if linear is not None:
            speeds['linear'] = linear
        if angular is not None:
            speeds['angular'] = angular

        return speeds

    def sweep(self, angle=math.pi/4, dist=0.3,
              ignore=Obstacle.PATH_IS_CLEAR, throw=False,
              linear=None, angular=None):
        """Look for blocks in a sweeping right arc and a sweeping left arc.
        Other version:
        def sweep(self, time=2.5, linear=0.3, angular=0.75,
                  ignore=Obstacle.PATH_IS_CLEAR, throw=False):

        Args:
        * angle - the angle to turn
        * dist - the distance to drive forward
        * ignore - obstacle bits to ignore while turning and driving
        * throw - whether to raise DriveExceptions
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - MoveResult of the last executed call of
          Swarmie.set_heading().
          drive_result == MoveResult.SUCCESS if all turns were successful.
          drive_result == MoveResult.OBSTACLE_HOME or MoveResult.OBSTACLE_CORNER
          if home tags aren't being ignored and a home tag is seen.
          drive_result == MoveResult.OBSTACLE_TAG if targets aren't being
          ignored and a target is seen.
          drive_result == MoveResult.OBSTACLE_SONAR if sonar is blocked and
          isn't being ignored.

        Raises:
        * mobility.Swarmie.HomeException - if home tags aren't being
          ignored, throw=True, and a home tag is seen.
        * mobility.Swarmie.HomeCornerException if home tags are ignored but home
          corners aren't, throw=True, and a home corner is seen.
        * mobility.Swarmie.TagException - if targets tags aren't being
          ignored, throw=True, and a target is seen.
        * mobility.Swarmie.ObstacleException - if sonar is blocked, throw=True,
          and sonar isn't being ignored.
        """
        speeds = self._get_speeds(linear, angular)

        start_heading = swarmie.get_odom_location().get_pose().theta
        ignore |= Obstacle.SONAR_BLOCK  # always ignore this one too

        try:
            swarmie.set_heading(start_heading - angle, ignore=ignore, **speeds)
            swarmie.drive(dist, ignore=ignore, **speeds)
            swarmie.drive(-dist, ignore=ignore, **speeds)
            swarmie.set_heading(start_heading + angle, ignore=ignore, **speeds)
            swarmie.drive(dist, ignore=ignore, **speeds)
            swarmie.drive(-dist, ignore=ignore, **speeds)
            swarmie.set_heading(start_heading, ignore=ignore, **speeds)
            # swarmie.timed_drive(time, linear, -angular, ignore=ignore)
            # swarmie.timed_drive(time, -linear, angular, ignore=ignore)

            # physical rover doesn't go left as well
            # if not swarmie.simulator_running():
            #     angular *= 1.5
            #     linear *= 1.2
            # swarmie.timed_drive(time, linear, angular, ignore=ignore)
            # swarmie.timed_drive(time, -linear, -angular, ignore=ignore)

        except HomeException:
            if throw:
                raise
            return MoveResult.OBSTACLE_HOME
        except TagException:
            if throw:
                raise
            return MoveResult.OBSTACLE_TAG
        except ObstacleException:
            if throw:
                raise
            return MoveResult.OBSTACLE_SONAR

        return MoveResult.SUCCESS

    def clear(self, angle, ignore=Obstacle.IS_SONAR, reset_heading=True,
              throw=False, angular=None):
        """Turn right, then left, then back to start heading.
        Helps to clear and mark the map if in a difficult spot.

        Args:
        * angle - the angle to turn
        * reset_heading - whether the rover should return to its start heading
          after turning left and right
        * throw - whether to raise exceptions, just like Swarmie.drive()
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - MoveResult of the last executed call of
          Swarmie.set_heading().
          drive_result == MoveResult.SUCCESS if all turns were successful.
          drive_result == MoveResult.OBSTACLE_HOME or MoveResult.OBSTACLE_CORNER
          if home tags aren't being ignored and a home tag is seen.
          drive_result == MoveResult.OBSTACLE_TAG if targets aren't being
          ignored and a target is seen.

        Raises:
        * mobility.Swarmie.HomeException - if home tags aren't being
          ignored, throw=True, and a home tag is seen.
        * mobility.Swarmie.HomeCornerException if home tags are ignored but home
          corners aren't, throw=True, and a home corner is seen.
        * mobility.Swarmie.TagException - if targets tags aren't being
          ignored, throw=True, and a target is seen.
        """
        speeds = self._get_speeds(angular=angular)

        start_heading = swarmie.get_odom_location().get_pose().theta
        ignore |= Obstacle.SONAR_BLOCK  # always ignore this one too

        # Planner._clear() uses Swarmie.set_heading(), which doesn't have
        # an option to return the MoveResult instead of raising an
        # exception. So catch exceptions, and return appropriate result.
        try:
            swarmie.set_heading(start_heading - angle, ignore=ignore, **speeds)
            swarmie.set_heading(start_heading + angle, ignore=ignore, **speeds)
            if reset_heading:
                swarmie.set_heading(start_heading, ignore=ignore, **speeds)
        except HomeException:
            if throw:
                raise
            return MoveResult.OBSTACLE_HOME
        except TagException:
            if throw:
                raise
            return MoveResult.OBSTACLE_TAG

        return MoveResult.SUCCESS

    def _go_around(self, angle, dist, linear=None, angular=None):
        """Turn by 'angle' and then drive 'dist'.

        Args:
        * angle - the angle to turn first
        * dist - the distance to turn second
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * turn_result - the MoveResult of the turn
        * drive_result - the MoveResult of the drive
        """
        speeds = self._get_speeds(linear, angular)

        ignore = Obstacle.IS_SONAR
        if self.avoid_targets is True:
            ignore |= Obstacle.TAG_TARGET
        elif self.avoid_home is True:
            # Ignore both types of tag and the corner of home because target
            # tags are likely to be in view inside the home nest.
            ignore |= Obstacle.VISION_SAFE

        cur_heading = swarmie.get_odom_location().get_pose().theta
        turn_result = swarmie.set_heading(
            cur_heading + angle,
            ignore=ignore,
            throw=False,
            **speeds
        )
        drive_result = swarmie.drive(dist, ignore=Obstacle.SONAR_BLOCK,
                                     throw=False, **speeds)

        return turn_result, drive_result

    def get_angle_to_face_point(self, point):
        """Returns the angle you should turn in order to face a point.

        Args:
        * point - geometry_msgs/Point the point to face in the /odom frame

        Returns:
        * angle - the angle you need to turn, in radians
        """
        start = swarmie.get_odom_location().get_pose()
        return angles.shortest_angular_distance(
            start.theta,
            math.atan2(point.y - start.y, point.x - start.x)
        )

    def face_nearest_block(self):
        """Have the rover turn to face the nearest block to it. Useful
        when exiting gohome (when going home without a block) or search.

        Does nothing if no blocks are seen, if there is a home tag closer
        to the rover than the nearest block, or if a sonar obstacle prevents
        the rover from making the turn.
        """
        block = swarmie.get_nearest_block_location(targets_buffer_age=8)

        if block is not None:
            angle = self.get_angle_to_face_point(block)
            swarmie.turn(angle,
                         ignore=Obstacle.VISION_SAFE,
                         throw=False)

        return

    def _get_next_waypoint(self, tolerance_step):
        """Get another nav plan and return the first waypoint. Try three
        times, incrementing self.tolerance by tolerance_step after a failure.

        Args:
        * tolerance_step - amount to increment tolerance by if nav_plan
          service fails

        Returns:
        * point - geometry_msgs/Point - the next point to drive to
        """
        print('\nGetting new nav plan.')

        for i in range(4):
            try:
                self.plan = swarmie.get_plan(
                    self.goal,
                    tolerance=self.tolerance,
                    use_home_layer=self.avoid_home
                )
                break  # plan received
            except rospy.ServiceException:
                print('ServiceException.')
                if i < 3:
                    print('Expanding tolerance.')
                    self.tolerance += tolerance_step
                else:
                    raise  # tried 3 times, we give up

        print('Received nav plan.')
        pose = self.plan.plan.poses[0]

        return Point(x=pose.pose.position.x, y=pose.pose.position.y)

    def _rviz_nav_goal_cb(self, msg, args):
        """Subscriber to help with testing. Responds to RViz nav_goals
        published with a mouse click.
        """
        avoid_targets = args[0]
        avoid_home = args[1]
        goal = Pose2D(x=msg.pose.position.x, y=msg.pose.position.y)
        tolerance = 0.0

        self.drive_to(goal, tolerance,
                      avoid_targets=avoid_targets, avoid_home=avoid_home)

    def _check_sonar_obstacles(self):
        """Check sonar obstacles over a short period of time, hopefully to
        weed out some of the noise and let us continue driving if we stopped
        for a 'fake' obstacle.

        Returns:
        * left_blocked - bool, whether left sonar seems blocked
        * center_blocked - bool, whether center sonar seems blocked
        * right_blocked - bool, whether right sonar seems blocked
        """
        # TODO: what's a good number?
        BLOCKED_THRESHOLD = 0.7

        rate = rospy.Rate(10)  # 10 hz
        count = 10
        left = 0
        center = 0
        right = 0

        for i in range(count):
            obstacle = swarmie.get_obstacle_condition()

            if obstacle & Obstacle.SONAR_LEFT == Obstacle.SONAR_LEFT:
                left += 1
            if (obstacle & Obstacle.SONAR_CENTER ==
                    Obstacle.SONAR_CENTER):
                center += 1
            if obstacle & Obstacle.SONAR_RIGHT == Obstacle.SONAR_RIGHT:
                right += 1

            rate.sleep()

        left_blocked = left / count > BLOCKED_THRESHOLD
        center_blocked = center / count > BLOCKED_THRESHOLD
        right_blocked = right / count > BLOCKED_THRESHOLD

        return left_blocked, center_blocked, right_blocked

    def _avoid_tag(self, id=0, ignore=Obstacle.IS_SONAR,
                   linear=None, angular=None):
        """Helper to Planner.drive_to(). Make one attempt to get around a
        home or target tag.

        Args:
        * id - the id of the tag to avoid (0 - target, 256 - home)
        * ignore - the Obstacle's to ignore while clearing, if necessary.
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - MoveResult of the avoidance attempt
        """
        speeds = self._get_speeds(linear, angular)

        sorted_detections = utils.sort_tags_left_to_right(
            swarmie.get_latest_targets(id=id)
        )

        # if count == 3:  # last resort
        #     self.current_state = Planner.STATE_DRIVE
        #     angle = self._get_angle_to_face(point)
        #     swarmie.turn(
        #         angle,
        #         ignore=Obstacle.TAG_TARGET,
        #         throw=False
        #     )
        #     self.result = swarmie.drive(
        #         .75,
        #         ignore=Obstacle.TAG_TARGET,
        #         throw=False
        #     )

        if len(sorted_detections) == 0:
            # no tags in view anymore
            print("I can't see anymore tags, I'll try creeping",
                  "and clearing.")
            self.prev_state = self.current_state
            self.current_state = Planner.STATE_DRIVE
            swarmie.drive(
                0.1,
                ignore=Obstacle.SONAR_BLOCK,
                throw=False,
                **speeds
            )
            drive_result = self.clear(math.pi / 8, ignore=ignore,
                                      angular=speeds['angular'])

        else:
            left_angle, left_dist = \
                self._get_angle_and_dist_to_avoid(
                    sorted_detections[0],
                    direction='left'
                )
            right_angle, right_dist = \
                self._get_angle_and_dist_to_avoid(
                    sorted_detections[-1],
                    direction='right'
                )
            angle = left_angle
            dist = left_dist

            if (self.current_state == Planner.STATE_AVOID_LEFT or
                    self.prev_state == Planner.STATE_AVOID_LEFT):
                # Keep going left. Should help avoid bouncing back
                # and forth between tags just out of view.
                print("I was turning left last time, so I'll keep",
                      "it that way.")
                self.prev_state = self.current_state
                self.current_state = Planner.STATE_AVOID_LEFT
                angle = left_angle
                dist = left_dist

            elif (self.current_state == Planner.STATE_AVOID_RIGHT or
                    self.prev_state == Planner.STATE_AVOID_RIGHT):
                # Keep going right
                print("I was turning right last time, so I'll",
                      "keep it that way.")
                self.prev_state = self.current_state
                self.current_state = Planner.STATE_AVOID_RIGHT
                angle = right_angle
                dist = right_dist

            else:
                # pick whichever angle is shortest
                if abs(right_angle) < abs(left_angle):
                    print('Right looks most clear, turning right.')
                    # print('Right turn makes most sense, turning right.')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_RIGHT
                    angle = right_angle
                    dist = right_dist
                else:
                    print('Left looks most clear, turning left.')
                    # print('Left turn makes most sense, turning left')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_LEFT

            _turn_result, drive_result = self._go_around(angle, dist, **speeds)

        return drive_result

    def _is_safe_to_back_up(self):
        """Returns True if it's safe for the rover to back up. It is safe to
        back up if the rover is further than 1.5 meters from the current home
        location, or if the rover is within 1.5 meters from home, but is
        facing home. In other words, it's not safe to back up if the rover
        is close to home and has it's back to home.

        Returns:
        * is_safe - bool whether it's safe to back up.
        """
        # Only back up if we're far enough away from home for it
        # to be safe. Don't want to back up into the nest!
        home_loc = swarmie.get_home_odom_location()
        current_loc = swarmie.get_odom_location().get_pose()
        dist = math.sqrt((home_loc.x - current_loc.x) ** 2
                         + (home_loc.y - current_loc.y) ** 2)
        if dist > 1.5:
            return True

        angle_to_home = self.get_angle_to_face_point(home_loc)
        if abs(angle_to_home) < math.pi / 2:
            return True

        return False

    def _face_point(self, point, ignore=Obstacle.PATH_IS_CLEAR, angular=None):
        """Turn to face a point in the odometry frame. Rover will attempt to
        turn the shortest angle to face the point, and if it fails (sonar
        detects something in the way, or the rover saw a type of tag it wants
        to stop for), it will possibly back up and try to turn in the opposite
        direction to face the point.

        Args:
        * point - geometry_msgs/Point the point to turn and face.
        * ignore - the Obstacle's to ignore while turning. Should only be
          either Obstacle.VISION_HOME or Obstacle.TAG_TARGET, or both. Sonar
          will not be ignored in the direction the rover is currently turning.
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        """
        speeds = self._get_speeds(angular=angular)

        print('Facing next point...')
        # Make sure all sonar sensors are never ignored together here
        if ignore & Obstacle.IS_SONAR == Obstacle.IS_SONAR:
            ignore ^= Obstacle.IS_SONAR
        ignore |= Obstacle.SONAR_BLOCK

        # Try turning in the shortest direction:
        turn_angle = self.get_angle_to_face_point(point)

        if turn_angle > 0:
            # turning left, pay attention to left sensor only
            cur_ignore = ignore | Obstacle.SONAR_CENTER | Obstacle.SONAR_RIGHT
        else:
            # turning right, pay attention to right sensor only
            cur_ignore = ignore | Obstacle.SONAR_CENTER | Obstacle.SONAR_LEFT

        drive_result = swarmie.turn(
            turn_angle,
            ignore=cur_ignore,
            throw=False,
            **speeds
        )

        # Return if successful, or if rover stopped for a cube or home tag
        if drive_result != MoveResult.OBSTACLE_SONAR:
            print("Completed turn or found an important AprilTag.")
            return drive_result

        # If possible, back up and try same direction again.
        if self._is_safe_to_back_up():
            dist = -0.15
            if (self.prev_state == Planner.STATE_AVOID_LEFT or
                    self.prev_state == Planner.STATE_AVOID_RIGHT or
                    self.prev_state == Planner.STATE_AVOID_REVERSE):
                dist = -0.25
            swarmie.drive(dist, ignore=ignore | Obstacle.IS_SONAR, throw=False)

            turn_angle = self.get_angle_to_face_point(point)
            if turn_angle > 0:
                cur_ignore = (ignore |
                              Obstacle.SONAR_CENTER | Obstacle.SONAR_RIGHT)
            else:
                cur_ignore = (ignore |
                              Obstacle.SONAR_CENTER | Obstacle.SONAR_LEFT)

            drive_result = swarmie.turn(
                turn_angle,
                ignore=cur_ignore,
                throw=False,
                **speeds
            )

            if drive_result != MoveResult.OBSTACLE_SONAR:
                print("Completed turn or found an important AprilTag.")
                return drive_result

        # Last resort, try turning in the other direction.
        if self._is_safe_to_back_up():
            dist = -0.15
            if (self.prev_state == Planner.STATE_AVOID_LEFT or
                    self.prev_state == Planner.STATE_AVOID_RIGHT or
                    self.prev_state == Planner.STATE_AVOID_REVERSE):
                dist = -0.25
            swarmie.drive(dist, ignore=ignore | Obstacle.IS_SONAR, throw=False)

        turn_angle = self.get_angle_to_face_point(point)

        # But don't bother if the rover is already mostly facing the
        # right direction.
        if abs(turn_angle) < math.pi / 2:
            print("Didn't make the whole turn, but I'm close enough.")
            return drive_result

        print("Trying to turn other way.")
        turn_angle = angles.two_pi_complement(turn_angle)
        turns = []

        if turn_angle > 0:
            cur_ignore = ignore | Obstacle.SONAR_CENTER | Obstacle.SONAR_RIGHT
        else:
            cur_ignore = ignore | Obstacle.SONAR_CENTER | Obstacle.SONAR_LEFT

        # Split turn angle into two steps if abs val is greater than PI.
        # The driver API only makes individual turns <= PI.
        if turn_angle >= math.pi:
            turns.append(9 * math.pi / 10)
            turns.append(turn_angle - (9 * math.pi / 10))
        elif turn_angle <= math.pi:
            turns.append(-9 * math.pi / 10)
            turns.append(turn_angle + (9 * math.pi / 10))
        else:
            turns.append(turn_angle)

        for turn in turns:
            drive_result = swarmie.turn(
                turn,
                ignore=cur_ignore,
                throw=False,
                **speeds
            )

            if drive_result != MoveResult.SUCCESS:
                return drive_result

        return drive_result

    def drive_to(self, goal, tolerance=0.0, tolerance_step=0.5,
                 max_attempts=10, avoid_targets=True, avoid_home=False,
                 use_waypoints=True, start_location=None,
                 distance_threshold=None, linear=None, angular=None):
        """Try to get the rover to goal location. Returns when at goal
        or if home target is found.

        !! avoid_targets and avoid_home shouldn't both be set to True.
           avoid_home will be set to False in this case. !!

        Args:
        * goal - geometry_msgs/Pose2D or geometry_msgs/Point the goal location.
        * tolerance - how close you'd like to get in meters. Could be useful if
          you get a rospy.ServiceException re-raised from this function. The
          function will try 3 times to expand the tolerance by tolerance_step
          from the initial tolerance. But the map/get_plan service may on rare
          occasions still fail to find a path. You can try re-calling with a
          larger initial tolerance in that case.
        * tolerance_step - how much to increment tolerance by if map/get_plan
          fails to find a path.
        * max_attempts - Number of consecutive times the rover can drive or
          turn and receive a MoveResult other than SUCCESS before giving up.
          This is a relatively large number because the first waypoint in a nav
          plan (or the goal itself if not using waypoints) can be 5 or 10
          meters away, and any time the rover comes upon a sonar or tag
          obstacle, its fail_count is incremented. It's not uncommon for the
          rover to come across a scattering of cubes that it must spend some
          time avoiding. During that time, it needs some leeway to drive
          slightly off course as a part of its avoidance behavior. Default
          value of 10 seems to work for the most part.
        * avoid_targets - whether the rover should avoid cubes while driving
          to the goal
        * avoid_home - whether the rover should drive around home tags while
          driving to the goal. Can be used during a search behavior.
        * use_waypoints - whether to use waypoints from searching the map. Can
          be used from the start or as a fallback if the map search fails.
        * start_location - geometry_msgs/Pose2D, the start location of the
          rover. Used by Planner.spiral_search() along with distance_threshold.
          to limit the rover's ability to get way off course while trying to
          get around something like the arena boundary.
        * distance_threshold - the maximum distance the rover's current
          location can be from the start_location before a PathException is
          raised.
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - mobility.msg.MoveResult the result of the drive
          command. drive_result == MoveResult.SUCCESS if we reached the goal.
          drive_result == MoveResult.OBSTACLE_HOME if we found a home tag.
          drive_result == MoveResult.OBSTACLE_TAG if we found a cube tag and
          avoid_targets is False.

        Raises:
        * mobility.Swarmie.PathException - if the rover fails more than
          max_attempts times to reach a single waypoint in its current
          navigation plan. Or, if using start_location and distance_threshold,
          if the rover's current location is further than distance_threshold
          from the start_location.
        * rospy.ServiceException - if a path can't be found in 3
          attempts, beginning with initial tolerance and incrementing by
          tolerance_step on each subsequent attempt.
        """
        speeds = self._get_speeds(linear, angular)

        print('\nRequest received')
        self.fail_count = 0
        self.tolerance = tolerance

        self.avoid_targets = avoid_targets
        if avoid_targets is True and avoid_home is True:
            avoid_home = False
        self.avoid_home = avoid_home

        current_ignore = Obstacle.IS_SONAR
        if self.avoid_targets is True:
            current_ignore |= Obstacle.TAG_TARGET
        elif self.avoid_home is True:
            current_ignore |= Obstacle.VISION_HOME

        self.goal.x = goal.x
        self.goal.y = goal.y

        self.cur_loc = swarmie.get_odom_location()
        self.current_state = Planner.STATE_IDLE
        self.prev_state = Planner.STATE_IDLE

        while (not self.cur_loc.at_goal(self.goal,
                                        Planner.DISTANCE_OK + self.tolerance)
               and self.fail_count < max_attempts):

            if use_waypoints is True:
                # get new plan and try to drive to first point in it
                point = self._get_next_waypoint(tolerance_step)
            else:
                point = goal

            self.prev_state = self.current_state
            self.current_state = Planner.STATE_DRIVE
            # Turn to approximate goal heading while ignoring sonar and tags
            # helps to prevent rover from trying to jump around obstacles
            # before it even starts along its new path
            self.result = self._face_point(
                point,
                ignore=current_ignore ^ Obstacle.IS_SONAR,
                angular=speeds['angular']
            )

            if self.result == MoveResult.SUCCESS:
                self.result = swarmie.drive_to(
                    point,
                    ignore=Obstacle.SONAR_BLOCK,
                    throw=False,
                    **speeds
                )

            if self.result == MoveResult.SUCCESS:
                # Success, we got to our waypoint, or got ourselves out of
                # whatever pickle we were just in.
                # Just get a new plan and drive to next point
                self.fail_count = 0
                self.prev_state = self.current_state
                self.current_state = Planner.STATE_IDLE
                print('Successfully drove to first point in nav plan.')

            # otherwise, something went wrong or we found home
            elif self.result == MoveResult.OBSTACLE_HOME:
                # get around the home tag obstacle
                count = 0

                # Give the rover 3 tries to avoid any tags nearby before
                # getting a new nav plan. MoveResult.OBSTACLE_SONAR takes
                # priority in the driver code, so it should be safe to continue
                # this loop if the MoveResult is just an OBSTACLE_HOME
                # self.fail_count may exceed limit here, but I'll let it go
                while count < 3 and self.result == MoveResult.OBSTACLE_HOME:
                    print('\nObstacle: Found Home.')
                    count += 1
                    self.fail_count += 1

                    if self.avoid_home is False:
                        print('Obstacle: Found Home.')
                        return MoveResult.OBSTACLE_HOME

                    self.result = self._avoid_tag(id=256,
                                                  ignore=current_ignore,
                                                  **speeds)

            elif self.result == MoveResult.OBSTACLE_TAG:
                # get around the tag obstacle
                count = 0

                # Give the rover 3 tries to avoid any tags nearby before
                # getting a new nav plan. MoveResult.OBSTACLE_SONAR takes
                # priority in the driver code, so it should be safe to continue
                # this loop if the MoveResult is just an OBSTACLE_TAG
                # self.fail_count may exceed limit here, but I'll let it go
                while count < 3 and self.result == MoveResult.OBSTACLE_TAG:
                    print('\nObstacle: Found a Tag.')
                    if self.avoid_targets is False:
                        if not self.sees_home_tag():
                            return self.result
                    else:
                        swarmie.add_resource_pile_location( override=True, 
                                                            ignore_claw=True)
                    count += 1
                    self.fail_count += 1

                    self.result = self._avoid_tag(id=0,
                                                  ignore=current_ignore,
                                                  **speeds)

            elif self.result == MoveResult.OBSTACLE_SONAR:
                # Check for home and tag obstacles just to be safe, because
                # sonar MoveResults take priority, and would mask a home or
                # target tag in view.
                obstacle = swarmie.get_obstacle_condition()

                if (obstacle & Obstacle.TAG_HOME == Obstacle.TAG_HOME and
                        self.avoid_home is False):
                    return MoveResult.OBSTACLE_HOME

                if (obstacle & Obstacle.TAG_TARGET == Obstacle.TAG_TARGET and
                        self.avoid_targets is False):
                    return MoveResult.OBSTACLE_TAG

                # get around the sonar obstacle
                self.fail_count += 1

                print('\nObstacle: Sonar.')
                left_blocked, center_blocked, right_blocked = \
                    self._check_sonar_obstacles()

                if (not left_blocked and
                        not center_blocked and not right_blocked):
                    print('\nFake sonar obstacle??')
                    pass  # 'fake' obstacle?

                elif not left_blocked and center_blocked and right_blocked:
                    print('Left looks clear, turning left.')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(math.pi / 4, 0.7, **speeds)
                    # swarmie.drive_to(point, throw=False)

                elif left_blocked and center_blocked and not right_blocked:
                    print('Right looks clear, turning right.')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_RIGHT
                    self._go_around(-math.pi / 4, 0.7, **speeds)
                    # swarmie.drive_to(point, throw=False)

                elif left_blocked and not center_blocked and not right_blocked:
                    print('Only left blocked, turning a little right.')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_RIGHT
                    self._go_around(-math.pi / 6, 0.6, **speeds)
                    # swarmie.drive_to(point, throw=False)

                elif not left_blocked and not center_blocked and right_blocked:
                    print('Only right blocked, turning a little left.')
                    self.prev_state = self.current_state
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(math.pi / 6, 0.6, **speeds)
                    # swarmie.drive_to(point, throw=False)

                else:
                    print('Neither left or right look clear.')

                    # Only back up if we're far enough away from home for it
                    # to be safe. Don't want to back up into the nest!
                    if self._is_safe_to_back_up():
                        print('Backing up.')
                        swarmie.drive(
                            -0.3,
                            ignore=Obstacle.IS_SONAR,
                            throw=False
                        )

                    if (self.current_state == Planner.STATE_AVOID_RIGHT or
                            self.prev_state == Planner.STATE_AVOID_RIGHT):
                        self.prev_state = self.current_state
                        self.current_state = Planner.STATE_AVOID_RIGHT
                        self.clear(-math.pi / 4, ignore=current_ignore,
                                   reset_heading=False,
                                   angular=speeds['angular'])
                        self._go_around(-math.pi / 4, 0.75, **speeds)

                    else:
                        self.prev_state = self.current_state
                        self.current_state = Planner.STATE_AVOID_LEFT
                        self.clear(math.pi / 4, ignore=current_ignore,
                                   reset_heading=False,
                                   angular=speeds['angular'])
                        self._go_around(math.pi / 4, 0.75, **speeds)

            elif self.result == MoveResult.INSIDE_HOME:
                raise InsideHomeException(MoveResult.INSIDE_HOME)

            elif self.result == MoveResult.PATH_FAIL:
                # Hope we can back up if this ever happens
                self.fail_count += 1

                print('\nPath Failure. Backing up.')
                self.prev_state = self.current_state
                self.current_state = Planner.STATE_AVOID_REVERSE
                swarmie.drive(
                    -0.5,
                    ignore=Obstacle.IS_SONAR | Obstacle.VISION_SAFE,
                    throw=False
                )

            self.cur_loc = swarmie.get_odom_location()

            if self.fail_count >= max_attempts:
                print('Failed to drive to goal {} times.'.format(
                    max_attempts)
                )
                raise PathException(MoveResult.PATH_FAIL)

            if start_location is not None:
                current_loc = self.cur_loc.get_pose()
                dist = math.sqrt((start_location.x - current_loc.x) ** 2
                                 + (start_location.y - current_loc.y) ** 2)
                if dist > distance_threshold:
                    raise PathException(MoveResult.PATH_FAIL)

        print('Successfully executed nav plan.')
        return MoveResult.SUCCESS

    def drive(self, distance, tolerance=0.0, tolerance_step=0.5,
              max_attempts=10, avoid_targets=True, avoid_home=False,
              use_waypoints=True, linear=None, angular=None):
        """Convenience wrapper to drive_to(). Drive the given distance, while
        avoiding sonar and target obstacles.

        !! avoid_targets and avoid_home shouldn't both be set to True.
           avoid_home will be set to False in this case. !!

        Args:
        * distance - the distance, in meters, to drive
        * tolerance - how close you'd like to get. See drive_to() for more
        * tolerance_step - how much tolerance is incremented if map/get_plan
          service fails.
        * max_attempts - number of times the rover can try to reach a single
          waypoint in the plan, or the goal itself if not using waypoints.
        * avoid_targets - whether the rover should avoid cubes while driving
          to the goal
        * avoid_home - whether the rover should drive around home tags while
          driving to the goal. Can be used during a search behavior.
        * use_waypoints - whether to use waypoints from searching the map. Can
          be used from the start or as a fallback if the map search fails.
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - mobility.msg.MoveResult the result of the drive
          command. drive_result == MoveResult.SUCCESS if we reached the goal.
          drive_result == MoveResult.OBSTACLE_HOME if we found a home tag.
          drive_result == MoveResult.OBSTACLE_TAG if we found a cube tag and
          avoid_targets is False.

        Raises:
        * mobility.Swarmie.PathException - if the rover fails more than
          max_attempts times to reach a single waypoint in its current
          navigation plan.
        * rospy.ServiceException if /map/get_plan fails. See drive_to() for
          more information.
        """
        speeds = self._get_speeds(linear, angular)

        self.cur_loc = swarmie.get_odom_location()
        start = self.cur_loc.get_pose()

        goal = Point()
        goal.x = start.x + distance * math.cos(start.theta)
        goal.y = start.y + distance * math.sin(start.theta)

        return self.drive_to(
            goal,
            tolerance=tolerance,
            tolerance_step=tolerance_step,
            max_attempts=max_attempts,
            avoid_targets=avoid_targets,
            avoid_home=avoid_home,
            use_waypoints=use_waypoints,
            **speeds
        )

    def _get_spiral_points(self, start_distance, distance_step, num_legs=10):
        """Get a list of waypoints for the spiral search pattern. Waypoints
        will be used as goals to planner.drive_to()

        Args:
        * start_distance - the distance of the first two legs of the spiral.
        * distance_step - how much to increment the leg length by after driving
          two legs.

        Returns:
        * points - the list of waypoints
        """
        start_loc = swarmie.get_odom_location().get_pose()
        points = []
        distance = start_distance
        angle = math.pi / 2

        prev_point = Point()
        prev_point.x = start_loc.x + distance * math.cos(start_loc.theta)
        prev_point.y = start_loc.y + distance * math.sin(start_loc.theta)
        points.append(prev_point)

        # todo: is this big enough, or too big?
        for i in range(1, num_legs):
            if i % 2 == 0:
                distance += distance_step

            point = Point()
            point.x = prev_point.x + distance * math.cos(start_loc.theta
                                                         + angle)
            point.y = prev_point.y + distance * math.sin(start_loc.theta
                                                         + angle)
            points.append(point)
            prev_point = point
            angle += math.pi / 2

        return points

    def spiral_search(self, start_distance, distance_step=0.5, num_legs=10,
                      tolerance=0.0, tolerance_step=0.5, max_attempts=5,
                      avoid_targets=True, avoid_home=False,
                      use_waypoints=True, linear=None, angular=None):
        """Search in a square spiral pattern. Can be used to search for tags
        or home, depending on values of avoid_targets and avoid_home.

        Args:
        * start_distance - the distance of the first two legs of the spiral.
        * distance_step - how much to increment the leg length by after driving
          two legs.
        * tolerance - the tolerance for the nav plans, see drive_to() for
          more information.
        * tolerance_step - how much to increment the tolerance by if a nav plan
          can't be found.
        * max_attempts - number of times the rover can try to reach a single
          waypoint in the plan, or the goal itself if not using waypoints.
        * avoid_targets - whether the rover should avoid cubes while driving
          to the goal
        * avoid_home - whether the rover should drive around home tags while
          driving to the goal. Can be used during a search behavior.
        * use_waypoints - whether to use waypoints from searching the map. Can
          be used from the start or as a fallback if the map search fails.
        * linear - the linear speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)
        * angular - the angular speed to drive at (the value currently in
          swarmie.speed_normal will be used if this isn't specified.)

        Returns:
        * drive_result - mobility.msg.MoveResult the result of the drive
          command.
            - drive_result == MoveResult.OBSTACLE_HOME if we found home and
              avoid_home is False.
            - drive_result == MoveResult.OBSTACLE_TAG if we found a cube tag
              and avoid_targets is False.
            - drive_result == MoveResult.SUCCESS if nothing is not found.

        Raises:
        * mobility.Swarmie.PathException - if the rover fails more than
          max_attempts times to reach a single waypoint in its current
          navigation plan. Or, if the rover's current location is further than
          distance_threshold from the start_location (helps to keep spiral
          search from wandering off too far, trying to get around walls).
        * rospy.ServiceException - if a path can't be found in 3
          attempts, beginning with initial tolerance and incrementing by
          tolerance_step on each subsequent attempt.
        """
        speeds = self._get_speeds(linear, angular)

        points = self._get_spiral_points(start_distance, distance_step,
                                         num_legs=num_legs)
        MAX_CONSECUTIVE_FAILURES = 3
        fail_count = 0

        start_loc = swarmie.get_odom_location().get_pose()

        for index, point in enumerate(points):
            # Stop if rover gets further away than it should currently be
            # from the center of the spiral. This can happen if the rover
            # starts following arena boundary walls.
            distance_threshold = (start_distance
                                  + (index+1) / 2.0 * distance_step)

            try:
                drive_result = self.drive_to(
                    point,
                    tolerance=tolerance,
                    tolerance_step=tolerance_step,
                    max_attempts=max_attempts,
                    avoid_targets=avoid_targets,
                    avoid_home=avoid_home,
                    use_waypoints=use_waypoints,
                    start_location=start_loc,
                    distance_threshold=distance_threshold,
                    **speeds
                )
                if (drive_result == MoveResult.OBSTACLE_HOME
                        or drive_result == MoveResult.OBSTACLE_TAG):
                    return drive_result
            except PathException:
                fail_count += 1
                if fail_count >= MAX_CONSECUTIVE_FAILURES:
                    raise

        return MoveResult.SUCCESS


def main():
    """For testing Planner class using RViz nav goals."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        '--avoid-home',
        action='store_true',
        help='whether the rover should avoid home during this run'
    )
    group.add_argument(
        '--avoid-targets',
        action='store_true',
        help='whether the rover should avoid targets during this run'
    )
    args = parser.parse_args()
    swarmie.start(node_name='planner')

    planner = Planner(use_rviz_nav_goal=True,
                      avoid_home=args.avoid_home,
                      avoid_targets=args.avoid_targets)
    rospy.spin()


if __name__ == '__main__':
    main()
