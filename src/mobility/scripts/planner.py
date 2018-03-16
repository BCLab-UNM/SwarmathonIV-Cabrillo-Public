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
import angles

from geometry_msgs.msg import PoseStamped, Pose2D, Point
from nav_msgs.srv import GetPlanResponse

from swarmie_msgs.msg import Obstacle
from mobility.msg import MoveResult

from mobility.swarmie import Swarmie, Location, HomeException, PathException


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
    FAIL_COUNT_LIMIT = 10

    def __init__(self, swarmie, use_rviz_nav_goal=False):
        """Create a new Planner.
        Args:
        * swarmie - the Swarmie object you've already instantiated
        * use_rviz_nav_goal - for testing/debugging. Subscribes to RViz
          nav_goals published on the topic /rovername/goal, and navigates to
          those goals using Planner.drive_to()
        """
        self.rovername = swarmie.rover_name
        self.swarmie = swarmie
        self.current_state = Planner.STATE_IDLE
        self.cur_loc = Location(None)
        self.goal = Pose2D()
        self.plan = GetPlanResponse()
        self.tolerance = 0.0
        self.result = MoveResult.SUCCESS
        self.fail_count = 0

        # Subscribers
        if use_rviz_nav_goal:
            print ('Using RViz 2D Nav Goals')
            self._nav_goal_sub = rospy.Subscriber(
                self.rovername + '/goal',
                PoseStamped,
                self._rviz_nav_goal_cb,
                queue_size=1
            )

    def _transform_to_base_link(self, detection, timeout=3.0):
        """Transform PoseStamped detection into base_link frame.
        Returns PoseStamped in base_link frame.

        Args:
        * detection - PoseStamped the pose of the tag in the /camera_link frame
        * timeout - float, the time to wait for the transform

        Returns:
        * pose - PoseStamped the pose of the tag in the /base_link frame

        Raises:
        * tf.Exception if timeout is exceeded
        """
        self.swarmie.xform.waitForTransform(
            self.rovername + '/base_link',
            detection.pose.header.frame_id,
            detection.pose.header.stamp,
            rospy.Duration(timeout)
        )

        return self.swarmie.xform.transformPose(self.rovername + '/base_link',
                                                detection.pose)

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
        base_link_pose = self._transform_to_base_link(detection)
        radius = math.sqrt(base_link_pose.pose.position.x**2
                           + base_link_pose.pose.position.y**2)
        tag_point = Point(x=base_link_pose.pose.position.x,
                          y=base_link_pose.pose.position.y)

        path_edge_point = Point()
        # solve for x given the radius and y-coord of a point on a circle
        # Just set x to zero if radius is too small (if tag is too close to
        # the rover. Protects math.sqrt() from evaluating a negative number.
        if radius > Planner.PATHWAY_EDGE_DIST:
            path_edge_point.x = math.sqrt(radius**2
                                          - Planner.PATHWAY_EDGE_DIST**2)
        else:
            path_edge_point.x = 0
        path_edge_point.y = Planner.PATHWAY_EDGE_DIST
        if direction == 'left':
            path_edge_point.y *= -1

        return (-self._angle_between(tag_point, path_edge_point),
                path_edge_point.x + OVERSHOOT_DIST)

    def get_angle_to_face(self, detection):
        """Get the angle required to turn and face a detection to put it in
        the center of the camera's field of view.

        Args:
        * detection - the PoseStamped detection in the /camera_link frame.
          This detection should be the detection whose position it makes the
          most sense to make a turn decision relative to.

        Returns:
        * angle - the angle in radians to turn.
        """
        base_link_pose = self._transform_to_base_link(detection)
        radius = math.sqrt(base_link_pose.pose.position.x**2
                           + base_link_pose.pose.position.y**2)
        tag_point = Point(x=base_link_pose.pose.position.x,
                          y=base_link_pose.pose.position.y)

        center_of_view_point = Point()
        # solve for x given the radius and y-coord of a point on a circle
        # y-coord is zero in this special case
        center_of_view_point.x = math.sqrt(radius**2)
        center_of_view_point.y = 0

        return -self._angle_between(tag_point, center_of_view_point)

    def face_home_tag(self):
        """Turn and face the home tag nearest the center of view if we
        see one. Does nothing if no home tag is seen."""
        home_detections = self._sort_home_tags_nearest_center(
            self.swarmie.get_latest_targets().detections
        )
        if len(home_detections) > 0:
            angle = self.get_angle_to_face(home_detections[0])
            current_heading = self.swarmie.get_odom_location().get_pose().theta
            self.swarmie.set_heading(
                current_heading + angle,
                ignore=Obstacle.IS_SONAR|Obstacle.IS_VISION
            )

    def sees_home_tag(self):
        """Returns true if the rover can see a home tag.
        Returns false otherwise.
        """
        detections = self.swarmie.get_latest_targets().detections

        for detection in detections:
            if detection.id == 256:
                return True

        return False


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

    def _sort_home_tags_nearest_center(self, detections):
        """Sort home tags (id == 256) in view by their distance from the center
        of the camera's field of view.

        Args:
        * detections - apriltags_ros/AprilTagDetectionArray the list
          of detections.

        Returns:
        * sorted_detections - sorted list of AprilTagDetections in view. Will
          be empty if no tags are in view.
        """
        sorted_detections = []

        for detection in detections:
            if detection.id == 256:
                sorted_detections.append(detection)

        return sorted(sorted_detections,
                      key=lambda x : abs(x.pose.pose.position.x))

    def _sort_target_tags_left_to_right(self, detections):
        """Sort target tags (id == 0) in view from left to right (by their x
        position in the camera frame). Removes/ignores tags close enough in the
        camera to likely be a block in the claw.

        Args:
        * detections - apriltags_ros/AprilTagDetectionArray the list
          of detections.

        Returns:
        * sorted_detections - sorted list of AprilTagDetections in view. Will
          be empty if no tags are in view.
        """
        BLOCK_IN_CLAW_DIST = 0.18
        sorted_detections = []

        for detection in detections:
            if (detection.id == 0 and
                    detection.pose.pose.position.z > BLOCK_IN_CLAW_DIST):
                sorted_detections.append(detection)

        return sorted(sorted_detections, key=lambda x : x.pose.pose.position.x)

    def _clear(self, angle, ignore=Obstacle.IS_SONAR, reset_heading=True):
        """Turn right, then left, then back to start heading.
        Helps to clear and mark the map if in a difficult spot.

        Args:
        * angle - the angle to turn
        * ignore - the stuff to ignore. See Swarmie.drive()
        * reset_heading - whether the rover should return to its start heading
          after turning left and right
        """
        start_heading = self.swarmie.get_odom_location().get_pose().theta
        self.swarmie.set_heading(start_heading-angle, ignore=ignore)
        self.swarmie.set_heading(start_heading+angle, ignore=ignore)
        if reset_heading:
            self.swarmie.set_heading(start_heading, ignore=ignore)

    def _go_around(self, angle, dist):
        """Turn by 'angle' and then drive 'dist'.

        Args:
        * angle - the angle to turn first
        * dist - the distance to turn second

        Returns:
        * turn_result - the MoveResult of the turn
        * drive_result - the MoveResult of the drive
        """
        cur_heading = self.swarmie.get_odom_location().get_pose().theta
        turn_result = self.swarmie.set_heading(
            cur_heading + angle,
            ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET,
            throw=False
        )
        drive_result = self.swarmie.drive(dist, throw=False)

        return turn_result, drive_result

    def _get_angle_to_face(self, point):
        """Returns the angle you should turn in order to face a point.

        Args:
        * point - geometry_msgs/Point the point to face in the /odom frame

        Returns:
        * angle - the angle you need to turn, in radians
        """
        start = self.cur_loc.get_pose()
        return angles.shortest_angular_distance(
            start.theta,
            math.atan2(point.y - start.y, point.x - start.x)
        )

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
                self.plan = self.swarmie.get_plan(
                    self.goal,
                    tolerance=self.tolerance
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

    def _rviz_nav_goal_cb(self, msg):
        """Subscriber to help with testing. Responds to RViz nav_goals
        published with a mouse click.
        """
        goal = Pose2D(x=msg.pose.position.x, y=msg.pose.position.y)
        tolerance = 0.0

        self.drive_to(goal, tolerance)

    def drive_to(self, goal, tolerance=0.0, tolerance_step=0.5):
        """Try to get the rover to goal location. Returns when at goal
        or if home target is found.

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

        Returns:
        * drive_result - mobility.msg.MoveResult the result of the drive
        command. drive_result == MoveResult.SUCCESS if we reached the goal.
        drive_result == MoveResult.OBSTACLE_HOME if we found a home tag.

        Raises:
        * mobility.Swarmie.PathException - if the rover fails more than
          Planner.FAIL_COUNT_LIMIT times to reach a single waypoint in the
          its current navigation plan.
        * rospy.ServiceException - if a path can't be found in 3
          attempts, beginning with initial tolerance and incrementing by
          tolerance_step on each subsequent attempt.
        """
        print('\nRequest received')
        self.fail_count = 0
        self.tolerance = tolerance

        self.goal.x = goal.x
        self.goal.y = goal.y

        self.cur_loc = self.swarmie.get_odom_location()
        self.current_state = Planner.STATE_IDLE

        while (not self.cur_loc.at_goal(self.goal,
                                        Planner.DISTANCE_OK+self.tolerance)
               and self.fail_count < Planner.FAIL_COUNT_LIMIT):

            # get new plan and try to drive to first point in it
            point = self._get_next_waypoint(tolerance_step)

            self.current_state = Planner.STATE_DRIVE
            # Turn to approximate goal heading while ignoring sonar and tags
            # helps to prevent rover from trying to jump around obstacles
            # before it even starts along its new path
            turn_angle = self._get_angle_to_face(point)
            self.result = self.swarmie.turn(
                turn_angle,
                ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET,
                throw=False
            )

            if self.result == MoveResult.OBSTACLE_HOME:
                print('Obstacle: Found Home.')
                return self.result

            self.result = self.swarmie.drive_to(point, throw=False)

            if self.result == MoveResult.SUCCESS:
                # Success, we got to our waypoint, or got ourselves out of
                # whatever pickle we were just in.
                # Just get a new plan and drive to next point
                self.fail_count = 0
                self.current_state = Planner.STATE_IDLE
                print('Successfully drove to first point in nav plan.')

            # otherwise, something went wrong or we found home
            elif self.result == MoveResult.OBSTACLE_HOME:
                print('Obstacle: Found Home.')
                return self.result

            elif self.result == MoveResult.OBSTACLE_TAG:
                # get around the tag obstacle
                count = 0

                # Give the rover 3 tries to avoid any tags nearby before
                # getting a new nav plan. MoveResult.OBSTACLE_SONAR takes
                # priority in the driver code, so it should be safe to continue
                # this loop if the MoveResult is just an OBSTACLE_TAG
                # self.fail_count may exceed limit here, but I'll let it go
                while count < 3 and self.result == MoveResult.OBSTACLE_TAG:
                    count += 1
                    self.fail_count += 1

                    print('\nObstacle: Found a Tag.')

                    sorted_detections = self._sort_target_tags_left_to_right(
                        self.swarmie.get_latest_targets().detections
                    )

                    # if count == 3:  # last resort
                    #     self.current_state = Planner.STATE_DRIVE
                    #     angle = self._get_angle_to_face(point)
                    #     self.swarmie.turn(
                    #         angle,
                    #         ignore=Obstacle.TAG_TARGET,
                    #         throw=False
                    #     )
                    #     self.result = self.swarmie.drive(
                    #         .75,
                    #         ignore=Obstacle.TAG_TARGET,
                    #         throw=False
                    #     )

                    if len(sorted_detections) == 0:
                        # no tags in view anymore
                        print("I can't see anymore tags, I'll try creeping",
                              "and clearing.")
                        self.current_state = Planner.STATE_DRIVE
                        self.result = self.swarmie.drive(
                            0.1,
                            throw=False
                        )
                        self._clear(
                            math.pi/8,
                            ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET
                        )

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

                        if self.current_state == Planner.STATE_AVOID_LEFT:
                            # Keep going left. Should help avoid bouncing back
                            # and forth between tags just out of view.
                            print("I was turning left last time, so I'll keep",
                                  "it that way.")
                            angle = left_angle
                            dist = left_dist

                        elif self.current_state == Planner.STATE_AVOID_RIGHT:
                            # Keep going right
                            print("I was turning right last time, so I'll",
                                  "keep it that way.")
                            angle = right_angle
                            dist = right_dist

                        else:
                            # pick whichever angle is shortest
                            if abs(right_angle) < abs(left_angle):
                                print('Right looks most clear, turning right.')
                                # print('Right turn makes most sense, turning right.')
                                self.current_state = Planner.STATE_AVOID_RIGHT
                                angle = right_angle
                                dist = right_dist
                            else:
                                print('Left looks most clear, turning left.')
                                # print('Left turn makes most sense, turning left')
                                self.current_state = Planner.STATE_AVOID_LEFT

                        _turn_result, self.result = self._go_around(
                            angle,
                            dist
                        )

            elif self.result == MoveResult.OBSTACLE_SONAR:
                # get around the sonar obstacle
                self.fail_count += 1

                print('\nObstacle: Sonar.')
                obstacle = self.swarmie.get_obstacle_condition()

                if (obstacle & Obstacle.IS_SONAR ==
                    Obstacle.SONAR_RIGHT|Obstacle.SONAR_CENTER):
                    print('Left looks clear, turning left.')
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(math.pi/4, 0.7)
                    # self.swarmie.drive_to(point, throw=False)

                elif (obstacle & Obstacle.IS_SONAR ==
                      Obstacle.SONAR_LEFT|Obstacle.SONAR_CENTER):
                    print('Right looks clear, turning right.')
                    self.current_state = Planner.STATE_AVOID_RIGHT
                    self._go_around(-math.pi/4, 0.7)
                    # self.swarmie.drive_to(point, throw=False)

                elif (obstacle & Obstacle.IS_SONAR ==
                      Obstacle.SONAR_LEFT):
                    print('Only left blocked, turning a little right.')
                    self.current_state = Planner.STATE_AVOID_RIGHT
                    self._go_around(-math.pi/6, 0.6)
                    # self.swarmie.drive_to(point, throw=False)

                elif (obstacle & Obstacle.IS_SONAR ==
                      Obstacle.SONAR_RIGHT):
                    print('Only right blocked, turning a little left.')
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(math.pi/6, 0.6)
                    # self.swarmie.drive_to(point, throw=False)

                else:
                    print('Neither left or right look clear, backing up')
                    self.current_state = Planner.STATE_AVOID_REVERSE
                    self.swarmie.drive(
                        -0.4,
                        ignore=Obstacle.IS_SONAR,
                        throw=False
                    )
                    self._clear(math.pi/4,
                                ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET,
                                reset_heading=False)
                    # just go left
                    self._go_around(math.pi/4, 0.75)

            elif self.result == MoveResult.PATH_FAIL:
                # shit, hope we can back up if this ever happens
                self.fail_count += 1

                print('\nPath Failure. Backing up.')
                self.current_state = Planner.STATE_AVOID_REVERSE
                self.swarmie.drive(
                    -0.5,
                    ignore=Obstacle.IS_SONAR|Obstacle.IS_VISION,
                    throw=False
                )

            self.cur_loc = self.swarmie.get_odom_location()

            if self.fail_count >= Planner.FAIL_COUNT_LIMIT:
                print('Failed to drive to goal {} times.'.format(
                    Planner.FAIL_COUNT_LIMIT)
                )
                raise PathException(MoveResult.PATH_FAIL)

        print('Successfully executed nav plan.')
        return MoveResult.SUCCESS

    def drive(self, distance, tolerance=0.0, tolerance_step=0.5):
        """Convenience wrapper to drive_to(). Drive the given distance, while
        avoiding sonar and target obstacles.

        Args:
        * distance - the distance, in meters, to drive
        * tolerance - how close you'd like to get. See drive_to() for more
        * tolerance_step - how much tolerance is incremented if map/get_plan
          service fails.

        Raises:
        * rospy.ServiceException if /map/get_plan fails. See drive_to() for
          more information.
        """
        self.cur_loc = self.swarmie.get_odom_location()
        start = self.cur_loc.get_pose()

        goal = Point()
        goal.x = start.x + distance * math.cos(start.theta)
        goal.y = start.y + distance * math.sin(start.theta)

        return self.drive_to(
            goal,
            tolerance=tolerance,
            tolerance_step=tolerance_step
        )

    def _get_spiral_points(self, start_distance, distance_step):
        """Get a list of waypoints for the spiral search pattern. Waypoints
        will be used as goals to planner.drive_to()

        Args:
        * start_distance - the distance of the first two legs of the spiral.
        * distance_step - how much to increment the leg length by after driving
          two legs.

        Returns:
        * points - the list of waypoints
        """
        start_loc = self.swarmie.get_odom_location().get_pose()
        points = []
        distance = start_distance
        angle = math.pi/2

        prev_point = Point()
        prev_point.x = start_loc.x + distance * math.cos(start_loc.theta)
        prev_point.y = start_loc.y + distance * math.sin(start_loc.theta)
        points.append(prev_point)

        # todo: is this big enough, or too big?
        for i in range(1, 10):
            if i % 2 == 0:
                distance += distance_step

            point = Point()
            point.x = prev_point.x + distance * math.cos(start_loc.theta
                                                         + angle)
            point.y = prev_point.y + distance * math.sin(start_loc.theta
                                                         + angle)
            points.append(point)
            prev_point = point
            angle += math.pi/2

        return points

    def spiral_home_search(self, start_distance, distance_step=0.5,
                           tolerance=0.0, tolerance_step=0.5):
        """Search for home in a square spiral pattern.

        Args:
        * start_distance - the distance of the first two legs of the spiral.
        * distance_step - how much to increment the leg length by after driving
          two legs.
        * tolerance - the tolerance for the nav plans, see drive_to() for
          more information.
        * tolerance_step - how much to increment the tolerance by if a nav plan
          can't be found.

        Returns:
        * result - MoveResult. MoveResult.OBSTACLE_HOME if home is found.
          MoveResult.PATH_FAIL if home is not found.

        Raises:
        * mobility.Swarmie.PathException - if the rover fails more than
          Planner.FAIL_COUNT_LIMIT times to reach a single waypoint in the
          its current navigation plan.
        * rospy.ServiceException - if a path can't be found in 3
          attempts, beginning with initial tolerance and incrementing by
          tolerance_step on each subsequent attempt.
        """
        drive_result = None
        points = self._get_spiral_points(start_distance, distance_step)

        for point in points:
            drive_result = self.drive_to(
                point,
                tolerance=tolerance,
                tolerance_step=tolerance_step
            )
            if drive_result == MoveResult.OBSTACLE_HOME:
                return drive_result

        return MoveResult.PATH_FAIL


def main():
    """For testing Planner class using RViz nav goals."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'rovername',
        help='required, name of the rover to connect to',
        nargs='?',
        default=None
    )
    args = parser.parse_args()

    if args.rovername is None:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = args.rovername
    swarmie = Swarmie(rovername)

    planner = Planner(swarmie, use_rviz_nav_goal=True)
    rospy.spin()


if __name__ == '__main__':
    main()
