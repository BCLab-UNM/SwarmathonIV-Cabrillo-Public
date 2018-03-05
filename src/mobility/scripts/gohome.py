#! /usr/bin/env python

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

from mobility.swarmie import Swarmie, HomeException, Location

def drive_straight_home_gps() :
    global swarmie 
    
    # Use GPS to figure out about where we are. 
    # FIXME: We need to hanlde poor GPS fix. 
    loc = swarmie.wait_for_fix(distance=4, time=60).get_pose()
    home = swarmie.get_home_gps_location()

    
    dist = math.hypot(loc.y - home.y, 
                      loc.x - home.x)
    
    angle = angles.shortest_angular_distance(loc.theta, 
                                             math.atan2(home.y - loc.y,
                                                        home.y - loc.x))
    
    swarmie.turn(angle, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)
    swarmie.drive(dist, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)

def drive_straight_home_odom() :
    global swarmie 

    # We remember home in the Odom frame when we see it. Unlike GPS
    # there's no need to translate the location into r and theta. The
    # swarmie's drive_to function takes a point in odometry space. 
    
    home = swarmie.get_home_odom_location() 
    swarmie.drive_to(home, ignore=Obstacle.TAG_TARGET | Obstacle.SONAR_CENTER)


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
                rovername + '/goal',
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
        OVERSHOOT_DIST = 0.25  # meters, distance to overshoot target by
        base_link_pose = self._transform_to_base_link(detection)
        radius = math.sqrt(base_link_pose.pose.position.x**2
                           + base_link_pose.pose.position.y**2)
        tag_point = Point(x=base_link_pose.pose.position.x,
                          y=base_link_pose.pose.position.y)

        path_edge_point = Point()
        # solve for x given the radius and y-coord of a point on a circle
        path_edge_point.x = math.sqrt(radius**2 - tag_point.y**2)
        path_edge_point.y = Planner.PATHWAY_EDGE_DIST
        if direction == 'left':
            path_edge_point.y *= -1

        return (-self._angle_between(tag_point, path_edge_point),
                path_edge_point.x + OVERSHOOT_DIST)

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

    def _sort_left_to_right(self, detections):
        """Sort tags in view from left to right (by their x position in the
        camera frame).

        Args:
        * detections - apriltags_ros/AprilTagDetectionArray the list
          of detections.

        Returns:
        * sorted_detections - sorted list of AprilTagDetections in view. Will
          be empty if no tags are in view.
        """
        return sorted(detections, key=lambda x : x.pose.pose.position.x)

    def _clear(self, angle, ignore=Obstacle.IS_SONAR):
        """Turn right, then left, then back to start heading.
        Helps to clear and mark the map if in a difficult spot.

        Args:
        * angle - the angle to turn
        * ignore - the stuff to ignore. See Swarmie.drive()
        """
        start_heading = self.swarmie.get_odom_location().get_pose().theta
        self.swarmie.set_heading(start_heading-angle, ignore=ignore)
        self.swarmie.set_heading(start_heading+angle, ignore=ignore)
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

        Raises:
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

        while (not self.cur_loc.at_goal(self.goal, Planner.DISTANCE_OK)
               and self.fail_count < Planner.FAIL_COUNT_LIMIT):

            # get new plan and try to drive to first point in it
            print('\nGetting new nav plan.')

            for i in range(4):
                try:
                    self.plan = self.swarmie.get_plan(
                        goal,
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
            point = Point(x=pose.pose.position.x, y=pose.pose.position.y)
            start = self.cur_loc.get_pose()

            self.current_state = Planner.STATE_DRIVE
            turn_angle = angles.shortest_angular_distance(
                start.theta,
                math.atan2(point.y - start.y, point.x - start.x)
            )
            # Turn to approximate goal heading while ignoring sonar and tags
            # helps to prevent rover from trying to jump around obstacles
            # before it even starts along its new path
            self.swarmie.turn(turn_angle,
                              ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET)
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
                return

            elif self.result == MoveResult.OBSTACLE_TAG:
                # get around the tag obstacle
                count = 0
                angle = 0  # argument to _go_around() below
                dist = 0  # argument to _go_around() below

                # Give the rover 3 tries to avoid any tags nearby before
                # getting a new nav plan. MoveResult.OBSTACLE_SONAR takes
                # priority in the driver code, so it should be safe to continue
                # this loop if the MoveResult is just an OBSTACLE_TAG
                # self.fail_count may exceed limit here, but I'll let it go
                while count < 3 and self.result == MoveResult.OBSTACLE_TAG:
                    count += 1
                    self.fail_count += 1

                    print('\nObstacle: Found a Tag.')

                    sorted_detections = self._sort_left_to_right(
                        self.swarmie.get_latest_targets().detections
                    )

                    if len(sorted_detections) == 0:
                        # no tags in view anymore
                        print("I can't see anymore tags, I'll try creeping",
                              "and clearing.")
                        self.current_state = Planner.STATE_DRIVE
                        self.result = self.swarmie.drive(
                            0.1,
                            throw=False
                        )
                        self._clear(math.pi/8, ignore=Obstacle.TAG_TARGET)

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
                                self.current_state = Planner.STATE_AVOID_RIGHT
                                angle = right_angle
                                dist = right_dist
                            else:
                                print('Left looks most clear, turning left.')
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

                if obstacle & Obstacle.SONAR_LEFT == 0:
                    print('Left looks clear, turning left.')
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(math.pi/4, 0.75)
                    self.swarmie.drive_to(point, throw=False)

                elif obstacle & Obstacle.SONAR_RIGHT == 0:
                    print('Right looks clear, turning right.')
                    self.current_state = Planner.STATE_AVOID_LEFT
                    self._go_around(-math.pi/4, 0.75)
                    self.swarmie.drive_to(point, throw=False)

                else:
                    print('Neither left or right look clear, backing up')
                    self.current_state = Planner.STATE_AVOID_REVERSE
                    self.swarmie.drive(
                        -0.2,
                        ignore=Obstacle.IS_SONAR,
                        throw=False
                    )
                    self._clear(math.pi/8,
                                ignore=Obstacle.IS_SONAR|Obstacle.TAG_TARGET)

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
            # break  # get new plan now, current one isn't working

            self.cur_loc = self.swarmie.get_odom_location()

        if self.fail_count >= Planner.FAIL_COUNT_LIMIT:
            print('Failed to drive to goal {} times.'.format(
                Planner.FAIL_COUNT_LIMIT)
            )
            return

        print('Successfully executed nav plan.')

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

        self.drive_to(goal, tolerance=tolerance, tolerance_step=tolerance_step)


def main():
    global swarmie 
    global rovername

    parser = argparse.ArgumentParser()
    parser.add_argument(
        'rovername',
        help='required, name of the rover to connect to',
        nargs='?',
        default=None
    )
    parser.add_argument(
        '-r',
        '--use_rviz',
        action='store_true',
        help='Use RViz 2D Nav Goal to request goals from path planner.'
    )
    args = parser.parse_args()
    
    if args.rovername is None:
        print ('usage:', sys.argv[0], '<rovername>')
        exit (-1)

    rovername = args.rovername
    swarmie = Swarmie(rovername)

    if args.use_rviz:
        planner = Planner(swarmie, use_rviz_nav_goal=True)
        # nav_goal_sub = rospy.Subscriber(
        #     rovername + '/goal',
        #     PoseStamped,
        #     rviz_nav_goal_cb,
        #     queue_size=1
        # )
        rospy.spin()
    else:
        try :
            # Try driving home with odometry before GPS
            drive_straight_home_odom()
            while not rospy.is_shutdown() :
                drive_straight_home_gps()
        except HomeException as e:
            # Found home!
            exit(0)
    
if __name__ == '__main__' : 
    main()
