#! /usr/bin/env python

from __future__ import print_function

import sys
import math
import rospy
import tf

from geometry_msgs.msg import Pose2D, PoseStamped
from swarmie_msgs.msg import Obstacle
from mobility.swarmie import Swarmie
from planner import Planner
from apriltags2_ros.msg import AprilTagDetection

def convert_to_Pose2D(t):
    global swarmie
    t2 = PoseStamped()
    t2.header = t.pose.header
    t2.pose = t.pose.pose
    t = t2
    swarmie.xform.waitForTransform(swarmie.rover_name + '/odom', t.header.frame_id, t.header.stamp, rospy.Duration(5.0))
    print("swarmie.rover_name:", swarmie.rover_name)
    odom_pose = swarmie.xform.transformPose(swarmie.rover_name + '/odom', t.pose)
    print("odom_pose:",type(odom_pose))
    quat = [odom_pose.pose.orientation.x, odom_pose.pose.orientation.y,
            odom_pose.pose.orientation.z, odom_pose.pose.orientation.w,
            ]
    (_r, _p, y) = tf.transformations.euler_from_quaternion(quat)
    pose = Pose2D()
    pose.x = odom_pose.pose.position.x
    pose.y = odom_pose.pose.position.y
    pose.theta = y
    return(pose)

def _sort_home_tags_nearest_center(detections):
        # type: (List[AprilTagDetection]) -> List[AprilTagDetection]
        """Sort home tags (id == 256) in view by their distance from the center
        of the camera's field of view.

        Args:
        * detections - List[AprilTagDetection] the list
          of detections.

        Returns:
        * sorted_detections - sorted list of AprilTagDetections in view. Will
          be empty if no tags are in view.
        """
        sorted_detections = []

        for detection in detections:  # type: AprilTagDetection
            if detection.id[0] == 256:
                sorted_detections.append(detection)
        return sorted(sorted_detections,
                      key=lambda x: abs(x.pose.pose.pose.position.x))

#apriltags2_ros/AprilTagDetectionArray
#std_msgs/Header header
#  uint32 seq
#  time stamp
#  string frame_id
#apriltags2_ros/AprilTagDetection[] detections
#  int32[] id
#  float64[] size
#  geometry_msgs/PoseWithCovarianceStamped pose
#    std_msgs/Header header
#      uint32 seq
#      time stamp
#      string frame_id
#    geometry_msgs/PoseWithCovariance pose
#      geometry_msgs/Pose pose
#        geometry_msgs/Point position
#          float64 x
#          float64 y
#          float64 z
#        geometry_msgs/Quaternion orientation
#          float64 x
#          float64 y
#          float64 z
#          float64 w
#      float64[36] covariance
def convert_PoseWithCovarianceStamped_to_PoseStamped(PwCS):
    ps = PoseStamped()
    ps.header = PwCS.pose.header
    ps.pose = PwCS.pose.pose.pose
    return ps

def main(s, **kwargs):
    '''Dropoff throws IndexError when no tags near swarmie '''
    global swarmie 
    
    swarmie = s 

    p = Planner(swarmie)
    home_detections = _sort_home_tags_nearest_center( p.swarmie.get_latest_targets().detections )
    swarmie.print_infoLog(swarmie.rover_name + ": Dropoff: Faceing Home")
    p.face_home_tag()
    rospy.sleep(1)
    swarmie.drive_to(swarmie.get_home_odom_location(), ignore=Obstacle.IS_VISION|Obstacle.IS_SONAR)
    try:
        swarmie.set_wrist_angle(.7)
        rospy.sleep(.4)
        if(swarmie.simulator_running()):
            swarmie.fingers_open()
        else:
            swarmie.set_finger_angle(1)
        rospy.sleep(.4)
        swarmie.set_wrist_angle(0)
        swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR)
    except: 
        swarmie.drive(-1, ignore=Obstacle.IS_VISION | Obstacle.IS_SONAR) #make sure to get out of home
        raise
    return 0 

if __name__ == '__main__' :
    sys.exit(main(Swarmie()))
