cmake_minimum_required(VERSION 2.8.3)
project(apriltags2to1)

find_package(catkin REQUIRED COMPONENTS
  apriltags2_ros
  message_generation
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  AprilTagDetection.msg AprilTagDetectionArray.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS std_msgs apriltags2_ros message_runtime rospy
)

catkin_install_python(
  PROGRAMS
  scripts/apriltag_converter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
