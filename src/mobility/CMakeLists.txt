cmake_minimum_required(VERSION 2.8.3)
project(mobility)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  random_numbers
  tf
  apriltags_ros
  rospy
  message_generation
  dynamic_reconfigure
  grid_map_ros
  grid_map_msgs
  nav_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/drive.py
)

add_message_files ( 
	FILES MoveResult.msg MoveRequest.msg 
)

add_service_files (
	FILES Core.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs apriltags_ros nav_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs 
  random_numbers tf apriltags_ros rospy message_runtime 
  grid_map_ros grid_map_msgs dynamic_reconfigure
  nav_msgs
)

catkin_install_python (
	PROGRAMS 
	scripts/core.py 
	scripts/rdb.py 
	scripts/cal_imu.py
	scripts/test_square.py
	scripts/task.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	DIRECTORY camera_info
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
