cmake_minimum_required(VERSION 2.8.3)
project(mobility)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Needed to compile grid_map
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  random_numbers
  tf
  apriltags_ros
  rospy
  message_generation
  dynamic_reconfigure
  obstacle_detection
  grid_map_ros
  grid_map_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/params.py
)

add_message_files ( 
	FILES MoveResult.msg MoveRequest.msg
)

add_service_files (
	FILES Core.srv FindTarget.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs 
  random_numbers tf apriltags_ros rospy message_runtime 
  obstacle_detection grid_map_ros grid_map_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mapping
  src/mapping.cpp
)

add_dependencies(mapping ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mapping
  ${catkin_LIBRARIES}
)

catkin_install_python (
	PROGRAMS scripts/core.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
