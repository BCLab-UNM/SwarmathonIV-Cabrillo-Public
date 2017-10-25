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
  grid_map_ros
  grid_map_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/drive.py
)

add_message_files ( 
	FILES MoveResult.msg MoveRequest.msg Obstacle.msg
)

add_service_files (
	FILES Core.srv FindTarget.srv LatestTarget.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs apriltags_ros)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs 
  random_numbers tf apriltags_ros rospy message_runtime 
  grid_map_ros grid_map_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mapping
  src/mapping.cpp
)

add_dependencies(mapping ${catkin_EXPORTED_TARGETS} mobility_gencpp)

target_link_libraries(
  mapping
  ${catkin_LIBRARIES}
)

catkin_install_python (
	PROGRAMS 
	scripts/core.py 
	scripts/task.py 
	scripts/rdb.py 
	scripts/search.py 
	scripts/pickup.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
