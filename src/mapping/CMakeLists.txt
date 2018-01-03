cmake_minimum_required(VERSION 2.8.3)
project(mapping)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# Needed to compile grid_map
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  apriltags_ros
  message_generation
  grid_map_ros
  grid_map_msgs
  swarmie_msgs
  cv_bridge
)

catkin_python_setup()

add_service_files (
	FILES FindTarget.srv LatestTarget.srv GetMap.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs apriltags_ros nav_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs 
  tf apriltags_ros message_runtime 
  grid_map_ros grid_map_msgs 
  nav_msgs swarmie_msgs cv_bridge 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mapping
  src/mapping.cpp
)

add_dependencies(mapping ${catkin_EXPORTED_TARGETS} mapping_gencpp)

target_link_libraries(
  mapping
  ${catkin_LIBRARIES}
)

install(TARGETS mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python (
	PROGRAMS 
	scripts/findcenter.py 
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
