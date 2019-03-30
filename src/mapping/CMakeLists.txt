cmake_minimum_required(VERSION 2.8.3)
project(mapping)

set(CMAKE_BUILD_TYPE "Release")

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
  apriltags2to1
  message_generation
  grid_map_ros
  grid_map_msgs
  swarmie_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/mapping.cfg
)

add_service_files (
	FILES FindTarget.srv GetMap.srv GetNavPlan.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs apriltags2to1 nav_msgs grid_map_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  tf apriltags2to1 message_runtime
  grid_map_ros grid_map_msgs 
  nav_msgs swarmie_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(
  mapping
  src/mapping.cpp
)

add_dependencies(mapping ${catkin_EXPORTED_TARGETS} mapping_gencpp
  ${PROJECT_NAME}_gencfg)

target_link_libraries(
  mapping
  ${catkin_LIBRARIES}
)

install(TARGETS mapping
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
