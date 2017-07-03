cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  message_generation
  apriltags_ros
)

add_message_files ( 
	FILES Obstacle.msg
)

add_service_files (
	FILES DetectionMask.srv
)

generate_messages(DEPENDENCIES)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs message_filters message_runtime apriltags_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

add_executable(
  obstacle src/obstacle.cpp
)

add_dependencies(obstacle ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  obstacle
  ${catkin_LIBRARIES}
)
