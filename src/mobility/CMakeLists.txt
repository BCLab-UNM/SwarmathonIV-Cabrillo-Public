cmake_minimum_required(VERSION 2.8.3)
project(mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  tf
  apriltags_ros
  rospy
  message_generation
)

catkin_python_setup()

add_service_files (
	FILES Command.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers tf apriltags_ros rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  mobility 
  src/PickUpController.cpp
  src/DropOffController.cpp
  src/SearchController.cpp
  src/mobility.cpp
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)
