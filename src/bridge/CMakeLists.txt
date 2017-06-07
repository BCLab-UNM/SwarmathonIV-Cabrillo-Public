cmake_minimum_required(VERSION 2.8.3)
project(bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs std_msgs tf nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
)

add_executable(
  sbridge src/sbridge.cpp
)

add_executable(
  abridge src/abridge.cpp src/usbSerial.cpp
)

target_link_libraries(
  abridge
  ${catkin_LIBRARIES}
)

target_link_libraries(
  sbridge
  ${catkin_LIBRARIES}
)
