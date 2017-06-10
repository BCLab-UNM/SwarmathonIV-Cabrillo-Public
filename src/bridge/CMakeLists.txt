cmake_minimum_required(VERSION 2.8.3)
project(bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  nav_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/params.py
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs std_msgs tf nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/bridgeConfig.cmake
)

add_executable(
  sbridge src/sbridge.cpp src/pid.cpp
)

add_executable(
  abridge src/abridge.cpp src/usbSerial.cpp src/pid.cpp
)

add_dependencies(sbridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(abridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  abridge
  ${catkin_LIBRARIES}
)

target_link_libraries(
  sbridge
  ${catkin_LIBRARIES}
)
