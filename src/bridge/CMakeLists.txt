cmake_minimum_required(VERSION 2.8.3)
project(bridge)

set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  nav_msgs
  dynamic_reconfigure
  swarmie_msgs
)

generate_dynamic_reconfigure_options(
    cfg/pid.py
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs sensor_msgs std_msgs tf nav_msgs dynamic_reconfigure swarmie_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(
  sbridge src/sbridge.cpp src/pid.cpp
)

add_dependencies(sbridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  abridge src/abridge.cpp src/usbSerial.cpp src/pid.cpp
)

add_dependencies(abridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(
  abridge
  ${catkin_LIBRARIES}
)

target_link_libraries(
  sbridge
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS abridge abridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS sbridge sbridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
