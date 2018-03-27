cmake_minimum_required(VERSION 2.8.3)
project(swarmie)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swarmie
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install (
  DIRECTORY launch/
  DESTINATION launch/
)

install (
  DIRECTORY misc/
  DESTINATION misc/
)
