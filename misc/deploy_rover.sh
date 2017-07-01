#! /bin/bash

#
# Run the system via non-interactive shell
#
source /opt/ros/kinetic/setup.bash
source ~/rover_workspace/devel/setup.bash

set -e

cd ~/rover_workspace
touch src/gazebo_plugins/CATKIN_IGNORE
touch src/rqt_rover_gui/CATKIN_IGNORE
catkin build --no-status --no-color
./misc/rover_onboard_node_launch.sh
