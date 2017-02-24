#!/bin/bash
echo "Running in $PWD" 
previous_gazebo_model_path=${GAZEBO_MODEL_PATH}
previous_gazebo_plugin_path=${GAZEBO_PLUGIN_PATH}
export SWARMATHON_APP_ROOT="$PWD"
export GAZEBO_MODEL_PATH="$PWD/simulation/models"
export GAZEBO_PLUGIN_PATH="$PWD/build/gazebo_plugins"
source "$PWD/devel/setup.bash"
echo Cleaning up ROS and Gazebo Processes

#Delete the rqt cache - can take 24 hours for changes in the UI
# to show up otherwise
rm ~/.config/ros.org/rqt_gui.ini

./cleanup.sh
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
roscore &
sleep 2

defworld=~/rover_workspace/simulation/worlds/powerlaw_targets_example.world
exe=$(basename $0)
if [ "$exe" == "dev.sh" ]; then
  echo "Executing single rover test mode."
  rqt -s rqt_rover_gui --args --startsim --single --world $defworld "$@"
elif [ "$exe" == "prelim.sh" ]; then
  rqt -s rqt_rover_gui --args --startsim --prelim --world $defworld "$@"
elif [ "$exe" == "final.sh" ]; then
  rqt -s rqt_rover_gui --args --startsim --final --powerlaw "$@"  
else
  rqt -s rqt_rover_gui --args "$@"
fi

# The rover program cleans up after itself but if there is a crash this helps to make sure there are no leftovers
echo Cleaning up ROS and Gazebo Processes
rosnode kill -a 
echo Killing rosmaster
pkill rosmaster
echo Killing roscore
pkill roscore
./cleanup.sh
# Restore previous environment
export GAZEBO_MODEL_PATH=$previous_gazebo_model_path
export GAZEBO_PLUGIN_PATH=$previous_gazebo_plugin_path
