#!/bin/bash

if [ -d "$(dirname $0)/install" ]; then 
    source "$(dirname $0)/install/setup.bash"
else
    source "$(dirname $0)/devel/setup.bash"
fi

#Delete the rqt cache - can take 24 hours for changes in the UI
# to show up otherwise
rm ~/.config/ros.org/rqt_gui.ini

defworld=~/rover_workspace/simulation/worlds/powerlaw_targets_example.world
exe=$(basename $0)
if [ "$exe" == "dev.sh" ]; then    
    roslaunch ./launch/rover_gui.launch startsim:=true single:=true world:=$defworld
elif [ "$exe" == "prelim.sh" ]; then
    roslaunch ./launch/rover_gui.launch startsim:=true round:=prelim world:=$defworld
elif [ "$exe" == "final.sh" ]; then
    roslaunch ./launch/rover_gui.launch startsim:=true round:=final world:=$defworld
else
    roslaunch ./launch/rover_gui.launch "$@"
fi

# This seems to be necessary to kill rover nodes
pkill roslaunch

