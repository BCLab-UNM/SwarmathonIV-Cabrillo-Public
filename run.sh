#!/bin/bash

#Delete the rqt cache - can take 24 hours for changes in the UI
# to show up otherwise
rm ~/.config/ros.org/rqt_gui.ini

defworld=~/rover_workspace/simulation/worlds/powerlaw_targets_example.world
exe=$(basename $0)
if [ "$exe" == "dev.sh" ]; then    
    roslaunch ./launch/rover_gui.launch startsim:=true single:=true world:=$defworld
    #rqt -s rqt_rover_gui --args --startsim --single --world $defworld "$@"
elif [ "$exe" == "prelim.sh" ]; then
    roslaunch ./launch/rover_gui.launch startsim:=true round:=prelim world:=$defworld
    #rqt -s rqt_rover_gui --args --startsim --prelim --world $defworld "$@"
elif [ "$exe" == "final.sh" ]; then
    roslaunch ./launch/rover_gui.launch startsim:=true round:=final world:=$defworld
    #rqt -s rqt_rover_gui --args --startsim --final --powerlaw "$@"  
else
    roslaunch ./launch/rover_gui.launch 
fi

# This seems to be necessary to kill rover nodes
pkill roslaunch

