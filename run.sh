#!/bin/bash

if [ -f "$(dirname $0)/devel/setup.bash" ]; then
    source "$(dirname $0)/devel/setup.bash"
else
    echo "ERROR: You must compile your code before you run this script."
    echo "ERROR: Run the following command:" 
    echo ""
    echo "  catkin build"
    echo ""
    exit -1
fi

function get_rover()
{
    rovercount=$(rostopic list | grep driveControl | wc -l)
    if [ "$rovercount" -gt 1 ]; then
	echo "ERROR: You can't use this script when there are multiple rovers."
	echo "ERROR: Use rosrun instead."
	echo ""
	echo 'ROS_NAMESPACE=/<rovername> rosrun mobility <node> <arguments>'
	echo ""
	exit -2
    elif [ "$rovercount" -eq 0 ]; then
	echo "ERROR: There are no rovers to connect to! You must start the GUI"
	echo "ERROR: and either begin a simulation or deploy to a rover."
	echo ""
	exit -2
    fi
    rover=$(rostopic list | grep driveControl | cut -d/ -f 2)
}

exe=$(basename $0)
if [ "$exe" == "run.sh" ]; then    
    # Delete the rqt cache - can take 24 hours for changes in the UI
    # to show up otherwise
    rm ~/.config/ros.org/rqt_gui.ini
    if [ $1 = "-noMM" ]; then
        shift
        echo -e "\e[1;36m Running GUI without Multimaster \e[0m"
        roslaunch ./launch/rover_gui_no_multimaster.launch "$@"
    else
        echo -e "\e[1;36m Running GUI with Multimaster \e[0m"
        roslaunch ./launch/rover_gui.launch "$@"
    fi
    
elif [ "$exe" == "dev.sh" ]; then
    get_rover
    echo "Connecting to rover $rover"
    ROS_NAMESPACE="/$rover" rosrun mobility "$@"
elif [ "$exe" == "rdb.sh" ]; then
    get_rover
    echo "Connecting to rover $rover"
    ROS_NAMESPACE="/$rover" rosrun mobility rdb.py
fi
