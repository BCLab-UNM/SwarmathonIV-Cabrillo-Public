#!/bin/bash

set -e

if [ -z "$1" ]
then
    export ROS_MASTER_URI=http://$(echo $SSH_CLIENT | cut -d' ' -f 1):11311
else
    export ROS_MASTER_URI=http://$1:11311
fi

projdir=$(dirname $0)/..

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  swarmie_dev=/dev/ttyACM0
  ublox_dev=/dev/ttyACM1
else
  swarmie_dev=/dev/ttyACM1
  ublox_dev=/dev/ttyACM0
fi

launchfile="$projdir/launch/swarmie.launch"
if [ -f ./launch/swarmie.launch ]; then
    launchfile=./launch/swarmie.launch
fi

echo "Loading Arduino code."
if [ -x ~/arduino-1.*/arduino ]; then
    arduino=~/arduino-1.*/arduino
elif [ -x /opt/arduino-1.*/arduino ]; then
    arduino=/opt/arduino-1.*/arduino
else
    echo "Couldn't find an Arduino executable."
    exit 1
fi

echo Using Arduino executable: $arduino

if [ -f $projdir/arduino/swarmie_control/swarmie_control.ino ]; then
    repo=$(realpath $projdir/arduino)
elif [ -f $(catkin locate)/arduino/swarmie_control/swarmie_control.ino ]; then
    repo=$(catkin locate)/arduino
else
    echo "Couldn't locate Arduino sketch."
    exit 1 
fi

sketch=$repo/swarmie_control/swarmie_control.ino
build=$repo/build

echo Sketch: $sketch
echo Build Dir: $build

$arduino --upload --preserve-temp-files --pref serial.port=$swarmie_dev --pref build.verbose=1 --pref upload.verbose=1 --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch

if [ -f $projdir/devel/setup.bash ]; then
    echo "deploy_host.sh not called assuming rover_onboard_node_launch"
    source /opt/ros/kinetic/setup.bash
    source $projdir/devel/setup.bash
fi

roslaunch $launchfile name:=$(hostname) simulation:=False swarmie_dev:=$swarmie_dev ublox_dev:=$ublox_dev
