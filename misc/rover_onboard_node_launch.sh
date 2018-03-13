#!/bin/bash

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

if [ -f ./Swarmathon-Arduino/Swarmathon_Arduino/Swarmathon_Arduino.ino ]; then
    repo=$(realpath ./Swarmathon-Arduino)
elif [ -f $(catkin locate)/Swarmathon-Arduino/Swarmathon_Arduino/Swarmathon_Arduino.ino ]; then
    repo=$(catkin locate)
else
    echo "Couldn't locate Arduino sketch."
    exit 1 
fi

sketch=$repo/Swarmathon_Arduino/Swarmathon_Arduino.ino
build=$repo/build

$arduino --upload --preserve-temp-files --pref serial.port=$swarmie_dev --pref build.verbose=1 --pref upload.verbose=1 --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch

source /opt/ros/kinetic/setup.bash
source $repo/devel/setup.bash

roslaunch $launchfile name:=$(hostname) simulation:=False swarmie_dev:=$swarmie_dev ublox_dev:=$ublox_dev
