#!/bin/bash

projdir=$(dirname $0)/..

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  swarmie_dev=/dev/ttyACM0
  ublox_dev=/dev/ttyACM1
else
  swarmie_dev=/dev/ttyACM1
  ublox_dev=/dev/ttyACM0
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

if [ -f ./arduino/CabrilloCalibrator/CabrilloCalibrator.ino ]; then
    repo=$(realpath ./arduino)
elif [ -f $(catkin locate)/arduino/CabrilloCalibrator/CabrilloCalibrator.ino ]; then
    repo=$(catkin locate)
else
    echo "Couldn't locate calibration sketch."
    exit 1 
fi

sketch=$repo/CabrilloCalibrator/CabrilloCalibrator.ino
build=$repo/build

$arduino --upload --preserve-temp-files --pref serial.port=$swarmie_dev --pref build.verbose=1 --pref upload.verbose=1 --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch

roscore &
sleep 1
rosrun rosserial_python serial_node.py $swarmie_dev &
sleep 2
python $projdir/misc/do_cal.py 
pkill rosrun
pkill roscore
