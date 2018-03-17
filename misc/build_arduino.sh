#! /bin/bash

if [ -x ~/arduino-1.*/arduino ]; then
    arduino=~/arduino-1.*/arduino
elif [ -x /opt/arduino-1.*/arduino ]; then
    arduino=/opt/arduino-1.*/arduino
else
    echo "Couldn't find an Arduino executable."
    exit 1
fi

echo Using Arduino executable: $arduino

repo=$(catkin locate)/arduino
build=$repo/build
sketch=$repo/swarmie_control/swarmie_control.ino

if [ -z "$1" ]; then
    sketch=$repo/swarmie_control/swarmie_control.ino
else
    sketch="$1"
fi

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  dev=/dev/ttyACM0
else
  dev=/dev/ttyACM1
fi

echo "Building for Leonardo on $dev"
echo Sketch: $sketch
echo Build Dir: $build

$arduino --upload --preserve-temp-files --pref serial.port=$dev --pref build.verbose=true --pref upload.verbose=true --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch
