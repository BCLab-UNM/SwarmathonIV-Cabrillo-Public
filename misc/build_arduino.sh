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

repo=$(catkin locate)/Swarmathon-Arduino
build=$repo/build
sketch=$repo/Swarmathon_Arduino/Swarmathon_Arduino.ino

if [ -z "$1" ]; then
    sketch=$repo/Swarmathon_Arduino/Swarmathon_Arduino.ino
else
    sketch="$1"
fi

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  dev=/dev/ttyACM0
else
  dev=/dev/ttyACM1
fi

echo "Building for Leonardo on $dev"

$arduino --upload --preserve-temp-files --pref serial.port=$dev --pref build.verbose=true --pref upload.verbose=true --pref build.path=$build --pref sketchbook.path=$repo --pref board=leonardo $sketch
