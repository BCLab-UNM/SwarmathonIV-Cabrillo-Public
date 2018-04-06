#! /bin/bash

arduino=/opt/arduino/arduino
repo=~/arduino
sketch=$repo/Calibrate/Calibrate.ino

if udevadm info /dev/ttyACM0 | grep -q Leonardo; then
  dev=/dev/ttyACM0
else
  dev=/dev/ttyACM1
fi

$arduino --upload --pref serial.port=$dev --pref build.verbose=true --pref upload.verbose=true --pref sketchbook.path=$repo --pref board=leonardo $sketch
minicom -D $dev -b 9600 --noinit -8
