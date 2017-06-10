#! /bin/bash

set -e
echo === Building Rover Software === 
(
  cd ~/rover_workspace
  if ping -q -c 1 -w 2 -n www.google.com >/dev/null 2>&1; then 
    git pull
  else
    echo "Skiping git pull because we're not connected to the internet"
  fi 
  catkin build
)

echo === Starting Rover ===
~/rover_workspace/misc/rover_onboard_node_launch.sh $(echo $SSH_CLIENT | cut -d' ' -f 1)

