#! /bin/bash

set -e

if [ -z "$1" ]; then
    echo "usage: $0 <rovername>"
    exit -1
fi

# Build deploy image
(
    cd ~/
    tar -cf - rover_workspace/camera_info rover_workspace/launch rover_workspace/misc rover_workspace/src rover_workspace/*.sh | ssh robot@$1 tar -xvf -    
)

ssh -t robot@$1 /home/robot/rover_workspace/misc/deploy_rover.sh
