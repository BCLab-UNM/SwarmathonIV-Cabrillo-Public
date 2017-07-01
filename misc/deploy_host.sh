#! /bin/bash

set -e

# Build deploy image
(
    cd ~/
    tar -cf - rover_workspace/camera_info rover_workspace/launch rover_workspace/misc rover_workspace/src rover_workspace/*.sh | ssh robot@lovelace tar -xvf -    
)

ssh -t robot@lovelace /home/robot/rover_workspace/misc/deploy_rover.sh

