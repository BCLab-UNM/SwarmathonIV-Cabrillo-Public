#! /bin/bash

pkill -f /opt/ros/indigo/lib/tf/static_transform_publisher
pkill -f /opt/ros/indigo/lib/usb_cam/usb_cam_node
pkill -f /home/robot/rover_workspace/devel/lib/mobility/mobility
pkill -f /home/robot/rover_workspace/devel/lib/obstacle_detection/obstacle
pkill -f /home/robot/rover_workspace/devel/lib/diagnostics/diagnostics
pkill -f /opt/ros/indigo/lib/apriltags_ros/apriltag_detector_node
pkill -f /home/robot/rover_workspace/devel/lib/abridge/abridge
pkill -f /home/robot/rover_workspace/devel/lib/ublox_gps/ublox_gps
pkill -f /opt/ros/indigo/lib/robot_localization/navsat_transform_node
pkill -f /opt/ros/indigo/lib/robot_localization/ekf_localization_node
pkill -f /opt/ros/indigo/lib/robot_localization/ekf_localization_node

