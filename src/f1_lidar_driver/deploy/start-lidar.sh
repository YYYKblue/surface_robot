#!/bin/bash
# Avoid loading login profiles and overlays belonging to the old ros2_ws.
set -eo pipefail
: "${F1_LIDAR_WORKSPACE:?Set F1_LIDAR_WORKSPACE to the absolute surface_ws root}"
source /opt/ros/humble/setup.bash
source "$F1_LIDAR_WORKSPACE/install/local_setup.bash"
set -u
f1_lidar_config="${F1_LIDAR_CONFIG:-$F1_LIDAR_WORKSPACE/install/f1_lidar_driver/share/f1_lidar_driver/config/pavo2.yaml}"
exec ros2 launch f1_lidar_driver lidar.launch.py "config:=$f1_lidar_config"
