#!/bin/bash
set -eo pipefail
: "${F1_WORKSPACE:?Set F1_WORKSPACE to the absolute workspace root}"
source /opt/ros/humble/setup.bash
source "$F1_WORKSPACE/install/local_setup.bash"
set -u
exec ros2 launch f1_operator_gateway operator_system.launch.py \
  "workspace_root:=$F1_WORKSPACE" bind_address:=0.0.0.0 \
  "output_enabled:=${F1_OUTPUT_ENABLED:-false}"
