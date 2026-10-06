#!/usr/bin/env bash
set -euo pipefail
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_ROOT=${WORKSPACE_ROOT:-${HOME}/wheeltec_ros2}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-${WORKSPACE_ROOT}/install/setup.bash}
if [ ! -e /dev/astra_s ]; then
  echo '[ERROR] Astra S alias /dev/astra_s is unavailable' >&2
  exit 1
fi
set +u
. "$ROS_SETUP"
. "$WORKSPACE_SETUP"
set -u
cd "$WORKSPACE_ROOT"
exec ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py camera_mode:=astra
