#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$HOME/wheeltec_ros2/install/setup.bash}
ROBOT_INIT_DELAY=${ROBOT_INIT_DELAY:-2}

cd "$APP_ROOT"

if [ "$ROBOT_INIT_DELAY" != "0" ]; then
  echo "[INFO] Waiting ${ROBOT_INIT_DELAY}s before starting robot base bringup"
  sleep "$ROBOT_INIT_DELAY"
fi

if [ -f "$ROS_SETUP" ]; then
  set +u
  . "$ROS_SETUP"
  set -u
else
  echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

if [ -f "$WORKSPACE_SETUP" ]; then
  set +u
  . "$WORKSPACE_SETUP"
  set -u
else
  echo "[ERROR] Workspace setup not found: $WORKSPACE_SETUP" >&2
  exit 1
fi

echo "[INFO] Starting ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py"
exec ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
