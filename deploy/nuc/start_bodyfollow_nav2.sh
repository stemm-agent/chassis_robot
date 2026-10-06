#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_ROOT=${WORKSPACE_ROOT:-$HOME/wheeltec_ros2}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$WORKSPACE_ROOT/install/setup.bash}
BODYFOLLOW_PACKAGE=${BODYFOLLOW_PACKAGE:-bodyreader}
BODYFOLLOW_LAUNCH_FILE=${BODYFOLLOW_LAUNCH_FILE:-bodyfollow_nav2.launch.py}
BODYFOLLOW_START_DELAY=${BODYFOLLOW_START_DELAY:-5}

if [ -f "$ROS_SETUP" ]; then
  set +u
  . "$ROS_SETUP"
  set -u
else
  echo "[ERROR] ROS 2 setup not found: $ROS_SETUP"
  exit 1
fi

if [ -f "$WORKSPACE_SETUP" ]; then
  set +u
  . "$WORKSPACE_SETUP"
  set -u
else
  echo "[ERROR] Workspace setup not found: $WORKSPACE_SETUP"
  exit 1
fi

cd "$WORKSPACE_ROOT"

if [ "$BODYFOLLOW_START_DELAY" != "0" ]; then
  echo "[INFO] Waiting ${BODYFOLLOW_START_DELAY}s for robot bringup to settle"
  sleep "$BODYFOLLOW_START_DELAY"
fi

echo "[INFO] Starting ros2 launch ${BODYFOLLOW_PACKAGE} ${BODYFOLLOW_LAUNCH_FILE}"
exec ros2 launch "$BODYFOLLOW_PACKAGE" "$BODYFOLLOW_LAUNCH_FILE"
