#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$HOME/wheeltec_ros2/install/setup.bash}
MIC_INIT_DELAY=${MIC_INIT_DELAY:-3}

cd "$APP_ROOT"

if [ -f "$ROS_SETUP" ]; then
  set +u
  . "$ROS_SETUP"
  set -u
fi

if [ -f "$WORKSPACE_SETUP" ]; then
  set +u
  . "$WORKSPACE_SETUP"
  set -u
else
  echo "[WARN] Workspace setup not found: $WORKSPACE_SETUP"
fi

cleanup() {
  trap - EXIT INT TERM

  if [ -n "${BASE_PID:-}" ] && kill -0 "$BASE_PID" >/dev/null 2>&1; then
    kill "$BASE_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${MIC_INIT_PID:-}" ] && kill -0 "$MIC_INIT_PID" >/dev/null 2>&1; then
    kill "$MIC_INIT_PID" >/dev/null 2>&1 || true
  fi

  wait ${BASE_PID:-} ${MIC_INIT_PID:-} 2>/dev/null || true
}

trap cleanup EXIT INT TERM

echo "[INFO] Starting ros2 launch wheeltec_mic_ros2 mic_init.launch.py"
ros2 launch wheeltec_mic_ros2 mic_init.launch.py &
MIC_INIT_PID=$!

echo "[INFO] Waiting ${MIC_INIT_DELAY}s before starting base.launch.py"
sleep "$MIC_INIT_DELAY"

echo "[INFO] Starting ros2 launch wheeltec_mic_ros2 base.launch.py"
ros2 launch wheeltec_mic_ros2 base.launch.py &
BASE_PID=$!

set +e
wait -n "$MIC_INIT_PID" "$BASE_PID"
EXIT_CODE=$?
set -e

echo "[WARN] One wheeltec_mic launch exited with code ${EXIT_CODE}, stopping the other one"
exit "$EXIT_CODE"
