#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$HOME/wheeltec_ros2/install/setup.bash}
MIC_INIT_DELAY=${MIC_INIT_DELAY:-3}
ENABLE_VOICE_SUMMON_COORDINATOR=${ENABLE_VOICE_SUMMON_COORDINATOR:-1}
ENABLE_VOICE_SUMMON_SHADOW_LOGGER=${ENABLE_VOICE_SUMMON_SHADOW_LOGGER:-1}
ENABLE_VOICE_SUMMON_BEARING_SEARCH_SHADOW=${ENABLE_VOICE_SUMMON_BEARING_SEARCH_SHADOW:-1}

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

  if [ -n "${VOICE_SUMMON_PID:-}" ] && kill -0 "$VOICE_SUMMON_PID" >/dev/null 2>&1; then
    kill "$VOICE_SUMMON_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${VOICE_SHADOW_PID:-}" ] && kill -0 "$VOICE_SHADOW_PID" >/dev/null 2>&1; then
    kill "$VOICE_SHADOW_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${VOICE_BEARING_SEARCH_SHADOW_PID:-}" ] && kill -0 "$VOICE_BEARING_SEARCH_SHADOW_PID" >/dev/null 2>&1; then
    kill "$VOICE_BEARING_SEARCH_SHADOW_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${BASE_PID:-}" ] && kill -0 "$BASE_PID" >/dev/null 2>&1; then
    kill "$BASE_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${MIC_INIT_PID:-}" ] && kill -0 "$MIC_INIT_PID" >/dev/null 2>&1; then
    kill "$MIC_INIT_PID" >/dev/null 2>&1 || true
  fi

  wait ${VOICE_BEARING_SEARCH_SHADOW_PID:-} ${VOICE_SHADOW_PID:-} ${VOICE_SUMMON_PID:-} ${BASE_PID:-} ${MIC_INIT_PID:-} 2>/dev/null || true
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

VOICE_SUMMON_PID=
if [ "$ENABLE_VOICE_SUMMON_COORDINATOR" = "1" ]; then
  echo "[INFO] Starting ros2 run wheeltec_mic_ros2 voice_summon_coordinator"
  ros2 run wheeltec_mic_ros2 voice_summon_coordinator &
  VOICE_SUMMON_PID=$!
else
  echo "[INFO] Voice summon coordinator disabled by ENABLE_VOICE_SUMMON_COORDINATOR=$ENABLE_VOICE_SUMMON_COORDINATOR"
fi

VOICE_SHADOW_PID=
if [ "$ENABLE_VOICE_SUMMON_SHADOW_LOGGER" = "1" ]; then
  echo "[INFO] Starting read-only voice summon shadow logger"
  ros2 run wheeltec_mic_ros2 voice_summon_shadow_logger &
  VOICE_SHADOW_PID=$!
else
  echo "[INFO] Voice summon shadow logger disabled by ENABLE_VOICE_SUMMON_SHADOW_LOGGER=$ENABLE_VOICE_SUMMON_SHADOW_LOGGER"
fi

VOICE_BEARING_SEARCH_SHADOW_PID=
if [ "$ENABLE_VOICE_SUMMON_BEARING_SEARCH_SHADOW" = "1" ]; then
  echo "[INFO] Starting read-only bearing-only search shadow"
  ros2 run wheeltec_mic_ros2 voice_summon_bearing_search_shadow &
  VOICE_BEARING_SEARCH_SHADOW_PID=$!
else
  echo "[INFO] Bearing-only search shadow disabled by ENABLE_VOICE_SUMMON_BEARING_SEARCH_SHADOW=$ENABLE_VOICE_SUMMON_BEARING_SEARCH_SHADOW"
fi

WAIT_PIDS=("$MIC_INIT_PID" "$BASE_PID")
if [ -n "${VOICE_SUMMON_PID:-}" ]; then
  WAIT_PIDS+=("$VOICE_SUMMON_PID")
fi
if [ -n "${VOICE_SHADOW_PID:-}" ]; then
  WAIT_PIDS+=("$VOICE_SHADOW_PID")
fi
if [ -n "${VOICE_BEARING_SEARCH_SHADOW_PID:-}" ]; then
  WAIT_PIDS+=("$VOICE_BEARING_SEARCH_SHADOW_PID")
fi

set +e
wait -n "${WAIT_PIDS[@]}"
EXIT_CODE=$?
set -e

echo "[WARN] One wheeltec_mic process exited with code ${EXIT_CODE}, stopping the rest"
exit "$EXIT_CODE"
