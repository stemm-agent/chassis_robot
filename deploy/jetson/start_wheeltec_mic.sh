#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$HOME/wheeltec_ros2/install/setup.bash}
MIC_INIT_DELAY=${MIC_INIT_DELAY:-3}
ENABLE_VOICE_SUMMON_COORDINATOR=${ENABLE_VOICE_SUMMON_COORDINATOR:-1}
VOICE_SUMMON_COORDINATOR_EXECUTABLE=${VOICE_SUMMON_COORDINATOR_EXECUTABLE:-voice_summon_coordinator_bearing_nav}
ENABLE_VOICE_SUMMON_BEARING_NAV_PROGRESS_WATCHDOG=${ENABLE_VOICE_SUMMON_BEARING_NAV_PROGRESS_WATCHDOG:-1}
ENABLE_VOICE_SUMMON_BEARING_NAV_YOLO_PREEMPT=${ENABLE_VOICE_SUMMON_BEARING_NAV_YOLO_PREEMPT:-1}
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

  if [ -n "${VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID:-}" ] && kill -0 "$VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID" >/dev/null 2>&1; then
    kill "$VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID" >/dev/null 2>&1 || true
  fi

  if [ -n "${VOICE_BEARING_NAV_YOLO_PREEMPT_PID:-}" ] && kill -0 "$VOICE_BEARING_NAV_YOLO_PREEMPT_PID" >/dev/null 2>&1; then
    kill "$VOICE_BEARING_NAV_YOLO_PREEMPT_PID" >/dev/null 2>&1 || true
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

  wait ${VOICE_BEARING_SEARCH_SHADOW_PID:-} ${VOICE_SHADOW_PID:-} ${VOICE_BEARING_NAV_YOLO_PREEMPT_PID:-} ${VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID:-} ${VOICE_SUMMON_PID:-} ${BASE_PID:-} ${MIC_INIT_PID:-} 2>/dev/null || true
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
  echo "[INFO] Starting ros2 run wheeltec_mic_ros2 ${VOICE_SUMMON_COORDINATOR_EXECUTABLE}"
  ros2 run wheeltec_mic_ros2 "$VOICE_SUMMON_COORDINATOR_EXECUTABLE" &
  VOICE_SUMMON_PID=$!
else
  echo "[INFO] Voice summon coordinator disabled by ENABLE_VOICE_SUMMON_COORDINATOR=$ENABLE_VOICE_SUMMON_COORDINATOR"
fi

VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID=
if [ "$ENABLE_VOICE_SUMMON_BEARING_NAV_PROGRESS_WATCHDOG" = "1" ]; then
  echo "[INFO] Starting bearing-nav translation progress watchdog"
  ros2 run wheeltec_mic_ros2 voice_summon_bearing_nav_progress_watchdog &
  VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID=$!
else
  echo "[INFO] Bearing-nav progress watchdog disabled by ENABLE_VOICE_SUMMON_BEARING_NAV_PROGRESS_WATCHDOG=$ENABLE_VOICE_SUMMON_BEARING_NAV_PROGRESS_WATCHDOG"
fi

VOICE_BEARING_NAV_YOLO_PREEMPT_PID=
if [ "$ENABLE_VOICE_SUMMON_BEARING_NAV_YOLO_PREEMPT" = "1" ]; then
  echo "[INFO] Starting bearing-nav RGB person preempt watcher"
  ros2 run wheeltec_mic_ros2 voice_summon_bearing_nav_yolo_preempt &
  VOICE_BEARING_NAV_YOLO_PREEMPT_PID=$!
else
  echo "[INFO] Bearing-nav RGB person preempt watcher disabled by ENABLE_VOICE_SUMMON_BEARING_NAV_YOLO_PREEMPT=$ENABLE_VOICE_SUMMON_BEARING_NAV_YOLO_PREEMPT"
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
if [ -n "${VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID:-}" ]; then
  WAIT_PIDS+=("$VOICE_BEARING_NAV_PROGRESS_WATCHDOG_PID")
fi
if [ -n "${VOICE_BEARING_NAV_YOLO_PREEMPT_PID:-}" ]; then
  WAIT_PIDS+=("$VOICE_BEARING_NAV_YOLO_PREEMPT_PID")
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
