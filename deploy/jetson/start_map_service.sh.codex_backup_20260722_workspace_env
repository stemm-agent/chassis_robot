#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
VENV_PATH=${VENV_PATH:-$APP_ROOT/.venv}
MAP_SERVICE_HOST=${MAP_SERVICE_HOST:-127.0.0.1}
MAP_SERVICE_PORT=${MAP_SERVICE_PORT:-8092}

export ROBOT_DEVICE_ID=${ROBOT_DEVICE_ID:-robot-001}
export MAP_FILE_PATH=${MAP_FILE_PATH:-/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm}
export MAP_SERVICE_PATH=${MAP_SERVICE_PATH:-/stemm/save_map}
export MAP_WAIT_TIMEOUT=${MAP_WAIT_TIMEOUT:-30}
export MAP_FILE_WAIT_TIMEOUT=${MAP_FILE_WAIT_TIMEOUT:-5}

cd "$APP_ROOT/jetson-agent"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found."
  exit 1
fi

if [ -f "$ROS_SETUP" ]; then
  set +u
  . "$ROS_SETUP"
  set -u
fi

if [ -f "$VENV_PATH/bin/activate" ]; then
  . "$VENV_PATH/bin/activate"
fi

PYTHON_BIN=$(command -v python3)

if ! "$PYTHON_BIN" -c "import fastapi, uvicorn" >/dev/null 2>&1; then
  echo "[ERROR] Missing Python dependencies. Install jetson-agent/requirements.txt first."
  exit 1
fi

if ! "$PYTHON_BIN" -c "import rclpy" >/dev/null 2>&1; then
  if [ -x /usr/bin/python3 ] && /usr/bin/python3 -c "import fastapi, uvicorn, rclpy" >/dev/null 2>&1; then
    PYTHON_BIN=/usr/bin/python3
    echo "[INFO] Active virtualenv cannot import ROS2 packages, switching to system Python."
  else
    echo "[ERROR] Python cannot import ROS2 packages (rclpy). Check ROS installation or recreate the venv with system site packages."
    exit 1
  fi
fi

echo "[INFO] Starting map service on ${MAP_SERVICE_HOST}:${MAP_SERVICE_PORT}"
exec "$PYTHON_BIN" -m uvicorn map_service:app --host "$MAP_SERVICE_HOST" --port "$MAP_SERVICE_PORT"
