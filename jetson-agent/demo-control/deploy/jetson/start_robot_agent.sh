#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
VENV_PATH=${VENV_PATH:-$APP_ROOT/.venv}
ROBOT_AGENT_PORT=${ROBOT_AGENT_PORT:-8001}

export ROBOT_DEVICE_ID=${ROBOT_DEVICE_ID:-robot-001}
export CMD_VEL_TOPIC=${CMD_VEL_TOPIC:-/cmd_vel}
export MANUAL_LINEAR_SPEED=${MANUAL_LINEAR_SPEED:-0.2}
export MANUAL_ANGULAR_SPEED=${MANUAL_ANGULAR_SPEED:-0.5}
export AUTO_STOP_TIMEOUT=${AUTO_STOP_TIMEOUT:-5.0}

cd "$APP_ROOT/jetson-agent"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found."
  exit 1
fi

if [ -f "$ROS_SETUP" ]; then
  # ROS setup scripts may read unset variables, so temporarily relax nounset.
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

echo "[INFO] Starting robot agent on 0.0.0.0:${ROBOT_AGENT_PORT}"
exec "$PYTHON_BIN" -m uvicorn robot_agent:app --host 0.0.0.0 --port "$ROBOT_AGENT_PORT"
