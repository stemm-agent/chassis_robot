#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-$HOME/wheeltec_ros2/install/setup.bash}
VENV_PATH=${VENV_PATH:-$APP_ROOT/.venv}

export MQTT_BROKER_HOST=${MQTT_BROKER_HOST:-127.0.0.1}
export MQTT_BROKER_PORT=${MQTT_BROKER_PORT:-1883}
export MQTT_DEVICE_ID=${MQTT_DEVICE_ID:-robot-001}
export MQTT_AUTO_START_RTMP=${MQTT_AUTO_START_RTMP:-true}
export MQTT_HEARTBEAT_INTERVAL=${MQTT_HEARTBEAT_INTERVAL:-5}
export MQTT_LAUNCH_PACKAGE=${MQTT_LAUNCH_PACKAGE:-wheeltec_nav2}
export MQTT_LAUNCH_FILE=${MQTT_LAUNCH_FILE:-wheeltec_nav2.launch.py}
export MQTT_NODE_PACKAGE=${MQTT_NODE_PACKAGE:-auto_recharge_ros2}
export MQTT_NODE_EXECUTABLE=${MQTT_NODE_EXECUTABLE:-navigate_to_charger}

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

if [ -f "$WORKSPACE_SETUP" ]; then
  set +u
  . "$WORKSPACE_SETUP"
  set -u
else
  echo "[WARN] Workspace setup not found: $WORKSPACE_SETUP"
fi

if [ -f "$VENV_PATH/bin/activate" ]; then
  . "$VENV_PATH/bin/activate"
fi

PYTHON_BIN=$(command -v python3)

if ! "$PYTHON_BIN" -c "import paho.mqtt.client" >/dev/null 2>&1; then
  echo "[ERROR] Missing Python dependencies. Install jetson-agent/requirements.txt first."
  exit 1
fi

if ! "$PYTHON_BIN" -c "import rclpy" >/dev/null 2>&1; then
  if [ -x /usr/bin/python3 ] && /usr/bin/python3 -c "import paho.mqtt.client, rclpy" >/dev/null 2>&1; then
    PYTHON_BIN=/usr/bin/python3
    echo "[INFO] Active virtualenv cannot import ROS2 packages, switching to system Python."
  else
    echo "[ERROR] Python cannot import ROS2 packages (rclpy). Check ROS installation or recreate the venv with system site packages."
    exit 1
  fi
fi

echo "[INFO] Starting launch MQTT bridge for device ${MQTT_DEVICE_ID}"
exec "$PYTHON_BIN" launch_mqtt_bridge.py