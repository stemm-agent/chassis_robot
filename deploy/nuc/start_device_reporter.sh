#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
VENV_PATH=${VENV_PATH:-$APP_ROOT/.venv}

export DEVICE_REPORT_DEVICE_ID=${DEVICE_REPORT_DEVICE_ID:-${MQTT_DEVICE_ID:-robot-001}}
export DEVICE_REPORT_CHECK_INTERVAL=${DEVICE_REPORT_CHECK_INTERVAL:-5}
export DEVICE_REPORT_HEARTBEAT_INTERVAL=${DEVICE_REPORT_HEARTBEAT_INTERVAL:-60}
export DEVICE_REPORT_HTTP_TIMEOUT=${DEVICE_REPORT_HTTP_TIMEOUT:-5}
export DEVICE_REPORT_INCLUDE_WAITING_STATE=${DEVICE_REPORT_INCLUDE_WAITING_STATE:-true}

cd "$APP_ROOT/jetson-agent"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[ERROR] python3 not found."
  exit 1
fi

if [ -f "$VENV_PATH/bin/activate" ]; then
  . "$VENV_PATH/bin/activate"
fi

PYTHON_BIN=$(command -v python3)

echo "[INFO] Starting device reporter for device ${DEVICE_REPORT_DEVICE_ID}"
exec "$PYTHON_BIN" device_reporter.py
