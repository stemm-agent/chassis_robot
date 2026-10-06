#!/usr/bin/env bash
set -euo pipefail

APP_ROOT=${APP_ROOT:-/opt/demo-control}
APP_HOST=${APP_HOST:-0.0.0.0}
APP_PORT=${APP_PORT:-3000}
JETSON_AGENT_URL=${JETSON_AGENT_URL:-http://127.0.0.1:8001}
MAP_SERVICE_URL=${MAP_SERVICE_URL:-http://127.0.0.1:8092}

cd "$APP_ROOT"

if ! command -v node >/dev/null 2>&1; then
  echo "[ERROR] node not found. Please install Node.js first."
  exit 1
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "[ERROR] npm not found. Please install npm first."
  exit 1
fi

if [ ! -d "node_modules" ]; then
  echo "[INFO] Installing Node.js dependencies..."
  npm install
fi

export HOST="$APP_HOST"
export PORT="$APP_PORT"
export JETSON_AGENT_URL
export MAP_SERVICE_URL

echo "[INFO] Starting demo-control on ${APP_HOST}:${APP_PORT}"
echo "[INFO] Forwarding manual control to ${JETSON_AGENT_URL}"
echo "[INFO] Forwarding map APIs to ${MAP_SERVICE_URL}"
exec npm start
