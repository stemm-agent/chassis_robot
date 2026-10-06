#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

python3 map_viewer_server.py \
  --host "${MAP_VIEWER_HOST:-0.0.0.0}" \
  --port "${MAP_VIEWER_PORT:-8091}" \
  --command-url "${MAP_VIEWER_COMMAND_URL:-http://127.0.0.1:3000/api/command}" \
  --map-file "${MAP_VIEWER_MAP_FILE:-/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm}" \
  --interval "${MAP_VIEWER_INTERVAL:-5}" \
  --device-id "${MAP_VIEWER_DEVICE_ID:-robot-001}"
