#!/usr/bin/env bash
set -euo pipefail

WIFI_IFACE=${WIFI_IFACE:-wlp4s0}
COOLDOWN_SECONDS=${COOLDOWN_SECONDS:-60}
WAIT_ATTEMPTS=${WAIT_ATTEMPTS:-15}
WAIT_INTERVAL=${WAIT_INTERVAL:-2}
ROBOT_READY_DELAY=${ROBOT_READY_DELAY:-8}
LOCK_FILE=${LOCK_FILE:-/run/demo-control-wifi-refresh.lock}
STAMP_FILE=${STAMP_FILE:-/run/demo-control-wifi-refresh.last}

SERVICES=(
  wheeltec-robot
  bodyfollow-nav2
  wheeltec-recharge-manager
  robot-agent
  stemm-cartographer-map-service
  launch-mqtt-bridge
  device-reporter
  wheeltec-mic
  low-battery-auto-recharge
  demo-control
)

log() {
  echo "[demo-control-wifi-refresh] $*"
}

get_ipv4() {
  ip -4 addr show dev "$WIFI_IFACE" scope global 2>/dev/null \
    | awk '/inet / {print $2}' \
    | cut -d/ -f1 \
    | head -n 1
}

get_wifi_type() {
  if command -v iw >/dev/null 2>&1; then
    iw dev "$WIFI_IFACE" info 2>/dev/null | awk '/type/ {print $2; exit}'
  fi
}

is_wifi_client_ready() {
  local wifi_type
  local ipv4

  wifi_type=$(get_wifi_type || true)
  ipv4=$(get_ipv4 || true)

  if [ -z "$ipv4" ]; then
    log "$WIFI_IFACE has no IPv4 address yet."
    return 1
  fi

  if [ "$wifi_type" = "AP" ]; then
    log "$WIFI_IFACE is still AP mode, skip refresh."
    return 1
  fi

  log "$WIFI_IFACE is ready, type=${wifi_type:-unknown}, ipv4=$ipv4."
  return 0
}

mkdir -p "$(dirname "$LOCK_FILE")"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  log "another refresh is running, skip."
  exit 0
fi

now=$(date +%s)
if [ -f "$STAMP_FILE" ]; then
  last=$(cat "$STAMP_FILE" 2>/dev/null || echo 0)
  if [ $((now - last)) -lt "$COOLDOWN_SECONDS" ]; then
    log "refreshed recently, skip."
    exit 0
  fi
fi

ready=false
for attempt in $(seq 1 "$WAIT_ATTEMPTS"); do
  if is_wifi_client_ready; then
    ready=true
    break
  fi
  log "waiting for Wi-Fi client mode... attempt ${attempt}/${WAIT_ATTEMPTS}"
  sleep "$WAIT_INTERVAL"
done

if [ "$ready" != "true" ]; then
  log "Wi-Fi client mode is not ready, do not restart services."
  exit 0
fi

echo "$now" > "$STAMP_FILE"

for service in "${SERVICES[@]}"; do
  log "restarting ${service}.service"
  if ! systemctl restart "${service}.service"; then
    log "WARN: failed to restart ${service}.service"
  fi

  if [ "$service" = "wheeltec-robot" ] && [ "$ROBOT_READY_DELAY" != "0" ]; then
    log "waiting ${ROBOT_READY_DELAY}s for wheeltec-robot ROS nodes to settle."
    sleep "$ROBOT_READY_DELAY"
  fi
done

log "service refresh finished."
