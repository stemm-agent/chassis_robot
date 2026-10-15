#!/usr/bin/env bash
# STEMM video stream gate - reconciler.
#
# Releasing a pause is normally done by video_stream_gate.py itself.  If that
# process is killed hard (SIGKILL, OOM kill, systemd unit killed) its in-memory
# list of paused PIDs is lost while the pipeline stays SIGSTOPped, which freezes
# both the local preview and the cloud push until a manual restart.
#
# This script closes that hole.  It ONLY sends SIGCONT to already-stopped
# processes that belong to the video pipeline; it never starts, stops or signals
# anything else, and it never touches motion/chassis/navigation nodes.
#
# Usage:
#   video_gate_reconciler.sh            # act only when the pause is orphaned
#   video_gate_reconciler.sh --boot     # same check, run at gate start-up
#   video_gate_reconciler.sh --force    # resume all stopped video processes
#   video_gate_reconciler.sh --status   # report only, change nothing
set -u

STATE_DIR="${STEMM_VIDEO_GATE_STATE_DIR:-/run/stemm-video-gate}"
MARKER="$STATE_DIR/paused"
MODE="${1:-}"

log() { echo "[video-gate-reconciler] $*"; }

# Video pipeline identification.  Must stay in sync with video_stream_gate.py.
is_video_cmd() {
  case "$1" in
    *rtmp_image_pusher*) return 0 ;;
    *local_hls_image_server*) return 0 ;;
    *"/live/robot001.mjpg"*) return 0 ;;
    *ffmpeg*libx264*) return 0 ;;
    *ffmpeg*robot001*) return 0 ;;
  esac
  return 1
}

stopped_video_pids() {
  local p cmd
  for p in /proc/[0-9]*; do
    [ -r "$p/stat" ] || continue
    pid=${p#/proc/}
    state=$(awk '{print $3}' "$p/stat" 2>/dev/null) || continue
    [ "$state" = "T" ] || continue
    cmd=$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null) || continue
    [ -n "$cmd" ] || continue
    if is_video_cmd "$cmd"; then
      echo "$pid"
    fi
  done
}

marker_owner() {
  [ -f "$MARKER" ] || return 1
  head -n1 "$MARKER" 2>/dev/null | tr -dc '0-9'
}

owner_alive() {
  local owner="$1"
  [ -n "$owner" ] || return 1
  [ -d "/proc/$owner" ] || return 1
  # the recorded owner must actually still be the gate node
  tr '\0' ' ' < "/proc/$owner/cmdline" 2>/dev/null | grep -q 'video_stream_gate.py'
}

release() {
  local reason="$1" n=0 pid
  for pid in $(stopped_video_pids); do
    if kill -CONT "$pid" 2>/dev/null; then
      log "$reason: resumed PID $pid"
      n=$((n + 1))
    fi
  done
  if [ "$n" -gt 0 ]; then
    rm -f "$MARKER" 2>/dev/null || true
    log "$reason: released $n video process(es), marker cleared"
  fi
  return 0
}

case "$MODE" in
  --status)
    owner=$(marker_owner) || owner=""
    log "marker=${MARKER} owner='${owner}' present=$([ -f "$MARKER" ] && echo yes || echo no)"
    if owner_alive "$owner"; then log "owner gate PID $owner is alive"; else log "owner gate PID '${owner}' is NOT alive"; fi
    log "stopped video pids: $(stopped_video_pids | tr '\n' ' ')"
    exit 0
    ;;
  --force)
    release "forced"
    exit 0
    ;;
  --boot)
    owner=$(marker_owner) || owner=""
    if [ -z "$owner" ]; then
      # no marker: if something is nevertheless stopped, recover it.
      release "boot/no-marker"
    elif ! owner_alive "$owner"; then
      release "boot/stale-owner-$owner"
    else
      log "boot: pause owned by live gate PID $owner, leaving pipeline paused"
    fi
    exit 0
    ;;
  *)
    owner=$(marker_owner) || owner=""
    if [ -z "$owner" ]; then
      # No marker means no live gate legitimately owns a pause: the gate either
      # never paused anything, or it died before it could clear its marker.
      # Any video process still in T state is therefore orphaned.  The window in
      # which the gate has SIGSTOPped but not yet written its marker is a few
      # milliseconds wide; releasing a pause early only costs a little CPU.
      if [ -n "$(stopped_video_pids)" ]; then
        release "no-marker/orphan"
      fi
      exit 0
    fi
    if owner_alive "$owner"; then
      exit 0
    fi
    release "stale-owner-$owner"
    exit 0
    ;;
esac
