#!/usr/bin/env python3
"""Video stream gate (installed as /usr/local/bin/video_stream_gate.py).

Difference from the previous revision: a "paused" state marker is written to
/run/stemm-video-gate so that a hard kill of this node cannot leave the video
pipeline frozen forever.

While body-follow is active (/mode == 2) the video pipeline processes are
paused with SIGSTOP so that follow/navigation gets the CPU.  When follow is
released they are resumed with SIGCONT.

SIGSTOP/SIGCONT is used instead of stopping services because:
  1) launch-mqtt-bridge.service also carries the motion command channel;
  2) SIGSTOP frees CPU immediately and needs no reconnect on resume;
  3) the process objects and MQTT session survive the pause.

The set of paused PIDs lives in memory, so a hard kill (SIGKILL / OOM) of this
node used to leave the pipeline frozen forever.  To make the state recoverable
the node now records "paused" in /run/stemm-video-gate/paused whenever anything
is frozen, and removes the marker as soon as everything is resumed.  A separate
reconciler (video_gate_reconciler.sh) uses that marker to detect an orphaned
pause and release it.  The marker is purely additive bookkeeping: it never
changes which processes are frozen.
"""
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Int8

FOLLOW_MODE = 2

# 2026-09-24: /stemm_cartographer/motion_blocked is only trusted for this long; a
# dead publisher must never leave the video pipeline frozen.
MAPPING_GATE_STALE_SEC = float(
    os.environ.get("STEMM_VIDEO_GATE_MAPPING_STALE_SEC", "10") or 10.0)

# Processes that belong to the local/cloud video pipeline (matched on cmdline).
VIDEO_PATTERNS = (
    "rtmp_image_pusher",
    "local_hls_image_server",
    "/live/robot001.mjpg",   # local HLS side ffmpeg transcoder
)
FFMPEG_HINT = "ffmpeg"

STATE_DIR = os.environ.get("STEMM_VIDEO_GATE_STATE_DIR", "/run/stemm-video-gate")
PAUSED_MARKER = os.path.join(STATE_DIR, "paused")

# Pausing the *cloud* RTMP chain costs more than it saves: the RTMP publisher
# socket to the media server goes silent, the server drops the publisher with
# "Broken pipe", the ffmpeg is respawned and the cloud HLS playlist needs a full
# GOP window (3 x 3.75 s) to be viewable again.  Measured on 2026-09-14: a 25 s
# freeze produced a ~28 s cloud outage.  CPU is still reclaimed by pausing the
# local preview encoder, so by default only that chain is paused.  Set
# STEMM_VIDEO_GATE_PAUSE_CLOUD=1 to restore the previous pause-everything policy.
PAUSE_CLOUD = os.environ.get("STEMM_VIDEO_GATE_PAUSE_CLOUD", "0").strip().lower() in {
    "1", "true", "yes", "on"
}


class VideoGate(Node):
    def __init__(self):
        super().__init__("video_stream_gate")
        self.frozen = {}          # pid -> cmdline
        self.last_mode = None
        self.started = time.monotonic()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Int8, "/mode", self._on_mode, qos)
        # 2026-09-24: a Cartographer mapping session also owns the vehicle, and its
        # nav2 bringup is CPU-sensitive (two sessions failed with a lost
        # change_state response under load).  Freeze the pipeline then too.
        self.mapping_blocked = False
        self.mapping_blocked_at = 0.0
        self.pause_cloud = PAUSE_CLOUD
        mapping_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, "/stemm_cartographer/motion_blocked",
                                 self._on_mapping_blocked, mapping_qos)
        self.create_timer(5.0, self._sweep)
        self._ensure_state_dir()
        # A marker left behind by a previous instance means that instance died
        # while the pipeline was frozen.  Reconcile before doing anything else.
        self._reconcile_orphan_marker()
        self.get_logger().info(
            "video_stream_gate ready: pause video push while follow is active or a "
            "mapping session owns the vehicle (pause_cloud=%s, state_dir=%s)"
            % (PAUSE_CLOUD, STATE_DIR))

    # ---------- state marker ----------
    def _ensure_state_dir(self):
        try:
            os.makedirs(STATE_DIR, exist_ok=True)
        except OSError as exc:
            self.get_logger().warning("cannot create state dir %s: %s" % (STATE_DIR, exc))

    def _write_marker(self):
        try:
            self._ensure_state_dir()
            with open(PAUSED_MARKER, "w") as fh:
                fh.write("%d\n" % os.getpid())
        except OSError as exc:
            self.get_logger().warning("cannot write paused marker: %s" % exc)

    def _clear_marker(self):
        try:
            if os.path.exists(PAUSED_MARKER):
                os.unlink(PAUSED_MARKER)
        except OSError as exc:
            self.get_logger().warning("cannot clear paused marker: %s" % exc)

    def _reconcile_orphan_marker(self):
        """Release a pause inherited from a previous, dead instance of this node."""
        try:
            with open(PAUSED_MARKER) as fh:
                owner = int((fh.read().strip() or "0"))
        except (OSError, ValueError):
            return
        if owner == os.getpid():
            return
        alive = False
        if owner > 0:
            try:
                os.kill(owner, 0)
                alive = True
            except OSError:
                alive = False
        if alive:
            # Another live gate owns the marker; leave it alone.
            return
        self.get_logger().warning(
            "found orphaned pause marker from PID %d; releasing stale SIGSTOP" % owner)
        self._release_stopped_pipeline()
        self._clear_marker()

    def _release_stopped_pipeline(self):
        n = 0
        for pid, cmd in self._candidates():
            if self._state(pid) in ("T", "t"):
                try:
                    os.kill(pid, signal.SIGCONT)
                    n += 1
                    self.get_logger().info("released stale stop on PID %d" % pid)
                except OSError as exc:
                    self.get_logger().warning("cannot resume PID %d: %s" % (pid, exc))
        if n:
            self.get_logger().info("released %d stale-stopped video process(es)" % n)
        return n

    # ---------- process discovery ----------
    def _is_cloud_process(self, cmd):
        """True for the RTMP/cloud push chain (pusher node or its ffmpeg)."""
        if "rtmp_image_pusher" in cmd:
            return True
        if FFMPEG_HINT in cmd and "flv" in cmd and "rtmp://" in cmd:
            return True
        return False

    def _candidates(self):
        out = []
        rtmp_pusher_pids = set()
        for pid in self._all_pids():
            cmd = self._cmdline(pid)
            if not cmd:
                continue
            if "rtmp_image_pusher" in cmd:
                rtmp_pusher_pids.add(pid)
            if any(p in cmd for p in VIDEO_PATTERNS):
                if not self.pause_cloud and self._is_cloud_process(cmd):
                    continue
                out.append((pid, cmd))
            elif FFMPEG_HINT in cmd and "/home/hnh" in cmd:
                # only claim cloud_video_pusher related ffmpeg
                if "libx264" in cmd or "mjpeg" in cmd or "robot001" in cmd:
                    if not self.pause_cloud and self._is_cloud_process(cmd):
                        continue
                    out.append((pid, cmd))
        # the rtmp-side ffmpeg is a child of the pusher, claim it by parent
        for pid in self._all_pids():
            ppid = self._ppid(pid)
            if ppid in rtmp_pusher_pids and "ffmpeg" in self._cmdline(pid):
                if not self.pause_cloud:
                    continue
                out.append((pid, self._cmdline(pid)))
        return out

    @staticmethod
    def _all_pids():
        for name in os.listdir("/proc"):
            if name.isdigit():
                yield int(name)

    @staticmethod
    def _cmdline(pid):
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as fh:
                return fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
        except Exception:
            return ""

    @staticmethod
    def _ppid(pid):
        try:
            with open("/proc/%d/stat" % pid) as fh:
                return int(fh.read().split(") ", 1)[1].split()[1])
        except Exception:
            return -1

    @staticmethod
    def _state(pid):
        try:
            with open("/proc/%d/stat" % pid) as fh:
                return fh.read().split(") ", 1)[1].split()[0]
        except Exception:
            return "?"

    # ---------- actions ----------
    def _freeze(self):
        added = 0
        for pid, cmd in self._candidates():
            if pid == os.getpid() or pid in self.frozen:
                continue
            if self._state(pid) in ("T", "t"):
                self.frozen[pid] = cmd
                continue
            try:
                os.kill(pid, signal.SIGSTOP)
                self.frozen[pid] = cmd
                added += 1
                self.get_logger().info("paused PID %d: %s" % (pid, cmd[:80]))
            except Exception as exc:
                self.get_logger().warning("cannot pause PID %d: %s" % (pid, exc))
        if added:
            self._write_marker()
            self.get_logger().info("video pipeline paused, %d process(es)" % len(self.frozen))

    def _resume(self):
        n = 0
        for pid in list(self.frozen):
            try:
                if os.path.exists("/proc/%d" % pid):
                    os.kill(pid, signal.SIGCONT)
                    n += 1
            except Exception as exc:
                self.get_logger().warning("cannot resume PID %d: %s" % (pid, exc))
            self.frozen.pop(pid, None)
        if n:
            self.get_logger().info("video pipeline resumed, %d process(es)" % n)
        self._clear_marker()

    def _sweep(self):
        """Freeze video while follow or a mapping session owns the car."""
        if self._pause_required():
            self._freeze()
        else:
            self.frozen = {p: c for p, c in self.frozen.items()
                           if os.path.exists("/proc/%d" % p)}
            self._clear_marker()
        if self.frozen:
            self._write_marker()

    def _mapping_gate_active(self):
        """True while a mapping session owns the vehicle (stale flag = cleared)."""
        if not self.mapping_blocked:
            return False
        if time.monotonic() - self.mapping_blocked_at > MAPPING_GATE_STALE_SEC:
            return False
        return True

    def _pause_required(self):
        return self.last_mode == FOLLOW_MODE or self._mapping_gate_active()

    def _apply_pause_state(self, reason):
        """Freeze or resume according to both inputs, including the cloud chain."""
        wanted_cloud = PAUSE_CLOUD or self._mapping_gate_active()
        if wanted_cloud != self.pause_cloud:
            self.pause_cloud = wanted_cloud
            self.get_logger().info(
                "video gate cloud policy -> pause_cloud=%s" % self.pause_cloud)
        if self._pause_required():
            time.sleep(2.0)
            self._freeze()
            self._freeze()      # second pass catches processes started in the window
            self.get_logger().info(
                "video pipeline held paused (%s): %d process(es)"
                % (reason, len(self.frozen)))
        else:
            self._resume()
            self.get_logger().info("video pipeline released (%s)" % reason)

    def _on_mapping_blocked(self, msg):
        blocked = bool(msg.data)
        self.mapping_blocked_at = time.monotonic()
        if blocked == self.mapping_blocked:
            return
        self.mapping_blocked = blocked
        self.get_logger().info(
            "mapping gate %s: re-evaluating video pause"
            % ("active" if blocked else "cleared"))
        self._apply_pause_state("mapping_blocked=%s" % blocked)

    def _on_mode(self, msg):
        mode = int(msg.data)
        if mode == self.last_mode:
            return
        # transient_local /mode replays its last sample at start-up; freezing
        # again in that case is harmless because _freeze is idempotent.
        prev, self.last_mode = self.last_mode, mode
        if mode == FOLLOW_MODE:
            # give the MQTT bridge self-healing window a head start
            time.sleep(2.0)
            self.get_logger().info("follow active (mode=%d): pausing video push" % mode)
            self._freeze()
            self._freeze()   # second pass catches processes started in the window
        elif prev == FOLLOW_MODE:
            self.get_logger().info("follow released (mode=%d): resuming video push" % mode)
            self._resume()

    def shutdown(self):
        self._resume()


def main():
    rclpy.init()
    node = VideoGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # rclpy may already have shut the context down during a signal exit
            pass


if __name__ == "__main__":
    main()
