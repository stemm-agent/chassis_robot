#!/usr/bin/env python3
"""Live verification tool: at what distance does vision actually see a low obstacle?

Prints, once per second, how many points of the *low obstacle band*
(z = 0.05 - 0.35 m above the floor) fall into each forward-distance bin measured
from the robot, restricted to +-0.6 m around the centreline so a test box stands
out against the room.

How to use it (vision navigation running, robot standing still):
  1. start the stack:   ros2 launch stemm_vision_mapping vision_nav.launch.py rviz:=true
  2. start this probe:  python3 ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/low_band_probe.py
  3. place a light coloured box (~15 cm tall) at 0.5 m, then 0.8, 1.0, 1.2, 1.5, 2.0 m
  4. the bin that jumps is the distance vision sees it; bins that never move mean
     the box is inside the depth camera's near blind zone (or below the height gate)

Falsifiable prediction (measured 2026-09-21): nothing appears below ~1.1-1.2 m,
and nothing at all below ~0.9 m.  If the 0.50-0.75 or 0.75-1.00 m bin jumps, that
prediction is wrong and the near blind zone is smaller than measured.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       HistoryPolicy)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import tf2_ros

TOPIC = '/vision/low_cloud'
ROBOT_FRAME = 'base_footprint'
LATERAL_LIMIT = 0.6
BINS = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 6.0]


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = (x * x + y * y + z * z + w * w) ** 0.5 or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def parse_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    if not all(k in off for k in ('x', 'y', 'z')):
        return None
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']],
                   'itemsize': msg.point_step})
    a = np.frombuffer(bytes(msg.data), dtype=dt, count=msg.width * msg.height)
    p = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float64)
    return p[np.isfinite(p).all(axis=1)]


class Probe(Node):
    def __init__(self):
        super().__init__('low_band_probe')
        q = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                       durability=QoSDurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PointCloud2, TOPIC, self.on_cloud, q)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.msg = None
        self.count = 0
        self.t0 = self.get_clock().now()
        self.create_timer(1.0, self.report)
        self.get_logger().info('low_band_probe: %s -> forward-distance bins (m), '
                               '|lateral| <= %.1f m' % (TOPIC, LATERAL_LIMIT))

    def on_cloud(self, msg):
        self.msg = msg
        self.count += 1

    def report(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if self.msg is None:
            print('t=%6.1fs  no message on %s yet (is the vision depth layer on?)' % (t, TOPIC))
            return
        p = parse_xyz(self.msg)
        if p is None or p.shape[0] == 0:
            print('t=%6.1fs  cloud has no points' % t)
            return
        try:
            tr = self.tf_buffer.lookup_transform(ROBOT_FRAME, self.msg.header.frame_id, Time())
        except Exception as exc:
            print('t=%6.1fs  %d pts, no TF %s<-%s (%s)' % (
                t, p.shape[0], ROBOT_FRAME, self.msg.header.frame_id, exc))
            return
        q = tr.transform.rotation
        R = quat_matrix(q)
        tt = tr.transform.translation
        b = p @ R.T + np.array([tt.x, tt.y, tt.z])
        sel = (np.abs(b[:, 1]) <= LATERAL_LIMIT) & (b[:, 0] > 0.0)
        fwd = b[sel, 0]
        hist, _ = np.histogram(fwd, bins=BINS)
        cells = ' '.join('%s:%-5d' % ('%.2f-%.2f' % (BINS[i], BINS[i + 1]), hist[i])
                         for i in range(len(BINS) - 1))
        nearest = ('%.2f m' % fwd.min()) if fwd.size else 'none'
        print('t=%6.1fs  low-band pts total=%-6d in front=%-6d nearest=%s' % (
            t, p.shape[0], int(sel.sum()), nearest))
        print('            %s' % cells)


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
