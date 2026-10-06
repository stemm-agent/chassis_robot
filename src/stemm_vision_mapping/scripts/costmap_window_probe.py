#!/usr/bin/env python3
"""One-shot probe of the local costmap: totals + counts inside given windows.

Use it to prove that an obstacle really reached the local costmap, and where:

    python3 scripts/costmap_window_probe.py 1.00,-0.40,0.30 1.00,0.40,0.30

Each argument is  x,y,radius  (map frame, metres).  Output counts lethal cells
(100) and inscribed cells (99) inside each window, plus the whole-costmap totals.
Compare a run with the object present against a run with it removed.
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       HistoryPolicy)
from nav_msgs.msg import OccupancyGrid

TOPIC = '/local_costmap/costmap'


class Probe(Node):
    def __init__(self):
        super().__init__('costmap_window_probe')
        q = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, TOPIC, self.cb, q)
        self.msg = None

    def cb(self, m):
        if self.msg is None:
            self.msg = m


def main():
    rclpy.init()
    node = Probe()
    t0 = node.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok() and node.msg is None:
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.get_clock().now().nanoseconds * 1e-9 - t0 > 12:
            break
    if node.msg is None:
        print('no message on %s within 12 s (is the navigation stack with the '
              'vision layer running?)' % TOPIC)
        rclpy.shutdown()
        raise SystemExit(1)
    m = node.msg
    rclpy.shutdown()
    d = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    print('costmap %dx%d res=%.3f origin=(%.2f,%.2f) lethal(100)=%d inscribed(99)=%d '
          'unknown(-1)=%d free(0)=%d' % (
              m.info.width, m.info.height, res, ox, oy, int((d == 100).sum()),
              int((d == 99).sum()), int((d == -1).sum()), int((d == 0).sum())))
    for spec in sys.argv[1:]:
        sx, sy, sr = [float(v) for v in spec.split(',')]
        i0 = max(0, int((sx - sr - ox) / res))
        i1 = min(m.info.width, int((sx + sr - ox) / res) + 1)
        j0 = max(0, int((sy - sr - oy) / res))
        j1 = min(m.info.height, int((sy + sr - oy) / res) + 1)
        win = d[j0:j1, i0:i1]
        print('  window (%.2f,%.2f) r=%.2f -> lethal=%d inscribed=%d free=%d cells=%d' % (
            sx, sy, sr, int((win == 100).sum()), int((win == 99).sum()),
            int((win == 0).sum()), win.size))


if __name__ == '__main__':
    main()
