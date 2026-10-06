#!/usr/bin/env python3
"""Find a collision-free start/goal pair on the global costmap of the running stack.

The planner refuses to start inside inflated/lethal space (for example while the
robot is parked in its charging dock, whose structure surrounds the map origin).
This probe reports the nearest free cell to the robot's believed pose and a second
free cell 1.2-1.8 m away, so planning can be verified even from the dock.

Output (one line each):
    ROBOT_POSE=x,y
    FREE_START=x,y
    FREE_GOAL=x,y
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


class Probe(Node):
    def __init__(self):
        super().__init__('free_start_probe')
        q = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST)
        self.gc = None
        self.pose = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.cb, q)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)

    def cb(self, m):
        if self.gc is None:
            self.gc = m

    def pose_cb(self, m):
        if self.pose is None:
            self.pose = m


def main():
    rclpy.init()
    n = Probe()
    t0 = n.get_clock().now().nanoseconds * 1e-9
    while rclpy.ok() and n.gc is None:
        rclpy.spin_once(n, timeout_sec=0.5)
        if n.get_clock().now().nanoseconds * 1e-9 - t0 > 15:
            break
    if n.gc is None:
        print('ERROR=no global costmap'); rclpy.shutdown(); return
    px = py = 0.0
    if n.pose is not None:
        px = n.pose.pose.pose.position.x
        py = n.pose.pose.pose.position.y
    m = n.gc
    rclpy.shutdown()
    d = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    print('ROBOT_POSE=%.2f,%.2f' % (px, py))
    print('ROBOT_CELL_VALUE=%d' % d[min(m.info.height - 1, max(0, int((py - oy) / res))),
                                    min(m.info.width - 1, max(0, int((px - ox) / res)))])
    free = np.argwhere(d == 0)
    if not free.size:
        print('ERROR=no free cell at all'); return
    cx, cy = (px - ox) / res, (py - oy) / res
    dist = (free[:, 0] - cy) ** 2 + (free[:, 1] - cx) ** 2
    order = np.argsort(dist)
    sj, si = int(free[order[0], 0]), int(free[order[0], 1])
    sx, sy = ox + (si + 0.5) * res, oy + (sj + 0.5) * res
    print('FREE_START=%.2f,%.2f' % (sx, sy))
    for k in order[::max(1, order.size // 6000)]:
        j, i = int(free[k, 0]), int(free[k, 1])
        fx, fy = ox + (i + 0.5) * res, oy + (j + 0.5) * res
        r = ((fx - sx) ** 2 + (fy - sy) ** 2) ** 0.5
        if 1.2 < r < 1.8:
            print('FREE_GOAL=%.2f,%.2f' % (fx, fy))
            return
    print('ERROR=no free goal candidate')


if __name__ == '__main__':
    main()
