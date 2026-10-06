#!/usr/bin/env python3
"""Publish a synthetic low obstacle as a point cloud, to test the chain
"points -> vision layer -> local costmap" without needing the camera or a box.

    python3 scripts/fake_low_obstacle.py --z 0.15 --x 1.0 --y -0.4 --seconds 8

The patch is a 5x5 grid (4 cm spacing, so ~0.16 x 0.16 m) published in the map
frame on /vision/live_cloud.  Check the effect with:

    python3 scripts/costmap_window_probe.py 1.00,-0.40,0.30

Expected (measured 2026-09-21 on this robot): a 0.15 m high patch adds ~16 lethal
cells right there, while a 0.03 m patch (below the 0.05 m height gate of the
vision layer) adds nothing.

NOTE: the marks stay in the local costmap until something clears them, and the
camera cannot clear them because they sit inside its near blind zone.  Clear them
explicitly afterwards:

    ros2 service call /local_costmap/clear_entirely_local_costmap \
        nav2_msgs/srv/ClearEntireCostmap "{}"
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       HistoryPolicy)
from sensor_msgs.msg import PointCloud2, PointField


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--z', type=float, default=0.15, help='height above the floor [m]')
    ap.add_argument('--x', type=float, default=1.0, help='forward distance in the map frame [m]')
    ap.add_argument('--y', type=float, default=0.0, help='lateral offset in the map frame [m]')
    ap.add_argument('--seconds', type=float, default=8.0)
    ap.add_argument('--topic', default='/vision/live_cloud')
    a = ap.parse_args()

    n_side, step = 5, 0.04
    pts = np.array([[a.x + (i - n_side // 2) * step, a.y + (j - n_side // 2) * step, a.z]
                    for i in range(n_side) for j in range(n_side)], dtype=np.float32)

    rclpy.init()
    node = Node('fake_low_obstacle')
    q = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                   durability=QoSDurabilityPolicy.VOLATILE,
                   history=HistoryPolicy.KEEP_LAST)
    pub = node.create_publisher(PointCloud2, a.topic, q)
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()

    t_end = time.time() + a.seconds
    k = 0
    while time.time() < t_end and rclpy.ok():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        k += 1
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.15)
    print('published %d clouds of %d points at map (%.2f, %.2f, z=%.3f) on %s'
          % (k, pts.shape[0], a.x, a.y, a.z, a.topic))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
