#!/usr/bin/env python3
"""把任一 OccupancyGrid 话题保存成 nav2 标准 pgm/yaml（自研，QoS 固定 transient_local+reliable）。

标准 ROS 2 调用：
  ros2 run stemm_vision_mapping save_grid.py --ros-args -p topic:=/vision/obstacle_grid -p out:=/path/name
"""
import os, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


class Saver(Node):
    def __init__(self):
        super().__init__('save_grid')
        self.declare_parameter('topic', '/vision/obstacle_grid')
        self.declare_parameter('out', '/tmp/grid')
        self.declare_parameter('timeout', 20.0)
        self.topic = self.get_parameter('topic').value
        self.out = os.path.expanduser(self.get_parameter('out').value)
        self.timeout = float(self.get_parameter('timeout').value)
        q = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.topic, self.cb, q)
        self.create_timer(self.timeout, self.timeout_cb)
        self.get_logger().info(f'等待 {self.topic} 的栅格（最多 {self.timeout}s）→ {self.out}.pgm')

    def timeout_cb(self):
        self.get_logger().error(f'超时：未收到 {self.topic}（发布者是否在运行？）')
        rclpy.shutdown()

    def cb(self, m):
        w, h = m.info.width, m.info.height
        d = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)
        img[d >= 65] = 0
        img[(d >= 0) & (d < 65)] = 254
        img = np.flipud(img)
        os.makedirs(os.path.dirname(self.out) or '.', exist_ok=True)
        with open(self.out + '.pgm', 'wb') as f:
            f.write(b'P5\n%d %d\n255\n' % (w, h)); f.write(img.tobytes())
        o = m.info.origin.position
        with open(self.out + '.yaml', 'w', encoding='utf-8') as f:
            f.write(f'image: {os.path.basename(self.out)}.pgm\nmode: trinary\n'
                    f'resolution: {m.info.resolution}\n'
                    f'origin: [{o.x:.6f}, {o.y:.6f}, 0.0]\nnegate: 0\n'
                    f'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
        occ = int((d >= 65).sum()); free = int(((d >= 0) & (d < 65)).sum())
        self.get_logger().info(f'已保存 {w}x{h} 占据={occ} 空闲={free} → {self.out}.pgm/.yaml')
        rclpy.shutdown()


def main():
    rclpy.init()
    n = Saver()
    try:
        rclpy.spin(n)
    except Exception:
        pass


if __name__ == '__main__':
    main()
