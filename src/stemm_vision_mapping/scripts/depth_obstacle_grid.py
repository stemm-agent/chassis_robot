#!/usr/bin/env python3
"""把深度点云累积成一张"只增不减"的 2D 障碍栅格（补激光盲区）。

- 订阅 PointCloud2（默认 /vision/live_cloud，相机光学系）
- TF 变换到 map 帧，按高度带过滤（默认 0.05~1.80 m：滤地面与过高点）
- 投影到 0.05 m 栅格，障碍**只增不减**（整段建图过程持续累积）
- 发布 nav_msgs/OccupancyGrid 到 /vision/obstacle_grid（transient_local，1 Hz）
  → 可被 map_saver_cli 存成 pgm/yaml，也可作为 Nav2 StaticLayer 的输入

参数：cloud_topic / global_frame / resolution / z_min / z_max / range_max /
      self_filter_radius / process_hz / publish_hz / output_topic
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
import tf2_ros


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class DepthObstacleGrid(Node):
    def __init__(self):
        super().__init__('depth_obstacle_grid')
        self.declare_parameter('cloud_topic', '/vision/live_cloud')
        self.declare_parameter('output_topic', '/vision/obstacle_grid')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 1.80)
        self.declare_parameter('range_max', 6.0)
        self.declare_parameter('self_filter_radius', 0.30)
        self.declare_parameter('process_hz', 5.0)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('max_side_cells', 6000)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('low_topic', '/vision/low_cloud')
        self.declare_parameter('low_z_min', 0.05)
        self.declare_parameter('low_z_max', 0.35)
        self.declare_parameter('voxel_topic', '/vision/voxel_cloud')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('cloud_publish_hz', 2.0)

        g = self.get_parameter
        self.cloud_topic = g('cloud_topic').value
        self.output_topic = g('output_topic').value
        self.global_frame = g('global_frame').value
        self.res = float(g('resolution').value)
        self.z_min = float(g('z_min').value)
        self.z_max = float(g('z_max').value)
        self.range_max = float(g('range_max').value)
        self.self_r = float(g('self_filter_radius').value)
        self.max_side = int(g('max_side_cells').value)
        self.min_pts = max(1, int(g('min_points_per_cell').value))
        self.low_topic = g('low_topic').value
        self.low_z_min = float(g('low_z_min').value)
        self.low_z_max = float(g('low_z_max').value)
        self.voxel_topic = g('voxel_topic').value
        self.voxel_size = max(0.01, float(g('voxel_size').value))
        self.cloud_hz = max(0.1, float(g('cloud_publish_hz').value))
        self.last_low = np.zeros((0, 3), dtype=np.float64)
        self.last_vox = np.zeros((0, 3), dtype=np.float64)

        self.cells = set()
        self.last_points = 0
        self.dropped = 0
        self.filtered = 0

        qos_cloud = QoSProfile(depth=2, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, qos_cloud)

        qos_pub = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, qos_pub)
        qos_vis = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.VOLATILE)
        self.pub_low = self.create_publisher(PointCloud2, self.low_topic, qos_vis)
        self.pub_vox = self.create_publisher(PointCloud2, self.voxel_topic, qos_vis)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(1.0 / max(0.1, float(g('publish_hz').value)), self.publish_grid)
        self.create_timer(1.0 / self.cloud_hz, self.publish_clouds)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'depth_obstacle_grid: {self.cloud_topic} -> {self.output_topic} '
            f'(frame={self.global_frame}, res={self.res}, z=[{self.z_min},{self.z_max}], '
            f'range<={self.range_max})')

    def report(self):
        self.get_logger().info(
            f'已累积障碍栅格 {len(self.cells)} 格；上帧有效点 {self.last_points}；'
            f'单点噪点滤除 {self.filtered}；超界丢弃 {self.dropped}')

    def on_cloud(self, msg):
        names = {f.name: f.offset for f in msg.fields}
        if not all(k in names for k in ('x', 'y', 'z')):
            return
        step = msg.point_step
        n = msg.width * msg.height
        if n == 0 or step == 0:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < n * step:
            return
        buf = raw[:n * step].reshape(n, step)

        def col(name):
            off = names[name]
            return buf[:, off:off + 4].copy().view(np.float32).reshape(-1)

        x, y, z = col('x'), col('y'), col('z')
        ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 0.0)
        if not ok.any():
            return

        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, msg.header.frame_id, Time())
        except Exception:
            return
        t = tr.transform.translation
        R = quat_to_matrix(tr.transform.rotation)
        P = np.stack([x[ok], y[ok], z[ok]], axis=1)
        Pw = P @ R.T + np.array([t.x, t.y, t.z])

        dx = Pw[:, 0] - t.x
        dy = Pw[:, 1] - t.y
        dist = np.hypot(dx, dy)
        keep = ((dist > self.self_r) & (dist < self.range_max) &
                (Pw[:, 2] > self.z_min) & (Pw[:, 2] < self.z_max))
        self.last_points = int(keep.sum())
        low = keep & (Pw[:, 2] >= self.low_z_min) & (Pw[:, 2] <= self.low_z_max)
        self.last_low = Pw[low]
        self.last_vox = (((np.floor(Pw[keep] / self.voxel_size) + 0.5) * self.voxel_size)
                         if keep.any() else np.zeros((0, 3), dtype=np.float64))
        if not keep.any():
            return

        ix = np.floor(Pw[keep, 0] / self.res).astype(np.int64)
        iy = np.floor(Pw[keep, 1] / self.res).astype(np.int64)
        # 单格最少点数滤波：孤立噪点（1 个点）不计入障碍，避免栅格边界被撑大
        # 索引打包必须负数安全：先整体加偏移，再用 (2*OFF) 进制展开
        OFF = 1 << 20
        base = 2 * OFF
        key = (ix + OFF) * base + (iy + OFF)
        uniq, counts = np.unique(key, return_counts=True)
        good = uniq[counts >= self.min_pts]
        if good.size:
            gx = good // base - OFF
            gy = good % base - OFF
            self.cells.update(zip(gx.tolist(), gy.tolist()))
        self.filtered = int(uniq.size - good.size)

    @staticmethod
    def _cloud_msg(pts, frame_id, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = int(pts.shape[0])
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * int(pts.shape[0])
        msg.is_dense = True
        msg.data = np.ascontiguousarray(pts, dtype=np.float32).tobytes()
        return msg

    def publish_clouds(self):
        stamp = self.get_clock().now().to_msg()
        if self.last_low.shape[0]:
            self.pub_low.publish(self._cloud_msg(self.last_low, self.global_frame, stamp))
        if self.last_vox.shape[0]:
            self.pub_vox.publish(self._cloud_msg(self.last_vox, self.global_frame, stamp))

    def publish_grid(self):
        if not self.cells:
            return
        xs = [c[0] for c in self.cells]
        ys = [c[1] for c in self.cells]
        minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
        w = maxx - minx + 1
        h = maxy - miny + 1
        if w > self.max_side or h > self.max_side:
            self.dropped += 1
            self.get_logger().warn(
                f'栅格超出上限 {w}x{h} > {self.max_side}，本次不发布（请调大 max_side_cells）')
            return
        data = np.full((h, w), -1, dtype=np.int8)
        for (cx, cy) in self.cells:
            data[cy - miny, cx - minx] = 100
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.info.resolution = self.res
        msg.info.width = int(w)
        msg.info.height = int(h)
        msg.info.origin.position.x = float(minx * self.res)
        msg.info.origin.position.y = float(miny * self.res)
        msg.info.origin.orientation.w = 1.0
        msg.data = data.reshape(-1).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DepthObstacleGrid()
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
