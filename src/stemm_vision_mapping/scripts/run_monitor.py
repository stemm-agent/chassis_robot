#!/usr/bin/env python3
"""实车建图运行监控：按时间序列记录地图生长、视觉障碍累积、位姿轨迹与云频率，
并周期性保存栅格快照，供事后分析。

用法：run_monitor.py --out <目录> [--period 5] [--snapshot 60]
产物：<目录>/monitor.csv（时间序列）、<目录>/events.log（事件）、
      <目录>/snapshots/{map,obs}_NNNN.pgm（周期快照）
"""
import argparse, csv, math, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import tf2_ros


def grid_to_pgm(msg, path):
    """OccupancyGrid → PGM（0=占据 205=未知 254=空闲）；行 0 为最高 y。"""
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int16).reshape(h, w)
    img = np.full((h, w), 205, dtype=np.uint8)
    img[data >= 65] = 0
    img[(data >= 0) & (data < 65)] = 254
    img = np.flipud(img)          # PGM 行 0 在上
    with open(path, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())
    return int((data >= 65).sum()), int(((data >= 0) & (data < 65)).sum()), int((data < 0).sum())


class Monitor(Node):
    def __init__(self, out_dir, period, snapshot):
        super().__init__('vision_run_monitor')
        self.out = out_dir
        os.makedirs(os.path.join(out_dir, 'snapshots'), exist_ok=True)
        self.period, self.snap_period = period, snapshot
        self.map_msg = None
        self.obs_msg = None
        self.cloud_msgs = 0
        self.prev_cloud = 0
        self.t0 = time.time()
        self.snap_idx = 0
        self.csv_path = os.path.join(out_dir, 'monitor.csv')
        self.ev_path = os.path.join(out_dir, 'events.log')
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['elapsed_s', 'wall', 'pose_x', 'pose_y', 'yaw_deg',
                         'map_w', 'map_h', 'map_occ', 'map_free', 'map_unk', 'map_ox', 'map_oy',
                         'obs_w', 'obs_h', 'obs_occ', 'obs_free', 'obs_unk', 'obs_ox', 'obs_oy',
                         'cloud_hz', 'cloud_total'])
        self.f.flush()

        q_map = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=QoSReliabilityPolicy.RELIABLE)
        q_cloud = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                             durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, q_map)
        self.create_subscription(OccupancyGrid, '/vision/obstacle_grid', self.on_obs, q_map)
        self.create_subscription(PointCloud2, '/vision/live_cloud', self.on_cloud, q_cloud)
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.create_timer(period, self.tick)
        self.create_timer(snapshot, self.snap)
        self.event(f'监控启动 out={out_dir} period={period}s snapshot={snapshot}s')

    def event(self, text):
        line = f'[{time.strftime("%H:%M:%S")}] {text}'
        self.get_logger().info(text)
        with open(self.ev_path, 'a', encoding='utf-8') as f:
            f.write(line + '\n')

    def on_map(self, msg):
        self.map_msg = msg
    def on_obs(self, msg):
        self.obs_msg = msg
    def on_cloud(self, msg):
        self.cloud_msgs += 1

    def pose(self):
        try:
            tr = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        return t.x, t.y, yaw

    def tick(self):
        el = time.time() - self.t0
        p = self.pose()
        m, o = self.map_msg, self.obs_msg
        def stats(g):
            if g is None:
                return ('', '', '', '', '', '', '')
            d = np.array(g.data, dtype=np.int16)
            occ = int((d >= 65).sum()); free = int(((d >= 0) & (d < 65)).sum()); unk = int((d < 0).sum())
            return (g.info.width, g.info.height, occ, free, unk,
                    round(g.info.origin.position.x, 3), round(g.info.origin.position.y, 3))
        ms, os_ = stats(m), stats(o)
        hz = (self.cloud_msgs - self.prev_cloud) / self.period
        self.prev_cloud = self.cloud_msgs
        self.w.writerow([round(el, 1), time.strftime('%H:%M:%S'),
                         round(p[0], 3) if p else '', round(p[1], 3) if p else '',
                         round(p[2], 1) if p else '',
                         *ms, *os_, round(hz, 1), self.cloud_msgs])
        self.f.flush()

    def snap(self):
        self.snap_idx += 1
        d = os.path.join(self.out, 'snapshots')
        if self.map_msg is not None:
            occ, free, unk = grid_to_pgm(self.map_msg, os.path.join(d, f'map_{self.snap_idx:04d}.pgm'))
            self.event(f'快照#{self.snap_idx} /map {self.map_msg.info.width}x{self.map_msg.info.height} 占据={occ} 空闲={free} 未知={unk}')
        if self.obs_msg is not None:
            occ, free, unk = grid_to_pgm(self.obs_msg, os.path.join(d, f'obs_{self.snap_idx:04d}.pgm'))
            self.event(f'快照#{self.snap_idx} /vision/obstacle_grid {self.obs_msg.info.width}x{self.obs_msg.info.height} 障碍格={occ}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--period', type=float, default=5.0)
    ap.add_argument('--snapshot', type=float, default=60.0)
    a = ap.parse_args()
    rclpy.init()
    n = Monitor(a.out, a.period, a.snapshot)
    try:
        rclpy.spin(n)
    except Exception:
        pass
    finally:
        n.event('监控结束')
        try:
            n.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
