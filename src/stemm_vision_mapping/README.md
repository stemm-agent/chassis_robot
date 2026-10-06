# stemm_vision_mapping（视觉融合建图，手动入口）

RTAB-Map 融合建图：**RGB-D（Astra S）+ 2D 激光（/scan）+ 轮式里程计（/odom_combined）**。
本轮范围：**只做建图**，命令行手动运行，不接 App、不做自动探索/返航、不动 Nav2 与生产链路。

## 前置条件（由常驻服务提供，本包不启动）
| 输入 | 话题 | 提供方 |
|---|---|---|
| 彩色/深度/内参 | `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/{color,depth}/camera_info` | `stemm-astra-camera.service` |
| 2D 激光 | `/scan` | `wheeltec-lidar.service` |
| 里程计与 TF | `/odom_combined`、`odom_combined→base_footprint→…` | `wheeltec-robot.service`（EKF + 静态 TF） |
| 3D 点云（可选） | `/x10/lslidar_point_cloud` | `wheeltec-lidar.service` |

## 用法
```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 1) 先做前置检查（只读）
bash ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/vision_preflight.sh

# 2) 启动建图（前台运行，Ctrl+C 结束）
ros2 launch stemm_vision_mapping vision_mapping.launch.py
#    开新图（会删指定库）：   ... delete_db:=true
#    指定库路径：            ... database_path:=/home/hnh/.local/share/stemm/vision_maps/run1.db
#    放宽同步容差：          ... sync_window:=0.15
#    额外吃雷达点云：        ... use_scan_cloud:=true

# 3) 遥控走车建图（人在旁边看着，慢速）

# 4) 导出 2D 地图（导航用）+ 查看数据库
bash ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/save_vision_map.sh

# 5) 优雅停止（若已在终端 Ctrl+C 可跳过）
bash ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/stop_vision_mapping.sh
```

## 产物
- 2D 占据栅格：`~/.local/share/stemm/vision_maps/stemm_vision_map_<时间戳>.{pgm,yaml}`（**不覆盖**现有 cartographer 地图）
- RTAB-Map 数据库：`~/.local/share/stemm/vision_maps/rtabmap_<时间戳>.db`（含位姿图与回环，供下一轮导航定位复用）

## 设计取舍（与厂商包的关键差异）
1. `frame_id=base_footprint`、显式 `map_frame_id=map` / `odom_frame_id=odom_combined`（厂商用 `camera_link`，且未指定 map/odom 帧）。
2. **不启动 rgbd_odometry**：轮式里程计做运动骨架，视觉只做特征/回环纠正（避免与 EKF 争 `odom_combined→base_footprint`）。
3. **不 bringup 雷达/相机/底盘**：厂商 launch 会重复拉起，抢串口与 USB。
4. `publish_tf=true`：建图期 rtabmap 是 `map→odom` 唯一发布者（AMCL 未运行）。
5. `approx_sync_max_interval=0.12 s`：实测彩色帧率在 8–26 Hz 间浮动、深度 ~12 Hz，厂商 0.05 s 过严。
6. `Grid/3D=false` + `CellSize=0.05`：直接产 2D 占据图给 Nav2。
7. `database_path` 显式指定且按时间戳命名：厂商三套流程共用 `~/.ros/rtabmap.db` 且建图带 `-d` 会删库。

## 回滚
- 停止节点：`scripts/stop_vision_mapping.sh`
- 移除本包：`rm -rf ~/wheeltec_ros2/src/stemm_vision_mapping ~/wheeltec_ros2/build/stemm_vision_mapping ~/wheeltec_ros2/install/stemm_vision_mapping`
- 依赖回滚（如需）：`sudo apt-get remove ros-humble-rtabmap*`（不影响生产链路）
- 本包不修改任何 systemd 服务与生产参数文件。

## 2026-09-22 相机安装高度对齐（消除 TF 双父抖动）

实测相机光心离地 0.26 m；此前 `base_footprint -> camera_link` 有两条互相冲突的来源，
tf2 会在两者之间来回切换（0.192 m / 0.306 m，差 11.4 cm），导致视觉点云高度抖动、
15 cm 以下障碍物的点被 `min_obstacle_height: 0.05` 滤掉。已把两条来源对齐：

| 文件 | 位置 | 改为 |
|---|---|---|
| turn_on_wheeltec_robot/config/robot_model.yaml | four_wheel_diff / four_wheel_diff_bs 的 base_to_camera | `[0.29879, 0.00024, 0.26]` |
| wheeltec_robot_urdf/.../four_wheel_diff_robot.urdf | camera_joint origin xyz | `0.29879 0.00024 0.11955`（0.26 - base_to_link 的 0.14045） |

两条链现在完全一致，抖动归零。备份见 /home/hnh/migration/camera_tf_fix_*/。

## 2026-09-22 生产导航功能对齐 + 验收记录（模拟验收，未下发真实导航目标）

对齐项（参考生产 `wheeltec_nav2.launch.py` / `wheeltec.rviz`）：

| 生产功能 | 本包实现 | 验证方式 |
|---|---|---|
| nav2 bringup：map_server/amcl/controller/planner/smoother/behavior/bt_navigator/waypoint_follower + 两个 lifecycle manager | `vision_nav.launch.py` 全部自带 | `ros2 lifecycle get` 8 个节点全 active |
| 定位权威 `amcl_odom_limiter`（map→odom 唯一发布者，30 Hz，0.25 m/s / 0.35 rad/s） | 参数逐字对齐 | `/amcl_odom_limiter/status` 被常驻 contract adaptor 消费，ready/blocked/anomaly 正常 |
| 单点导航（action） | `/navigate_to_pose`（标准 nav2 action） | action 服务器存在，10 个生产侧客户端可直接复用 |
| 单点导航（话题） | `/goal_pose` → bt_navigator（本 nav2 版本原生订阅；语音侧 `command_recognition` 走这条路） | 订阅者实测为 bt_navigator |
| RViz 交互 | `config/vision_nav.rviz`：Nav2 面板 + Nav2 Goal 工具 + 2D Goal Pose + 2D Pose Estimate(`/initialpose`) + 地图/全局与局部代价地图/路径/足迹/粒子/视觉点云/视觉栅格 | RViz 无插件错误加载；订阅者列表含 rviz2_vision_nav |
| 可选 waypoint_cycle / lasertracker | `enable_waypoint_cycle` / `enable_lasertracker` 同名参数 | `nav2_waypoint_cycle` 在 NUC 上未安装（生产同样不可用）；`simple_follower_ros2/lasertracker` 在 |

验收（`scripts/nav_acceptance.sh`，**默认只做规划、不下发运动目标**）：17 项通过 / 0 项失败。
- 8 个生命周期节点 active；`/scan` 12.0 Hz；`map→base_footprint` TF 正常；
- 局部代价地图订阅 `/vision/live_cloud`（视觉融合确实进入避障链路）；
- 规划器在融合地图上成功产出路径（`compute_path_to_pose` → SUCCEEDED）。

**重要操作前提**：机器人停在充电桩内时，其所在栅格在全局代价地图上被膨胀为高代价
（实测该点代价 89，周围 0.25 m 内有 23 个障碍栅格），规划器会拒绝"从桩上起步"
（planner 原话 `Starting point in lethal space!`）。此时验收脚本会自动改用**显式空闲起点**
继续验证规划链路，并打印 `ROBOT_CELL_VALUE` 提示。真实单点导航前请先把机器人驶离桩位。

真实导航验收（**机器人必须不在桩上、现场无人**）：
```bash
ALLOW_MOTION=1 bash ~/wheeltec_ros2/src/stemm_vision_mapping/scripts/nav_acceptance.sh --move
```
不带 `ALLOW_MOTION=1` 时 `--move` 会被拒绝（防误动闸门）。

单点目标下发（命令行）：
```bash
bash scripts/nav_goal.sh --x 1.5 --y 0.0          # 走 /navigate_to_pose（同 RViz "Nav2 Goal"）
bash scripts/nav_goal.sh --x 1.5 --y 0.0 --topic  # 走 /goal_pose（同 RViz "2D Goal Pose" 与语音链路）
bash scripts/nav_goal.sh --status                 # 查看 action 服务器与客户端
```
