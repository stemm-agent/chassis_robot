"""视觉融合【导航】一条命令入口。

参考对象 = **车端实况生产导航**（wheeltec_nav2.launch.py，guard_default='false' 的无守卫配置），
组件与参数尽可能逐字照搬，仅两类差异：
  [A] 代价地图图层：加入 vision_voxel_layer（实时深度）+ 全局静态层用"融合视觉/雷达地图"；
  [B] AMCL 硬起点由参数播种（生产有守卫提供粗先验，本模式没有）。

照搬的组件（与生产一致）：
  map_server / amcl / amcl_odom_limiter（map→odom 唯一发布者） / lifecycle_manager_localization
  controller_server(局部代价地图) / smoother_server / planner_server(全局代价地图)
  behavior_server / bt_navigator / waypoint_follower / lifecycle_manager_navigation
  可选（默认 false，与生产同款参数化）：waypoint_cycle、lasertracker
另外启动：本项目的视觉深度链路（point_cloud_xyzrgb + depth_obstacle_grid）
不做：守卫链（生产已默认停用）、cmd_vel 门控（生产默认关闭）、组合容器（保持独立进程便于调试）
单例锁：/tmp/stemm_vision_nav-<uid>.launch.lock（照搬生产的 flock 方案，路径独立以免与生产互锁）

用法：
  ros2 launch stemm_vision_mapping vision_nav.launch.py
  ros2 launch stemm_vision_mapping vision_nav.launch.py initial_x:=1.0 initial_yaw:=1.57
  ros2 launch stemm_vision_mapping vision_nav.launch.py map:=/path/fused.yaml start_vision:=false
  ros2 launch stemm_vision_mapping vision_nav.launch.py rviz:=true      # 带 RViz2 可视化
"""
import errno
import fcntl
import glob
import os
import stat
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

_SINGLETON_FD = None
LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = ['controller_server', 'smoother_server', 'planner_server',
                    'behavior_server', 'bt_navigator', 'waypoint_follower']


def _lock_path():
    return Path('/tmp') / f'stemm_vision_nav-{os.getuid()}.launch.lock'


def _claim_singleton():
    """照搬生产 wheeltec_nav2.launch.py 的单例锁思路（独立路径，避免与生产互锁）。"""
    global _SINGLETON_FD
    if _SINGLETON_FD is not None:
        return
    lock_path = _lock_path()
    flags = os.O_CREAT | os.O_RDWR | os.O_CLOEXEC | getattr(os, 'O_NOFOLLOW', 0)
    try:
        fd = os.open(lock_path, flags, 0o600)
    except OSError as exc:
        raise RuntimeError(f'无法打开视觉导航单例锁 {lock_path}: {exc}') from exc
    locked = False
    try:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_uid != os.getuid():
            raise RuntimeError(f'视觉导航单例锁归属/类型不安全: {lock_path}')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            locked = True
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                raise
            os.lseek(fd, 0, os.SEEK_SET)
            holder = os.read(fd, 64).decode('ascii', errors='replace').strip() or 'unknown'
            raise RuntimeError(
                f'已有视觉导航 launch 在运行（持有者 PID {holder}）；拒绝重复启动')
        os.ftruncate(fd, 0)
        os.write(fd, f'{os.getpid()}\n'.encode('ascii'))
    except Exception:
        if locked:
            fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)
        raise
    _SINGLETON_FD = fd
    print(f'[stemm_vision_mapping] 已持有视觉导航单例锁 {lock_path}')


def _latest_fused_map():
    hits = sorted(glob.glob(os.path.expanduser(
        '~/.local/share/stemm/vision_maps/run_*/validation_conservative_fused.yaml')),
        key=os.path.getmtime)
    return hits[-1] if hits else ''


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('stemm_vision_mapping')
    g = lambda k: LaunchConfiguration(k).perform(context)

    map_yaml = g('map') or _latest_fused_map()
    params_file = g('params_file') or os.path.join(pkg, 'config', 'param_four_wheel_diff_vision_nav.yaml')

    # RewrittenYaml 的取值必须是字符串（传 bool/float 会 'bool' object is not iterable）
    overrides = {
        'use_sim_time': g('use_sim_time'),
        'yaml_filename': map_yaml,
        'initial_pose.x': g('initial_x'),
        'initial_pose.y': g('initial_y'),
        'initial_pose.yaw': g('initial_yaw'),
    }
    configured = RewrittenYaml(source_file=params_file, root_key='',
                               param_rewrites=overrides, convert_types=True)
    use_sim = g('use_sim_time').lower() == 'true'
    autostart = g('autostart').lower() == 'true'      # bool！lifecycle_manager 要求 bool
    delay = float(g('startup_delay'))

    def nav(pkg_name, exe, name, extra=None):
        params = [configured] if extra is None else [configured, extra]
        return Node(package=pkg_name, executable=exe, name=name, output='screen',
                    parameters=params,
                    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    actions = [
        LogInfo(msg='=' * 78),
        LogInfo(msg='视觉融合导航（照搬生产无守卫配置 + 视觉深度层）'),
        LogInfo(msg=f'  融合地图 : {map_yaml or "<未找到，请用 map:= 指定>"}'),
        LogInfo(msg=f'  参数文件 : {params_file}'),
        LogInfo(msg=f'  AMCL 硬起点: x={g("initial_x")} y={g("initial_y")} yaw={g("initial_yaw")}'
                    '（tf_broadcast=false → map→odom 由 amcl_odom_limiter 发布）'),
        LogInfo(msg=f'  视觉深度层: {g("start_vision")}   单例锁: {_lock_path()}'),
        LogInfo(msg='=' * 78),
    ]

    # [1] 视觉深度链路（本项目特有）
    if g('start_vision').lower() == 'true':
        actions.append(Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', name='vision_live_cloud',
            output='screen',
            parameters=[{'approx_sync': True,
                         'approx_sync_max_interval': float(g('sync_window')),
                         'sync_queue_size': 10, 'qos': 2, 'decimation': 2,
                         'min_depth': 0.3, 'max_depth': 6.0, 'voxel_size': 0.0,
                         'use_sim_time': use_sim}],
            remappings=[('rgb/image', '/camera/color/image_raw'),
                        ('rgb/camera_info', '/camera/color/camera_info'),
                        ('depth/image', '/camera/depth/image_raw'),
                        ('cloud', '/vision/live_cloud')]))
        actions.append(Node(
            package='stemm_vision_mapping', executable='depth_obstacle_grid.py',
            name='depth_obstacle_grid', output='screen',
            parameters=[{'cloud_topic': '/vision/live_cloud',
                         'output_topic': '/vision/obstacle_grid',
                         'global_frame': 'map', 'resolution': 0.05,
                         'z_min': 0.05, 'z_max': 1.80, 'range_max': 6.0,
                         'self_filter_radius': 0.30, 'publish_hz': 1.0,
                         'min_points_per_cell': 2}]))

    # [2] 定位层：map_server + amcl + amcl_odom_limiter（逐字照搬生产参数）+ 定位生命周期
    actions.append(GroupAction([
        nav('nav2_map_server', 'map_server', 'map_server'),
        nav('nav2_amcl', 'amcl', 'amcl'),
        Node(package='stemm_nav2_manager', executable='amcl_odom_limiter',
             name='amcl_odom_limiter', output='screen',
             parameters=[{
                 'map_frame': 'map',
                 'odom_frame': 'odom_combined',
                 'base_frame': 'base_footprint',
                 'pose_topic': '/amcl_pose',
                 'publish_rate': 30.0,
                 'max_translation_rate': 0.25,
                 'max_yaw_rate': 0.35,
                 'warn_pose_age_sec': 2.0,
                 'status_topic': '/amcl_odom_limiter/status',
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim, 'autostart': autostart,
                          'node_names': LOCALIZATION_NODES}]),
    ]))

    # [3] 可选组件（与生产同款：默认 false）
    actions.append(Node(name='waypoint_cycle', package='nav2_waypoint_cycle',
                        executable='nav2_waypoint_cycle',
                        condition=IfCondition(g('enable_waypoint_cycle'))))
    actions.append(Node(name='lasertracker', package='simple_follower_ros2',
                        executable='lasertracker', output='screen',
                        condition=IfCondition(g('enable_lasertracker'))))

    # [4] 导航层：等定位就绪后按生命周期顺序拉起
    actions.append(TimerAction(period=delay, actions=[GroupAction([
        nav('nav2_controller', 'controller_server', 'controller_server'),
        nav('nav2_smoother', 'smoother_server', 'smoother_server'),
        nav('nav2_planner', 'planner_server', 'planner_server'),
        nav('nav2_behaviors', 'behavior_server', 'behavior_server'),
        nav('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        nav('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim, 'autostart': autostart,
                          'node_names': NAVIGATION_NODES}]),
    ])]))
    # [5] optional rviz2 (default off; enable with rviz:=true)
    if g('rviz').lower() == 'true':
        rviz_config = os.path.join(pkg, 'config', 'vision_nav.rviz')
        actions.append(LogInfo(msg=f'rviz2 config : {rviz_config}'))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2_vision_nav',
            output='screen', arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim}]))

    return actions


def generate_launch_description():
    _claim_singleton()      # 失败即拒绝启动（与生产一致：宁可不开，不可重复开）
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='融合地图 yaml；留空=自动取最新一份'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='留空=使用包内合并参数'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('start_vision', default_value='true'),
        DeclareLaunchArgument('enable_waypoint_cycle', default_value='false'),
        DeclareLaunchArgument('enable_lasertracker', default_value='false'),
        DeclareLaunchArgument('startup_delay', default_value='6.0'),
        DeclareLaunchArgument('sync_window', default_value='0.15'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='launch rviz2 with config/vision_nav.rviz'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
