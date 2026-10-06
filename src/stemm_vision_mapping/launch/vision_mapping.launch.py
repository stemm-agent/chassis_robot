"""STEMM 视觉融合建图（手动命令行入口）。

启动内容：rtabmap_sync/rgbd_sync + rtabmap_slam/rtabmap（SLAM 模式）。
明确不启动：雷达、相机、底盘 bringup（由常驻 systemd 服务提供）；
            自动探索、返航、Nav2、AMCL（本轮范围外，且避免与生产链路冲突）。

用法：
  ros2 launch stemm_vision_mapping vision_mapping.launch.py
  ros2 launch stemm_vision_mapping vision_mapping.launch.py delete_db:=true      # 开新图（会删旧库）
  ros2 launch stemm_vision_mapping vision_mapping.launch.py database_path:=/path/to/x.db
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_MAP_DIR = os.path.expanduser('~/.local/share/stemm/vision_maps')


def _setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('stemm_vision_mapping')
    params_file = LaunchConfiguration('params_file').perform(context)
    database_path = os.path.expanduser(
        LaunchConfiguration('database_path').perform(context))
    delete_db = LaunchConfiguration('delete_db').perform(context).lower() == 'true'
    launch_viz = LaunchConfiguration('launch_viz').perform(context).lower() == 'true'
    launch_rviz = LaunchConfiguration('launch_rviz').perform(context).lower() == 'true'
    live_cloud = LaunchConfiguration('live_cloud').perform(context).lower() == 'true'
    obstacle_grid = LaunchConfiguration('obstacle_grid').perform(context).lower() == 'true'
    use_scan_cloud = LaunchConfiguration('use_scan_cloud').perform(context).lower() == 'true'
    sync_window = LaunchConfiguration('sync_window').perform(context)

    if not params_file:
        params_file = os.path.join(pkg_share, 'config', 'vision_rtabmap_params.yaml')

    db_dir = os.path.dirname(database_path)
    os.makedirs(db_dir, exist_ok=True)

    if delete_db:
        for suffix in ('', '.backup', '-journal', '-wal', '-shm'):
            stale = database_path + suffix if suffix else database_path
            if os.path.exists(stale):
                os.remove(stale)
                print(f'[stemm_vision_mapping] removed stale database: {stale}')

    remaps_common = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[params_file, {
            'approx_sync_max_interval': float(sync_window),
            'use_sim_time': False,
        }],
        remappings=remaps_common,
    )

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[params_file, {
            'database_path': database_path,
            'subscribe_scan_cloud': use_scan_cloud,
            'use_sim_time': False,
            # 让 /cloud_map（视觉融合 3D 地图）更稠密、更远：
            'cloud_decimation': 2,
            'cloud_max_depth': 6.0,
            'cloud_voxel_size': 0.0,
        }],
        remappings=remaps_common + [
            ('odom', '/odom_combined'),
            ('scan', '/scan'),
            ('scan_cloud', '/x10/lslidar_point_cloud'),
            ('rgbd_image', '/rgbd_image'),
        ],
        arguments=['--delete_db_on_start'] if delete_db else [],
    )

    live_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='vision_live_cloud',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': float(sync_window),
            'sync_queue_size': 10,
            'qos': 2,
            'decimation': 2,
            'min_depth': 0.3,
            'max_depth': 6.0,
            'voxel_size': 0.0,
            'use_sim_time': False,
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('cloud', '/vision/live_cloud'),
        ],
    )

    actions = [
        LogInfo(msg='=' * 72),
        LogInfo(msg='STEMM 视觉融合建图（RTAB-Map）已启动'),
        LogInfo(msg=f'  参数文件   : {params_file}'),
        LogInfo(msg=f'  地图数据库 : {database_path}'),
        LogInfo(msg=f'  删库重建   : {delete_db}'),
        LogInfo(msg=f'  同步容差   : {sync_window} s'),
        LogInfo(msg=f'  雷达点云   : {use_scan_cloud}  (/x10/lslidar_point_cloud)'),
        LogInfo(msg=f'  可视化     : {launch_viz}'),
        LogInfo(msg='  遥控走车即可建图；结束时用 scripts/save_vision_map.sh 导出 2D 地图'),
        LogInfo(msg='=' * 72),
        rgbd_sync,
        rtabmap,
    ]
    if live_cloud:
        actions.append(live_cloud_node)
    if obstacle_grid:
        actions.append(Node(
            package='stemm_vision_mapping',
            executable='depth_obstacle_grid.py',
            name='depth_obstacle_grid',
            output='screen',
            parameters=[{
                'cloud_topic': '/vision/live_cloud',
                'output_topic': '/vision/obstacle_grid',
                'global_frame': 'map',
                'resolution': 0.05,
                'z_min': 0.05,
                'z_max': 1.80,
                'range_max': 6.0,
                'self_filter_radius': 0.30,
                'process_hz': 5.0,
                'publish_hz': 1.0,
                'min_points_per_cell': 2,
            }],
        ))

    if launch_rviz:
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'vision_mapping.rviz')],
        ))

    if launch_viz:
        actions.append(Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[params_file, {'use_sim_time': False}],
            remappings=remaps_common + [
                ('odom', '/odom_combined'),
                ('scan', '/scan'),
                ('rgbd_image', '/rgbd_image'),
            ],
        ))
    return actions


def generate_launch_description():
    default_db = os.path.join(
        DEFAULT_MAP_DIR, datetime.now().strftime('rtabmap_%Y%m%d-%H%M%S.db'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'database_path', default_value=default_db,
            description='RTAB-Map 数据库路径（默认按时间戳落在 ~/.local/share/stemm/vision_maps/）'),
        DeclareLaunchArgument(
            'delete_db', default_value='false',
            description='true = 删除该数据库后从零建图'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='留空使用包内 config/vision_rtabmap_params.yaml'),
        DeclareLaunchArgument(
            'sync_window', default_value='0.12',
            description='RGB-D 近似同步容差（秒）；彩色帧率浮动时适当放宽'),
        DeclareLaunchArgument(
            'use_scan_cloud', default_value='false',
            description='true = 额外把 /x10/lslidar_point_cloud 喂给 rtabmap'),
        DeclareLaunchArgument(
            'launch_viz', default_value='false',
            description='true = 启动 rtabmap_viz（需已安装 ros-humble-rtabmap-viz）'),
        DeclareLaunchArgument(
            'live_cloud', default_value='true',
            description='true = 额外发布 /vision/live_cloud（当前视角的实时彩色点云，约 10 Hz）'),
        DeclareLaunchArgument(
            'obstacle_grid', default_value='true',
            description='true = 累积深度障碍栅格到 /vision/obstacle_grid（补激光盲区，可保存/进代价地图）'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='true = 用包内 config/vision_mapping.rviz 启动 RViz2（需有桌面显示环境）'),
        OpaqueFunction(function=_setup),
    ])
