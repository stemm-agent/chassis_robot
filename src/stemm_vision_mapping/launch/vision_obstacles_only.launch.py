"""只启动"视觉障碍"链路（不含 rtabmap）——供 AMCL 定位导航时提供实时深度障碍层。

启动内容：
  rtabmap_util/point_cloud_xyzrgb  → /vision/live_cloud        （当前视角实时彩色点云，约 10-15 Hz）
  depth_obstacle_grid.py           → /vision/obstacle_grid     （累积的激光盲区障碍栅格，1 Hz）

明确不启动：rtabmap（避免与 AMCL 争 map→odom）、雷达/相机 bringup、Nav2。
用法：
  ros2 launch stemm_vision_mapping vision_obstacles_only.launch.py
  可选参数：sync_window / min_points_per_cell / z_min / z_max / range_max / publish_hz
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULTS = dict(sync_window='0.15', min_points_per_cell='2', z_min='0.05',
                z_max='1.80', range_max='6.0', publish_hz='1.0', decimation='2')


def _setup(context, *args, **kwargs):
    g = lambda k: LaunchConfiguration(k).perform(context)
    sync_window = float(g('sync_window'))
    remaps = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        ('cloud', '/vision/live_cloud'),
    ]
    live = Node(
        package='rtabmap_util', executable='point_cloud_xyzrgb', name='vision_live_cloud',
        output='screen',
        parameters=[{
            'approx_sync': True, 'approx_sync_max_interval': sync_window,
            'sync_queue_size': 10, 'qos': 2,
            'decimation': int(g('decimation')), 'min_depth': 0.3, 'max_depth': float(g('range_max')),
            'voxel_size': 0.0, 'use_sim_time': False,
        }],
        remappings=remaps,
    )
    grid = Node(
        package='stemm_vision_mapping', executable='depth_obstacle_grid.py',
        name='depth_obstacle_grid', output='screen',
        parameters=[{
            'cloud_topic': '/vision/live_cloud', 'output_topic': '/vision/obstacle_grid',
            'global_frame': 'map', 'resolution': 0.05,
            'z_min': float(g('z_min')), 'z_max': float(g('z_max')),
            'range_max': float(g('range_max')), 'self_filter_radius': 0.30,
            'publish_hz': float(g('publish_hz')),
            'min_points_per_cell': int(g('min_points_per_cell')),
        }],
    )
    return [
        LogInfo(msg='视觉障碍层：/vision/live_cloud（实时）+ /vision/obstacle_grid（累积）'),
        LogInfo(msg='注意：本 launch 不含 rtabmap，map→odom 由 AMCL 提供'),
        live, grid,
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in DEFAULTS.items()] +
        [OpaqueFunction(function=_setup)])
