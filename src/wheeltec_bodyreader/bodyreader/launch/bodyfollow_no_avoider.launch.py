"""
人体跟随 — 无避障版本 (bodyfollow_no_avoider)

与 bodyfollow.launch.py 的区别：
    1. 不启动 body_avoider (VFH避障) 节点
    2. 不启动 360° 激光雷达
    3. follower_no_avoider 将 PD 速度发到 /cmd_vel_raw
    4. collision_guard 仅负责“后退碰撞后急停”安全门控

架构（无避障）：
    body_posture → follower_no_avoider (PD控制) → /cmd_vel_raw → collision_guard → /cmd_vel

适用场景：
  - 无激光雷达的纯跟随测试
  - 排查避障是否干扰跟随
  - 对比有/无避障的跟随效果

启动方式:
  ros2 launch bodyreader bodyfollow_no_avoider.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ★ 无避障版本不启动激光雷达
        # (如需在 rviz 查看雷达数据，取消下面注释)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('turn_on_wheeltec_robot'), 'launch', 'wheeltec_lidar.launch.py')
        #     )
        # ),
        
        # 人体骨架识别节点
        Node(
            package='bodyreader',
            executable='main',
            name='body_main',
            output='screen',
            parameters=[
                {'rgb_stream': True},
                {'body_stream': True}
            ]
        ),
        
        # 动作定义节点
        Node(
            package='bodyreader',
            executable='bodydata_process',
            name='bodydata_process',
            output='screen'
        ),
        
        # 跟随节点 — 无避障版 (PD控制, 输出到 /cmd_vel_raw)
        # z_p=1.2, z_d=0.5: 柔和追踪, 避免过冲振荡
        Node(
            package='bodyreader',
            executable='follower_no_avoider',
            name='body_follower_no_avoider',
            parameters=[
                {'bodyfollow_x_p': 0.5},
                {'bodyfollow_x_d': 0.33},
                {'bodyfollow_z_p': 1.2},
                {'bodyfollow_z_d': 0.5},
                {'mode': 2}
            ]
        ),

        # 后退碰撞保护：仅在持续后退且 odom 显示被顶住/走不动时告警
        Node(
            package='bodyreader',
            executable='collision_guard',
            name='collision_guard',
            output='screen',
            parameters=[
                {'reverse_speed_threshold': -0.05},
                {'reverse_confirm_time': 0.20},
                {'reverse_stall_confirm_time': 0.50},
                {'reverse_motion_ratio_threshold': 0.35},
                {'soft_planar_accel_threshold': 1.2},
                {'hard_planar_accel_threshold': 2.5},
                {'soft_jerk_threshold': 6.0},
                {'hard_jerk_threshold': 15.0},
                {'forward_decel_threshold': 1.2},
                {'yaw_rate_threshold': 0.8},
                {'odom_speed_threshold': 0.03},
                {'impact_memory_time': 0.40},
                {'odom_reverse_speed_threshold': 0.05},
                {'odom_linear_accel_quiet_threshold': 0.15},
                {'odom_yaw_rate_quiet_threshold': 0.10},
                {'odom_yaw_accel_quiet_threshold': 0.50},
                {'wheel_push_confirm_cycles': 2},
                {'release_hold_time': 1.0}
            ]
        ),
        
        # 反馈（语音反馈/图像UI）
        Node(
            package='bodyreader',
            executable='feedback',
            name='body_feedback',
            output='screen',
            parameters=[
                {'voice_feedback': True},
                {'interaction': 0}
            ]
        ),
        
        # 显示节点
        Node(
            package='bodyreader',
            executable='display.py',
            name='body_display'
        ),
        
        # ★ 无避障版本：不启动 body_avoider
        # follower_no_avoider 直接将期望速度发到 /cmd_vel_raw，再由 collision_guard 输出 /cmd_vel

        # 图像传输节点
        Node(
            package='bodyreader',
            executable='compressed.py',
            name='compressed',
            parameters=[
                {'input_image_topic': '/body/body_display'},
                {'output_image_topic': '/repub/body/body_display/compressed'}
            ]
        ),
    ])
