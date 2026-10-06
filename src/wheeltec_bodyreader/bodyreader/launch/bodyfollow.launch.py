import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 启动 360° 激光雷达（避障用）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('turn_on_wheeltec_robot'), 'launch', 'wheeltec_lidar.launch.py')
            )
        ),
        
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
        
        # 跟随节点 (PD控制, 发布到 /cmd_vel_raw)
        # z_p=1.2: 角度P增益, z_d=0.15: 角度D增益 (从0.5降至0.15, 抑制PD过冲振荡)
        Node(
            package='bodyreader',
            executable='follower',
            name='body_follower',
            parameters=[
                {'bodyfollow_x_p': 0.5},
                {'bodyfollow_x_d': 0.33},
                {'bodyfollow_z_p': 1.2},
                {'bodyfollow_z_d': 0.15},
                {'mode': 2}
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
        
        # VFH 避障节点（360°雷达避障，参考 Navigation2 DWB 策略）
        # follower 发布期望速度到 /cmd_vel_raw，body_avoider 融合 /scan 后输出安全 /cmd_vel
        Node(
            package='bodyreader',
            executable='body_avoider',
            name='body_avoider',
            output='screen',
            parameters=[
                {'hist_bins': 72},               # 72扇区，5°分辨率
                {'safe_distance': 1.2},           # 安全距离1.2m
                {'safe_distance_stop': 0.5},      # 0.5m内强制停车
                {'robot_radius': 0.20},           # 机器人半径
                {'hist_threshold': 0.25},         # 直方图阻塞阈值（配合线性密度，墙壁~0.45m触发）
                {'smooth_width': 2},              # 平滑窗口
                {'max_linear_speed': 0.5},        # 最大线速度
                {'max_angular_speed': 1.0},       # 最大角速度
                {'angular_p_gain': 2.5},          # 转向P增益
                {'repulsive_gain': 0.8},          # 排斥力增益（从2.0降至0.8，避免与follower PD振荡）
                {'min_valley_width_deg': 30.0},   # 最小安全扇区30°

                # 人体方向豁免（距离门控）
                {'exemption_angle_deg': 25.0},     # 豁免半角25°（人体周围±25°豁免）
                {'exemption_dist_margin': 0.3},    # 距离门控0.3m（测距<人距-0.3m视为障碍物）

                # 绕行侧方脱离
                {'bypass_clearance_distance': 0.3}, # 侧方障碍物脱离阈值(m)，绕行中该侧所有点距>此值即视为绕过
            ]
        ),

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


