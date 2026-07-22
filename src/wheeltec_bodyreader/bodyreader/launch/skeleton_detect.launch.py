"""
骨架识别纯检测 demo

启动方式:
  ros2 launch bodyreader skeleton_detect.launch.py

仅启动 Astra 深度相机的骨架检测 + 可视化显示，不包含底盘控制、跟随、姿态交互、语音反馈。

原始数据话题:
  /bodylist    — 人体骨架关节坐标 (bodyreader_msg/Bodylist)
  /body/mask   — 人体掩膜数据 (bodyreader_msg/Maskdata)
  /image_raw   — RGB 图像 (sensor_msgs/Image)

可视化话题 (可用 rqt_image_view 查看):
  /body/body_display              — 骨架画在 RGB 图上的合成画面
  /repub/body/body_display/compressed  — 压缩版

在 rqt 中查看:
  rqt_image_view /body/body_display
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 骨架识别节点 (Orbbec Astra SDK)
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

        # 可视化: 把骨架关节/骨骼画到 RGB 图像上
        Node(
            package='bodyreader',
            executable='display.py',
            name='body_display'
        ),

        # 图像压缩转发 (方便 rqt_image_view 低带宽查看)
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
