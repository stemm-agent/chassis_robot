from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    rtmp_url = LaunchConfiguration('rtmp_url')
    media_host = LaunchConfiguration('media_host')
    rtmp_base_url = LaunchConfiguration('rtmp_base_url')
    hls_base_url = LaunchConfiguration('hls_base_url')
    stream_app = LaunchConfiguration('stream_app')
    stream_name = LaunchConfiguration('stream_name')
    hub_id = LaunchConfiguration('hub_id')
    camera_device_id = LaunchConfiguration('camera_device_id')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    bitrate = LaunchConfiguration('bitrate')
    auto_start = LaunchConfiguration('auto_start')

    rtmp_pusher = Node(
        package='cloud_video_pusher',
        executable='rtmp_image_pusher',
        name='rtmp_image_pusher',
        output='screen',
        parameters=[{
            'image_topic': ParameterValue(image_topic, value_type=str),
            'rtmp_url': ParameterValue(rtmp_url, value_type=str),
            'media_host': ParameterValue(media_host, value_type=str),
            'rtmp_base_url': ParameterValue(rtmp_base_url, value_type=str),
            'hls_base_url': ParameterValue(hls_base_url, value_type=str),
            'stream_app': ParameterValue(stream_app, value_type=str),
            'stream_name': ParameterValue(stream_name, value_type=str),
            'hub_id': ParameterValue(hub_id, value_type=str),
            'camera_device_id': ParameterValue(camera_device_id, value_type=str),
            'width': ParameterValue(width, value_type=int),
            'height': ParameterValue(height, value_type=int),
            'fps': ParameterValue(fps, value_type=float),
            'bitrate': ParameterValue(bitrate, value_type=str),
            'auto_start': ParameterValue(auto_start, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('rtmp_url', default_value=''),
        DeclareLaunchArgument('media_host', default_value=''),
        DeclareLaunchArgument('rtmp_base_url', default_value='rtmp://8.148.249.36/live'),
        DeclareLaunchArgument('hls_base_url', default_value='http://8.148.249.36:8088/live'),
        DeclareLaunchArgument('stream_app', default_value='live'),
        DeclareLaunchArgument('stream_name', default_value='robot001'),
        DeclareLaunchArgument('hub_id', default_value='910007'),
        DeclareLaunchArgument('camera_device_id', default_value='robot-camera-1'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('bitrate', default_value='800k'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        rtmp_pusher,
    ])