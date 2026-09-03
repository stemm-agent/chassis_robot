from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    bind_address = LaunchConfiguration('bind_address')
    public_host = LaunchConfiguration('public_host')
    http_port = LaunchConfiguration('http_port')
    stream_app = LaunchConfiguration('stream_app')
    stream_name = LaunchConfiguration('stream_name')
    hub_id = LaunchConfiguration('hub_id')
    camera_device_id = LaunchConfiguration('camera_device_id')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps = LaunchConfiguration('fps')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    poll_interval_ms = LaunchConfiguration('poll_interval_ms')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('public_host', default_value=''),
        DeclareLaunchArgument('http_port', default_value='18080'),
        DeclareLaunchArgument('stream_app', default_value='live'),
        DeclareLaunchArgument('stream_name', default_value='robot001'),
        DeclareLaunchArgument('hub_id', default_value='910007'),
        DeclareLaunchArgument('camera_device_id', default_value='robot-camera-1'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15.0'),
        DeclareLaunchArgument('jpeg_quality', default_value='65'),
        DeclareLaunchArgument('poll_interval_ms', default_value='90'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        Node(
            package='cloud_video_pusher',
            executable='local_hls_image_server',
            name='local_hls_image_server',
            output='screen',
            parameters=[{
                'image_topic': ParameterValue(image_topic, value_type=str),
                'bind_address': ParameterValue(bind_address, value_type=str),
                'public_host': ParameterValue(public_host, value_type=str),
                'http_port': ParameterValue(http_port, value_type=int),
                'stream_app': ParameterValue(stream_app, value_type=str),
                'stream_name': ParameterValue(stream_name, value_type=str),
                'hub_id': ParameterValue(hub_id, value_type=str),
                'camera_device_id': ParameterValue(camera_device_id, value_type=str),
                'width': ParameterValue(width, value_type=int),
                'height': ParameterValue(height, value_type=int),
                'fps': ParameterValue(fps, value_type=float),
                'jpeg_quality': ParameterValue(jpeg_quality, value_type=int),
                'poll_interval_ms': ParameterValue(poll_interval_ms, value_type=int),
                'auto_start': ParameterValue(auto_start, value_type=bool),
            }],
        ),
    ])
