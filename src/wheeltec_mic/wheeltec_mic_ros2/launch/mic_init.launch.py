import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'use_mode',
        default_value='HIWONDER',
        description='选择使用的麦克风模块类型: HIWONDER, M2, NEW_M2, M07'
    )
    use_mode_ = LaunchConfiguration('use_mode')

    hiwonder_serial_arg = DeclareLaunchArgument(
        'hiwonder_serial_port',
        default_value='/dev/hiwonder_mic',
        description='Hiwonder 麦克风阵列控制串口'
    )
    hiwonder_audio_arg = DeclareLaunchArgument(
        'hiwonder_audio_device',
        default_value='hw:CARD=XFMDPV0018,DEV=0',
        description='Hiwonder 麦克风阵列 ALSA 录音设备'
    )
    hiwonder_serial_port = LaunchConfiguration('hiwonder_serial_port')
    hiwonder_audio_device = LaunchConfiguration('hiwonder_audio_device')
    
    # 获取功能包的共享目录
    pkg_share_dir = get_package_share_directory('wheeltec_mic_ros2')
    audio_save_path = os.path.join(pkg_share_dir, 'audio')
    command_config = os.path.join(pkg_share_dir, 'config', 'param.yaml')
    feedback_audio_path = os.path.join(pkg_share_dir, 'feedback_voice')
    resource_param = {"audio_path": feedback_audio_path}
    
    wheeltec_mic = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_mic",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "M2"']))
    )

    hiwonder_mic = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_mic",
        name="hiwonder_mic",
        output='screen',
        parameters=[{"usart_port_name": hiwonder_serial_port,
                    "serial_baud_rate": 115200,
                    "protocol_keepalive": True,
                    "keepalive_interval_ms": 200,
                    "serial_retry_interval_ms": 1000,
                    "serial_open_retries": 3,
                    "startup_mic_type": "mic6_circle"}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "HIWONDER"']))
    )

    wheeltec_m2n = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_m2n",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "virtual_usart_port_name": "/dev/wheeltec_mic_virtual",
                    "serial_baud_rate": 115200,
                    "audio_path": audio_save_path}], 
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "NEW_M2"']))
    )

    wheeltec_m07 = Node(
        package="wheeltec_mic_ros2",
        executable="wheeltec_m07",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}], 
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "M07"']))
    )

    voice_control = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        parameters=[{"appid": "3861bdbb",
                    "record_device_name": "hw:CARD=XFMDPV0018,DEV=0",
                    "pcm_gain": 1.0}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "M2"']))
    )

    voice_control_hiwonder = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        name="voice_control_hiwonder",
        output='screen',
        parameters=[{"appid": "3861bdbb",
                    "record_device_name": hiwonder_audio_device,
                    "device_type": use_mode_,
                    "pcm_gain": 1.0,
                    "audio_open_retries": 20,
                    "audio_open_retry_interval_ms": 1000}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "HIWONDER"']))
    )

    voice_control_m2n = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        parameters=[{"appid": "3861bdbb",
                    "record_device_name": "hw:CARD=L6Microphone,DEV=0",
                    "pcm_gain": 1.0}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "NEW_M2"']))
    )

    voice_control_m07 = Node(
        package="wheeltec_mic_ros2",
        executable="voice_control",
        output='screen',
        parameters=[{"appid": "3861bdbb",
                    "record_device_name": "hw:CARD=Device,DEV=0",
                    "device_type": use_mode_,
                    "pcm_gain": 1.0}],
        condition=IfCondition(PythonExpression(['"', use_mode_, '" == "M07"']))
    )

    call_recognition = Node(
        package="wheeltec_mic_ros2",
        executable="call_recognition",
        output='screen',
        parameters=[command_config]
    )

    command_recognition = Node(
        package="wheeltec_mic_ros2",
        executable="command_recognition",
        output='screen',
        parameters=[
            resource_param,
            command_config,
            {"device_type": use_mode_},
        ]
    )

    node_feedback = Node(
        package="wheeltec_mic_ros2",
        executable="node_feedback",
        output='screen',
        parameters=[
            resource_param,
            {"device_type": use_mode_},
        ]
    )

    # motion_control = Node(
    #     package="wheeltec_mic_ros2",
    #     executable="motion_control",
    #     output='screen',
    #     parameters=[
    #         resource_param,
    #         command_config,
    #         {"device_type": use_mode_},
    #     ]
    # )

    ld = LaunchDescription()
    ld.add_action(mode_arg)
    ld.add_action(hiwonder_serial_arg)
    ld.add_action(hiwonder_audio_arg)
    ld.add_action(hiwonder_mic)
    ld.add_action(wheeltec_mic)
    # ld.add_action(wheeltec_m2n)
    # ld.add_action(wheeltec_m07)
    ld.add_action(voice_control_hiwonder)
    ld.add_action(voice_control)
    # ld.add_action(voice_control_m2n)
    # ld.add_action(voice_control_m07)

    # ld.add_action(call_recognition)  
    # ld.add_action(command_recognition)
    
    # ld.add_action(node_feedback)
    # ld.add_action(motion_control)
    
    return ld