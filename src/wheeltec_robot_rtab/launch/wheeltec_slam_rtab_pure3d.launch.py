import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', )
    qos = LaunchConfiguration('qos')
    Localization = LaunchConfiguration('Localization')
    launch_rtabmapviz = LaunchConfiguration('launch_rtabmapviz')
    launch_robot = LaunchConfiguration('launch_robot')

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )

    # --- RTAB-Map dedicated visualization (rtabmapviz) ---
    rtabmap_viz_dir = get_package_share_directory('wheeltec_rviz2')
    wheeltec_rtabmapviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rtabmap_viz_dir, 'launch', 'wheeltec_rtabmapviz.launch.py')),
            condition=IfCondition(launch_rtabmapviz),
    )

    parameters={
          'frame_id':'camera_link', #wheeltec:camera_link
          'use_sim_time':use_sim_time,
          'wait_for_transform':2.0,
          'subscribe_rgbd':True,
          'subscribe_scan':False,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'False',
          'Rtabmap/DetectionRate':'1.0',
          'RGBD/LinearUpdate':'0.10',
          'RGBD/AngularUpdate':'0.10',
          'Kp/MaxFeatures':'400',
          'Vis/MaxFeatures':'500',
          'Grid/RangeMax':'5.0',
          'Mem/NotLinkedNodesKept':'false',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    
    remappings=[
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        wheeltec_camera,wheeltec_rtabmapviz,
        # Set env var to print messages to stdout immediately
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument('qos',default_value='2',description='General QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('Localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('launch_rtabmapviz', default_value='false', choices=['true', 'false'], description='Launch RTAB-Map visualization UI.'),
          DeclareLaunchArgument('launch_robot', default_value='false', choices=['true', 'false'], description='Launch base robot bringup from this SLAM launch.'),
        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.05, 'queue_size':5, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # Localization mode:
        Node(
            condition=IfCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),      
        # SLAM mode:
        Node(
            condition=UnlessCondition(Localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
           
    ])
