import os,yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml

def load_yaml(file_path: Path) -> dict:
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    cfg_params = load_yaml(os.path.join(get_package_share_directory('turn_on_wheeltec_robot'),'config','wheeltec_param.yaml'))
    car_mode = cfg_params['car_mode']
    print(f"car_mode:{car_mode}")
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_camera = LaunchConfiguration('launch_camera')
    launch_waypoint_cycle = LaunchConfiguration('launch_waypoint_cycle')
    
    my_nav_dir = get_package_share_directory('wheeltec_nav2')
    my_map_dir = os.path.join(my_nav_dir, 'map')
    my_map_file = 'WHEELTEC.yaml'
    
    rtabmap_nav_dir = get_package_share_directory('wheeltec_robot_rtab')
    my_param_dir = os.path.join(rtabmap_nav_dir, 'params')
    my_param_file = 'rtabmap_nav_params.yaml'
    
    wheeltec_bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
            condition=IfCondition(launch_lidar),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(wheeltec_bringup_dir, 'launch', 'wheeltec_camera.launch.py')),
            condition=IfCondition(launch_camera),
    )
    wheeltec_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rtabmap_nav_dir, 'launch', 'rtabmap_localization.launch.py')),
    )    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(my_map_dir, my_map_file),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('wheeltec_nav2'), 'param','wheeltec_params', f'param_{car_mode}.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_launch_lidar_cmd = DeclareLaunchArgument(
        'launch_lidar', default_value='true', choices=['true', 'false'],
        description='Launch physical lidar driver')

    declare_launch_camera_cmd = DeclareLaunchArgument(
        'launch_camera', default_value='false', choices=['true', 'false'],
        description='Launch camera driver')

    declare_launch_waypoint_cycle_cmd = DeclareLaunchArgument(
        'launch_waypoint_cycle', default_value='false', choices=['true', 'false'],
        description='Launch waypoint cycle helper')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization_launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])
    waypoint_cycle = Node(
            condition=IfCondition(launch_waypoint_cycle),
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    #wheeltec sensors
    ld.add_action(wheeltec_lidar)
    ld.add_action(wheeltec_camera)
    #wheeltec_localization
    #ld.add_action(wheeltec_localization)
    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_launch_lidar_cmd)
    ld.add_action(declare_launch_camera_cmd)
    ld.add_action(declare_launch_waypoint_cycle_cmd)
    ld.add_action(waypoint_cycle)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
