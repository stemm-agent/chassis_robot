import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_dir = get_package_share_directory('wheeltec_slam_toolbox')
    rrt_dir = get_package_share_directory('wheeltec_robot_rrt')
    manager_dir = get_package_share_directory('stemm_nav2_manager')
    astra_dir = get_package_share_directory('astra_camera')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        )
    )

    depth_camera = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(astra_dir, 'launch', 'astra.launch.xml')),
        launch_arguments={
            'camera_name': 'camera',
            'enable_color': 'false',
            'enable_ir': 'false',
            'enable_depth': 'true',
            'enable_point_cloud': 'true',
            'depth_width': '320',
            'depth_height': '240',
            'depth_fps': '10',
            'publish_tf': 'true',
            'tf_publish_rate': '10.0',
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rrt_dir, 'launch', 'navigation', 'nav2_bringup_launch.py')
        ),
        launch_arguments={
            'slam': 'True',
            'map': os.path.join(rrt_dir, 'map_data', 'image_map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(manager_dir, 'config', 'stemm_safe_nav2_params.yaml'),
            'default_bt_xml_filename': os.path.join(rrt_dir, 'behaviour_trees', 'navigate_w_replanning_time.xml'),
            'autostart': 'True',
        }.items(),
    )

    manager = Node(
        package='stemm_nav2_manager',
        executable='stemm_nav2_manager',
        name='stemm_nav2_manager',
        output='screen',
        parameters=[{
            'map_save_path': '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map',
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'auto_explore': False,
            'min_explore_distance': 0.6,
            'max_explore_distance': 1.6,
            'front_stop_distance': 0.40,
            'goal_clearance': 0.38,
            'front_emergency_distance': 0.30,
        }],
    )

    coverage = Node(
        package='stemm_coverage_explorer',
        executable='coverage_explorer',
        name='stemm_coverage_explorer',
        output='screen',
        parameters=[{
            'auto_start': True,
            'min_goal_distance': 0.45,
            'max_goal_distance': 3.0,
            'obstacle_clearance': 0.38,
            'known_clearance': 0.20,
            'corridor_clearance': 0.28,
            'frontier_radius': 0.7,
            'bad_goal_radius': 0.55,
            'blocked_place_radius': 0.55,
            'global_costmap_topic': '/global_costmap/costmap',
            'local_costmap_topic': '/local_costmap/costmap',
            'costmap_timeout_sec': 3.0,
            'min_frontier_cells': 8,
            'gain_scale': 1.0,
            'potential_scale': 1.2,
            'clearance_scale': 0.8,
            'lookahead_scale': 0.35,
            'finish_save_delay_sec': 90.0,
            'bad_ttl_sec': 240.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam,
        depth_camera,
        nav2,
        manager,
        coverage,
    ])
