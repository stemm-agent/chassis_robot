import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from stemm_cartographer_exploration.lifecycle_retry import LifecycleRetryState


ROOT = Path(__file__).resolve().parents[1]


def test_package_xml_is_well_formed():
    ET.parse(ROOT / 'package.xml')


def test_yaml_files_are_well_formed():
    for path in (ROOT / 'config').glob('*.yaml'):
        assert yaml.safe_load(path.read_text(encoding='utf-8')) is not None


def test_cartographer_nav2_keeps_global_navigation_policy_isolated():
    params = yaml.safe_load(
        (ROOT / 'config' / 'stemm_safe_nav2_params.yaml').read_text(
            encoding='utf-8'))
    bt_params = params['bt_navigator']['ros__parameters']
    controller = params['controller_server']['ros__parameters']
    progress = controller['progress_checker']
    global_costmap = params['global_costmap']['global_costmap'][
        'ros__parameters']

    assert bt_params['default_server_timeout'] == 200
    assert bt_params['default_nav_to_pose_bt_xml'].endswith(
        '/stemm_nav2_manager/config/stemm_no_backup_nav_to_pose.xml')
    assert 'failure_tolerance' not in controller
    assert progress['required_movement_radius'] == 0.05
    assert progress['movement_time_allowance'] == 10.0
    assert global_costmap['plugins'] == ['static_layer', 'inflation_layer']

    tree = ET.parse(
        ROOT / 'config' / 'stemm_cartographer_nav_to_pose.xml')
    recoveries = {
        node.attrib['name']: node
        for node in tree.findall('.//RecoveryNode')
    }
    assert set(recoveries) == {
        'CartographerNavigateRecovery', 'ComputePathToPose', 'FollowPath'}
    assert recoveries['CartographerNavigateRecovery'].attrib[
        'number_of_retries'] == '4'
    assert recoveries['ComputePathToPose'].attrib[
        'number_of_retries'] == '1'
    assert recoveries['FollowPath'].attrib['number_of_retries'] == '1'
    assert len(tree.findall('.//RoundRobin')) == 1
    assert len(tree.findall('.//Spin')) == 1
    assert len(tree.findall('.//Wait')) == 1
    assert not tree.findall('.//BackUp')

    manager_text = (
        ROOT / 'stemm_cartographer_exploration' / 'cartographer_manager.py'
    ).read_text(encoding='utf-8')
    assert (
        'goal.behavior_tree = self.exploration_behavior_tree'
        in manager_text)
    assert 'ActionClient(' in manager_text
    assert 'Spin, self.stall_spin_action_name' in manager_text
    assert 'recovery_twist.angular' not in manager_text
    assert '_spin_clearance_status' not in manager_text
    assert 'stall_spin_clearance_m' not in manager_text
    assert 'clear_entirely_local_costmap' not in manager_text
    assert 'clear_entirely_global_costmap' in manager_text


def test_launch_files_compile():
    for path in (ROOT / 'launch').glob('*.launch.py'):
        compile(path.read_text(encoding='utf-8'), str(path), 'exec')


def test_manager_starts_only_after_nav2_is_stably_active():
    launch_path = (
        ROOT / 'launch' / 'stemm_cartographer_auto_mapping.launch.py')
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))
    functions = {
        node.name: node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef)
    }

    before_ready_names = {
        node.id
        for node in ast.walk(functions['after_tf_guard'])
        if isinstance(node, ast.Name)
    }
    after_ready_names = {
        node.id
        for node in ast.walk(functions['after_nav2_ready'])
        if isinstance(node, ast.Name)
    }

    assert 'manager' not in before_ready_names
    assert {'manager', 'rrt_nodes'} <= after_ready_names

    guard_text = (
        ROOT / 'stemm_cartographer_exploration' / 'nav2_ready_guard.py'
    ).read_text(encoding='utf-8')
    assert "'/behavior_server'" in guard_text
    assert "ActionClient(self, Spin, '/spin')" in guard_text
    assert 'self.spin_action_client.server_is_ready()' in guard_text


def test_launch_uses_cartographer_specific_manager():
    launch_text = (
        ROOT / 'launch' / 'stemm_cartographer_auto_mapping.launch.py'
    ).read_text(encoding='utf-8')
    assert "package='stemm_cartographer_exploration'" in launch_text
    assert "executable='stemm_cartographer_nav2_manager'" in launch_text
    assert 'default_bt_xml_filename' not in launch_text
    assert "'exploration_no_progress_timeout_sec': 18.0" in launch_text
    assert "'exploration_pose_stall_timeout_sec': 7.0" in launch_text
    assert "'exploration_pose_stall_radius_m': 0.10" in launch_text
    assert (
        "'exploration_pose_stall_goal_tolerance_m': 0.30"
        in launch_text)
    assert "'rrt_fallback_wait_sec': 0.0" in launch_text
    assert "'rrt_projection_radius_m': 1.0" in launch_text
    assert "'-publish_period_sec', '2.0'" in launch_text
    assert "'feedback_baseline_settle_sec': 2.0" in launch_text
    assert "'stall_cancel_result_timeout_sec': 5.0" in launch_text
    assert "'stall_cancel_max_attempts': 2" in launch_text
    assert "'stall_confirmation_sec': 0.8" in launch_text
    assert "'stall_spin_action_name': '/spin'" in launch_text
    assert "'stall_spin_yaw_rad': 1.05" in launch_text
    assert "'stall_spin_result_timeout_sec': 6.0" in launch_text
    assert "'blocked_recovery_delay_sec': 1.5" in launch_text
    assert "'blocked_recovery_cooldown_sec': 5.0" in launch_text
    assert "'cancel_zero_guard_period_sec': 0.05" in launch_text


def test_launch_uses_independent_nonzero_odom_session_guard():
    launch_text = (
        ROOT / 'launch' / 'stemm_cartographer_auto_mapping.launch.py'
    ).read_text(encoding='utf-8')
    setup_text = (ROOT / 'setup.py').read_text(encoding='utf-8')
    guard_text = (
        ROOT / 'stemm_cartographer_exploration' / 'odom_session_guard.py'
    ).read_text(encoding='utf-8')

    assert "package='stemm_cartographer_exploration'" in launch_text
    assert "executable='stemm_cartographer_odom_session_guard'" in launch_text
    assert "'odom_topic': '/odom_combined'" in launch_text
    assert "'max_message_age_sec': 0.35" in launch_text
    assert "'max_tf_age_sec': 0.35" in launch_text
    assert "'stable_sec': 0.8" in launch_text
    assert "'linear_speed_mps': 0.02" in launch_text
    assert "'angular_speed_rps': 0.05" in launch_text
    assert "'pose_consistency_position_m': 0.03" in launch_text
    assert "'pose_consistency_yaw_rad': 0.0523598776" in launch_text
    assert "'odom_queue_depth': 120" in launch_text
    assert "'ready_confirmation_sec': 0.15" in launch_text
    assert "'ready_confirmation_messages': 3" in launch_text
    assert "executable='mapping_odom_reset'" not in launch_text
    assert 'after_odom_session_guard' in launch_text
    assert 'mapping_tf_guard' in launch_text
    assert 'stemm_cartographer_odom_session_guard' in setup_text
    assert 'Time.from_msg(message.header.stamp)' in guard_text
    assert 'timeout=Duration(seconds=0.0)' in guard_text
    assert 'timestamp-aligned odometry message/TF mismatch' in guard_text
    assert 'self.pending_odom = deque()' in guard_text
    assert 'self.window.reset_stability()' in guard_text
    assert 'publisher GID changed during startup' in guard_text
    assert 'latest fused odometry TF timestamp stopped advancing' in guard_text
    assert 'self.odom_stamp_repeated = True' in guard_text
    assert 'for _ in range(31)' in guard_text
    assert 'confirmed_on_later_cycle' in guard_text
    assert 'advancing_odom_messages' not in guard_text


def test_lifecycle_retry_times_out_and_recovers():
    retry = LifecycleRetryState(
        ('/bt_navigator',),
        request_timeout_sec=1.5,
        retry_delay_sec=0.5,
        retry_max_delay_sec=2.0,
    )

    retry.mark_started('/bt_navigator', 10.0)
    assert not retry.request_expired('/bt_navigator', 11.49)
    assert retry.request_expired('/bt_navigator', 11.5)
    assert retry.mark_failure('/bt_navigator', 11.5) == 0.5
    assert not retry.can_retry('/bt_navigator', 11.99)
    assert retry.can_retry('/bt_navigator', 12.0)

    retry.mark_started('/bt_navigator', 12.0)
    assert retry.mark_failure('/bt_navigator', 13.5) == 1.0
    retry.mark_started('/bt_navigator', 14.5)
    assert retry.mark_failure('/bt_navigator', 16.0) == 2.0
    retry.mark_started('/bt_navigator', 18.0)
    assert retry.mark_failure('/bt_navigator', 19.5) == 2.0

    retry.mark_success('/bt_navigator', 21.5)
    assert retry.failure_count['/bt_navigator'] == 0
    assert retry.can_retry('/bt_navigator', 21.5)


def test_recharge_service_drop_in_uses_safe_group_shutdown():
    drop_in = (
        ROOT / 'deploy' / 'wheeltec-recharge-manager-fast-stop.conf'
    ).read_text(encoding='utf-8')
    assert 'KillMode=control-group' in drop_in
    assert 'KillSignal=SIGINT' in drop_in
    assert 'TimeoutStopSec=' not in drop_in


def test_recharge_start_script_execs_the_real_node():
    start_script = (
        ROOT / 'deploy' / 'start_recharge_manager.sh'
    ).read_text(encoding='utf-8')
    assert '/install/auto_recharge_ros2/lib/' in start_script
    assert 'exec "${RECHARGE_EXECUTABLE}" "$@"' in start_script
    assert 'exec ros2 run' not in start_script


def test_service_guard_gracefully_cancels_recharge_before_stop():
    guard = (
        ROOT / 'deploy' / 'stemm-cartographer-service-guard'
    ).read_text(encoding='utf-8')
    assert 'stemm_cartographer_recharge_stop_guard' in guard
    assert 'runuser -u wheeltec' in guard
    assert 'timeout --signal=TERM 12s' in guard
    assert 'STOP_GRACE_ATTEMPTS=80' in guard
    assert 'STOP_KILL_ATTEMPTS=30' in guard
    assert 'systemctl stop --no-block "${unit}"' in guard
    assert 'systemctl kill --kill-who=all --signal=KILL "${unit}"' in guard
    assert 'stopping feature service: ${unit}' in guard
    assert 'stopped feature service: ${unit}' in guard
    assert 'restarting feature service: ${unit}' in guard
    assert 'restarted feature service: ${unit}' in guard
    assert 'restart_status=0' in guard
    assert 'restart_status=1' in guard
    assert guard.index('request_recharge_stop\n') < guard.index(
        'systemctl stop --no-block "${unit}"')


def test_preflight_rejects_orphaned_recharge_processes():
    preflight = (
        ROOT / 'stemm_cartographer_exploration' / 'preflight_guard.py'
    ).read_text(encoding='utf-8')
    assert "'recharge_manager'" in preflight
    assert "'recharge_manage'" in preflight
    assert "'low_battery_auto_recharge'" in preflight
    assert "'body_nav2_profile_guard'" in preflight
    assert "'body_identity_bridge'" in preflight


def test_rrt_uses_normalized_cartographer_map():
    params = yaml.safe_load(
        (ROOT / 'config' / 'exploration_params.yaml').read_text(
            encoding='utf-8'))
    for node_name in ('local_rrt', 'global_rrt', 'filter', 'assigner'):
        node = params[node_name]['ros__parameters']
        assert node['map_topic'] == '/stemm_cartographer/rrt_map'
    for node_name in ('local_rrt', 'global_rrt'):
        node = params[node_name]['ros__parameters']
        assert node['map_service'] == (
            '/stemm_cartographer/rrt_dynamic_map')

    launch_text = (
        ROOT / 'launch' / 'stemm_cartographer_auto_mapping.launch.py'
    ).read_text(encoding='utf-8')
    assert "'rrt_map_topic': '/stemm_cartographer/rrt_map'" in launch_text
    assert (
        "'rrt_map_service': '/stemm_cartographer/rrt_dynamic_map'"
        in launch_text)
    assert "'rrt_occupied_threshold': 50" in launch_text
