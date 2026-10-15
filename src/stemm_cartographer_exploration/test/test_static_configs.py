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
    assert 'from nav_msgs.srv import GetMap' in guard_text
    assert "'rrt_map_service', '/stemm_cartographer/rrt_dynamic_map'" in guard_text
    assert 'self.map_ready' in guard_text
    assert 'GetMap.Request()' in guard_text


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
    assert "'rrt_map_service': '/stemm_cartographer/rrt_dynamic_map'" in launch_text


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
    assert 'wait_for_units_stopped' in guard
    assert 'stop_units()' in guard
    assert 'systemctl kill --kill-who=all --signal=KILL "${unit}"' in guard
    assert 'stopping feature service: ${unit}' in guard
    assert 'stopped feature service: ${unit}' in guard
    assert 'restarting feature service: ${unit}' in guard
    assert 'restarted feature service: ${unit}' in guard
    assert 'restart_status=0' in guard
    assert 'restart_status=1' in guard
    assert guard.index('if ! request_recharge_stop; then') < guard.index(
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

def test_return_home_tolerance_exceeds_nav2_goal_checker():
    """2026-10-12: 返航位置容差必须严格大于 Nav2 的 xy_goal_tolerance。

    两者曾都等于 0.15 —— 零余量。Nav2 判"到达"时停在 0.18m，而返航判据要求
    ≤0.15m，差 3.6cm 永远无法满足，实测导致
    'return-home is outside the position-only tolerance: 0.186m > 0.150m'
    无限刷屏并把 returnHomeRetry 顶到上限。
    """
    import re

    manager_text = (
        ROOT / 'stemm_cartographer_exploration' / 'cartographer_manager.py'
    ).read_text(encoding='utf-8')
    m = re.search(
        r"'return_home_xy_completion_tolerance_m',\s*([0-9.]+)\)", manager_text)
    assert m, '未找到 return_home_xy_completion_tolerance_m 的声明'
    return_home_tol = float(m.group(1))

    params = yaml.safe_load(
        (ROOT / 'config' / 'stemm_safe_nav2_params.yaml').read_text(
            encoding='utf-8'))
    nav2_tol = float(
        params['controller_server']['ros__parameters']['goal_checker'][
            'xy_goal_tolerance'])

    assert nav2_tol == 0.15, 'Nav2 容差变了，需重新评估本关系: %r' % nav2_tol
    assert return_home_tol > nav2_tol, (
        '返航容差 %.3f 必须严格大于 Nav2 的 %.3f，否则 Nav2 判到达后返航判据'
        '永远无法满足' % (return_home_tol, nav2_tol))
    assert return_home_tol >= nav2_tol + 0.05, (
        '余量不足（%.3f - %.3f = %.3f < 0.05），停车抖动会破坏判据'
        % (return_home_tol, nav2_tol, return_home_tol - nav2_tol))


def test_escape_return_home_heading_tolerance_is_not_five_degrees():
    """2026-10-12: escape 的返航朝向容差不能是 5°。

    系统已声明 return_home_completion_requires_orientation=False（只按位置判定
    完成），5° 的朝向要求会卡住返航收尾：实测
    'footprint clear but heading offset 35 deg > 5 deg; continuing'，
    转满 1.05rad 预算仍差 16-22° 后 rotation_budget_exhausted。
    """
    import re

    text = (ROOT / 'stemm_cartographer_exploration'
            / 'escape_controller.py').read_text(encoding='utf-8')
    m = re.search(
        r"'return_home_heading_tolerance_rad',\s*([0-9.]+)\)", text)
    assert m, '未找到 return_home_heading_tolerance_rad 的声明'
    tol = float(m.group(1))
    assert tol > 0.087 + 1e-9, (
        '返航朝向容差仍为 %.3f rad（≤5°），会与"位置完成"的设计矛盾' % tol)
    assert tol <= 0.60, '返航朝向容差 %.3f 过大（>34°），失去"基本维持朝向"的意义' % tol

def test_escape_single_intervention_rotates_enough():
    """2026-10-12: escape 单次介入必须真正改变几何（不再是 7°）。

    原 min_steps_before_exit=5 × rotate_speed_radps=0.25 × command_period_sec=0.1
    = 0.125 rad ≈ 7.2°，转完 footprint 仍在内切膨胀环内，释放后立刻又满足
    楔死条件 → 表现为"高频自转"，8 次触发里 4 次落在同一格。
    """
    import re

    text = (ROOT / 'stemm_cartographer_exploration'
            / 'escape_controller.py').read_text(encoding='utf-8')

    def val(name):
        m = re.search(r"'%s',\s*([0-9.]+)\)" % name, text)
        assert m, '未找到参数 %s' % name
        return float(m.group(1))

    steps = val('min_steps_before_exit')
    speed = val('rotate_speed_radps')
    period = val('command_period_sec')
    per_arm = steps * speed * period

    assert per_arm >= 0.35, (
        '单次介入只转 %.3f rad（%.0f°），太小，会在原地反复抓同一处楔死'
        % (per_arm, per_arm * 57.2958))
    assert per_arm <= 1.20, (
        '单次介入 %.3f rad 过大，可能一次转过头' % per_arm)
    # 单次不得直接吃掉返航预算
    home_budget = val('return_home_max_rotation_rad')
    assert per_arm < home_budget, (
        '单次介入 %.2f rad 不应超过返航预算 %.2f rad' % (per_arm, home_budget))

def test_exploration_connectivity_gate_is_wired():
    """2026-10-12: 探索目标必须有连通性闸门，且默认开启、可现场关闭。

    原选点只验证「目标格已知自由 + 圆盘净空 + 直线阻塞比」，从不检查
    「从当前位置到目标是否连通」。实测 14:02：4 个探索目标里 2 个耗满 50s
    硬上限（结束时仍剩 4.5m/8.8m），机器人 6 个位置全在原点 0.7m 内。
    """
    import re

    text = (ROOT / 'stemm_cartographer_exploration'
            / 'cartographer_manager.py').read_text(encoding='utf-8')

    m = re.search(
        r"declare_parameter\('exploration_require_connected_route',\s*(True|False)\)",
        text)
    assert m, '未找到 exploration_require_connected_route 参数'
    assert m.group(1) == 'True', '该闸门必须默认开启'

    assert 'def _reachable_free_cells(self):' in text, '缺少可达域计算'
    assert 'def _explore_target_is_connected(self, pose):' in text, '缺少三态判定'
    # 必须在派发关口拒绝，而不是只在选点时过滤
    assert 'exploration goal rejected: target' in text, '缺少拒发日志'
    assert 'self.failed_goals.append((' in text, '拒绝时未拉黑（会反复选同一死角）'
    # 无法判定必须放行
    assert 'return None' in text


def test_exploration_connectivity_gate_never_blocks_on_unknown():
    """闸门必须是"仅可证明不同域时才拒绝"——None/异常一律放行。

    这是安全前提：判定器任何不确定都必须放行，绝不能阻断探索。
    """
    text = (ROOT / 'stemm_cartographer_exploration'
            / 'cartographer_manager.py').read_text(encoding='utf-8')

    # 取 _explore_target_is_connected 的函数体
    start = text.index('def _explore_target_is_connected(self, pose):')
    end = text.index('def _home_route_connectivity(self, pose):')
    body = text[start:end]

    # 只有 `return goal_cell in cells` 这一条返回布尔，其余全是 None
    bool_returns = [ln.strip() for ln in body.splitlines()
                    if ln.strip().startswith('return ') and 'None' not in ln]
    assert bool_returns == ['return goal_cell in cells'], (
        '三态判定只应有唯一布尔出口，实际: %r' % bool_returns)
    # 有异常兜底且返回 None
    assert 'except Exception:' in body, '缺少异常兜底'
    assert 'return None' in body, '异常路径未返回 None（放行）'

def test_localization_adaptor_has_bounded_startup_grace():
    """2026-10-12 N-8: adaptor 必须有"有界启动宽限期"，否则 interlock 死锁。

    adaptor 通常早于有图导航栈启动，而 /amcl_odom_limiter/status 只在导航栈内
    产生。若启动瞬间就抢答"限幅器缺失 -> 不健康"，它会 latch
    interlock.ready=false，把导航栈启动本身挡死，形成死锁
    （实测 15:49:55 重启后 0.2s 即 latch，导航栈停在 starting）。

    本测试直接读外部 adaptor 源码（/home/hnh/stemm_localization_authority/），
    不在本包内 —— 找不到文件时跳过，避免在无该路径的环境误报。
    """
    import re
    from pathlib import Path

    p = Path('/home/hnh/stemm_localization_authority/contract_adaptor.py')
    if not p.is_file():
        import pytest
        pytest.skip('localization adaptor not present on this host')
    text = p.read_text(encoding='utf-8')

    m = re.search(r"param\('startup_grace_sec',\s*([0-9.]+)\)", text)
    assert m, '未找到 startup_grace_sec 参数'
    grace = float(m.group(1))
    assert grace >= 30.0, '宽限期仅 %.0fs，导航栈来不及起来' % grace
    assert grace <= 600.0, '宽限期 %.0fs 过长，会掩盖长期缺失的限幅器' % grace

    # N-8b: 宽限期不得按墙钟收回，必须持续到限幅器首次上报为止
    # （90s 计时会在 4~5 分钟的建图会话结束前过期，重新把 auto-nav 锁死）
    assert "return True, 'startup_grace(waited %.0fs)' % waited" in text, \
        '宽限期仍在按时间收回'
    assert 'if waited >= self.startup_grace_sec:' in text, \
        '缺少"仅提示、不收回"的实现'
    assert 'self._grace_last_warn' in text, '缺少周期性提醒'
    assert 'grace_left' not in text, '仍残留按时间收回的旧逻辑'
    # 宽限期外的保守行为必须保留（不得顺手删掉 fail-safe）
    assert "return False, 'limiter_status_missing'" in text
    # 收到过状态后宽限期即失效：严格判据必须还在
    assert "'limiter_status_stale(%.2fs)'" in text
    assert "'no_amcl_pose_yet'" in text
    assert "'correction_lag(%.3fm)'" in text

def test_spin_ahead_sweep_is_small_enough_to_recover():
    """2026-10-12 N-10b: Spin 的前瞻扫掠必须足够小，否则自转恢复永远被否。

    spin.cpp: max_cycle_count = cycle_frequency * simulate_ahead_time，每步 0.1rad；
    任一位姿 footprint 代价 >= 254(LETHAL) 即 "Collision Ahead - Exiting Spin"。

    实测 16:26 轮自转恢复 5/5 失败；而 BT 恢复树
    （stemm_no_backup_nav_to_pose.xml，因后向 130° 盲区刻意不含 BackUp）
    里只有 Spin 能改变位姿，于是 6 次重试全在"清代价地图 + 转不动"之间空转，
    目标耗满 50s 硬上限被取消 —— 6 个目标 0 个到达。
    """
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    text = (root / 'config' / 'stemm_safe_nav2_params.yaml').read_text(
        encoding='utf-8')
    m = re.search(r'^\s*simulate_ahead_time:\s*([0-9.]+)', text, re.M)
    assert m, '未找到 behavior_server 的 simulate_ahead_time'
    v = float(m.group(1))
    # cycle_frequency 为 10.0，故扫掠弧度 = 10 * v * 0.1 = v
    sweep_deg = v * 57.2958
    assert v <= 1.0, \
        'simulate_ahead_time=%.2f -> 前瞻扫掠 %.0f°，过大：只要该弧内出现一个 ' \
        '254 就会否决整个自转恢复（实测 5/5 失败）' % (v, sweep_deg)
    assert v >= 0.1, \
        'simulate_ahead_time=%.2f 过小，等于放弃前瞻碰撞检查' % v

    # 不得顺手改掉转速相关参数（改动面最小化）
    for key, expect in (('max_rotational_vel', 0.3),
                        ('min_rotational_vel', 0.05),
                        ('rotational_acc_lim', 0.3)):
        mm = re.search(r'^\s*%s:\s*([0-9.]+)' % key, text, re.M)
        assert mm, '未找到 %s' % key
        assert abs(float(mm.group(1)) - expect) < 1e-9, \
            '%s 被改成 %s，本次不应改动转速参数' % (key, mm.group(1))

def test_behavior_tree_delegates_rotation_recovery_to_manager():
    """2026-10-12 N-10c: BT 恢复树不得再自带 Spin。

    实测 16:52 轮：behavior_server 收到 12 次 spin、全部 Collision Ahead 失败；
    其中 `Turning 1.57` 10 次来自 BT 的 <Spin spin_dist="1.57"/>，
    只有 1 次是 manager 的 target_yaw=±1.05。即 83% 的失败来自 BT 这条固定自转。

    而该 BT 文件自己的注释写明：
      "Motion recovery is handled by stemm_nav2_manager because the 2D lidar has
       a rear blind zone."
    manager 那条（_start_stall_spin_recovery）步长更小(1.05rad)、会交替方向、
    且有自研 253 预检 —— 与注释的设计意图一致。
    """
    import xml.etree.ElementTree as ET
    from pathlib import Path

    root = Path(__file__).resolve().parents[2] / 'stemm_nav2_manager' / 'config'
    f = root / 'stemm_no_backup_nav_to_pose.xml'
    assert f.is_file(), '未找到 %s' % f
    tree = ET.parse(str(f))
    tags = [el.tag for el in tree.iter()]
    assert 'Spin' not in tags, \
        'BT 恢复树仍含 Spin 节点：实测它的固定 1.57rad 自转 100%% 被 ' \
        'behavior_server 以 Collision Ahead 否决，且占全部 spin 失败的 83%%'
    # 清代价地图与 Wait 必须保留（恢复树不能只剩空转）
    assert any(t == 'ClearEntireCostmap' for t in tags), '恢复树丢了清代价地图'
    assert any(t == 'Wait' for t in tags), '恢复树丢了 Wait'
    # 顶部重试次数不得被顺手改掉
    text = f.read_text(encoding='utf-8')
    assert 'number_of_retries="4"' in text, '顶部 number_of_retries 被改动'

    # manager 侧必须仍然具备自转恢复能力（否则位姿恢复无人负责）
    mgr = (Path(__file__).resolve().parents[1]
           / 'stemm_cartographer_exploration' / 'cartographer_manager.py')
    mtext = mgr.read_text(encoding='utf-8')
    assert 'def _start_stall_spin_recovery' in mtext, \
        'manager 的自转恢复被删了，而 BT 也不再负责 —— 位姿恢复将无人负责'
    assert 'from nav2_msgs.action import NavigateToPose, Spin' in mtext or \
        'NavigateToPose, Spin' in mtext, 'manager 未导入 Spin action'

