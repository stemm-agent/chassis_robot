#!/usr/bin/env python3
import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = PACKAGE_DIR / 'scripts'
if not SCRIPTS_DIR.is_dir():
    # Local patch staging keeps the two scripts directly beside package.xml.
    SCRIPTS_DIR = PACKAGE_DIR
MONITOR_PATH = SCRIPTS_DIR / 'nav2_start_feedback_monitor'
BRIDGE_PATH = SCRIPTS_DIR / 'voice_recharge_bridge'
MANIFEST_PATH = PACKAGE_DIR / 'package.xml'


def read_source(path):
    return path.read_text(encoding='utf-8')


def method_source(path, class_name, method_name):
    source = read_source(path)
    tree = ast.parse(source, filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            for item in node.body:
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    if item.name == method_name:
                        return ast.get_source_segment(source, item)
    raise AssertionError('%s.%s not found in %s' % (
        class_name, method_name, path))


class VoiceNavigationHealthContractTest(unittest.TestCase):
    def test_scripts_parse_as_python(self):
        for path in (MONITOR_PATH, BRIDGE_PATH):
            source = read_source(path)
            compile(source, str(path), 'exec')

    def test_start_feedback_is_guarded_by_lifecycle_guard_interlock_and_tf(self):
        source = read_source(MONITOR_PATH)
        collect = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor',
            '_collect_health_blockers')
        check = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor', '_check_nav2')

        self.assertIn('GetState', source)
        self.assertIn('State.PRIMARY_STATE_ACTIVE', source)
        for node_name in (
            'map_server',
            'amcl',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'waypoint_follower',
        ):
            self.assertIn("'%s'" % node_name, source)

        self.assertIn("'/amcl_tf_guard/anomaly'", source)
        self.assertIn("'/nav2_guard_interlock/ready'", source)
        self.assertIn("'/nav2_guard_interlock/blocked'", source)
        self.assertGreaterEqual(
            source.count('DurabilityPolicy.TRANSIENT_LOCAL'), 1)
        self.assertIn('_guard_received_at < self._watch_started_at', collect)
        self.assertIn('_interlock_blockers(now)', collect)
        self.assertIn('_tf_chain_ready(now)', collect)
        self.assertIn('server_is_ready()', collect)

        self.assertIn('self.health_stable_sec', check)
        self.assertIn('self.health_stable_checks', check)
        self.assertIn(
            'self._publish_feedback(self._select_started_audio())', check)
        voice_callback = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor',
            '_voice_words_callback')
        self.assertNotIn('_select_started_audio', voice_callback)
        self.assertNotIn('_publish_feedback', voice_callback)

    def test_interlock_status_is_fresh_and_fail_closed(self):
        source = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor', '_interlock_blockers')
        self.assertIn('count_publishers', source)
        self.assertIn(
            '_interlock_ready_received_at < self._watch_started_at', source)
        self.assertIn(
            '_interlock_blocked_received_at < self._watch_started_at', source)
        self.assertIn('self.interlock_max_age_sec', source)
        self.assertIn('self._interlock_ready is not True', source)
        self.assertIn('self._interlock_blocked is not False', source)
        self.assertIn('publisher unavailable', source)

    def test_tf_gate_checks_both_edges_freshness_and_continuity(self):
        source = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor', '_tf_chain_ready')
        discontinuity = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor', '_tf_discontinuity')
        self.assertIn('self.global_frame, self.odom_frame', source)
        self.assertIn('self.odom_frame, self.base_frame', source)
        self.assertIn('self.tf_max_age_sec', source)
        self.assertIn('self.tf_future_tolerance_sec', source)
        self.assertIn('self.tf_continuity_hold_sec', source)
        self.assertIn('self.tf_continuity_min_samples', source)
        self.assertIn('timestamp regressed', discontinuity)
        self.assertIn('sample gap', discontinuity)
        self.assertIn('self.base_max_linear_speed * dt', discontinuity)
        self.assertIn('self.base_max_angular_speed * dt', discontinuity)

    def test_seed_accept_timeout_gives_amcl_time_to_converge(self):
        """2026-10-12 N-4a: AMCL 播种接受超时必须给激光匹配留够收敛时间。

        原值 3.0s：同一场会话里 1.3s 收敛成功、6.7s 未收敛都出现过。
        一旦超时即以 sticky 的 refused 发布，bridge 随即扣住自动拉起导航栈，
        而 bridge 的重试入口要求会话仍在跑（会话往往已 closed）——
        结果导航栈一直起不来（2026-10-12 14:41 实测）。
        """
        import re

        source = read_source(BRIDGE_PATH)
        m = re.search(
            r"declare_parameter\('seed_accept_timeout_s',\s*([0-9.]+)\)",
            source)
        self.assertIsNotNone(m, '未找到 seed_accept_timeout_s 声明')
        timeout_s = float(m.group(1))
        self.assertGreaterEqual(
            timeout_s, 8.0,
            '播种接受超时仅 %.1fs，对 AMCL 激光匹配收敛太紧' % timeout_s)
        self.assertLessEqual(
            timeout_s, 30.0,
            '播种接受超时 %.1fs 过长，会明显拖慢返航后的导航恢复' % timeout_s)

        # 收敛判据不得被放宽（否则等于牺牲定位正确性）
        m2 = re.search(
            r"declare_parameter\('seed_accept_max_error_m',\s*([0-9.]+)\)",
            source)
        self.assertIsNotNone(m2, '未找到 seed_accept_max_error_m 声明')
        self.assertLessEqual(float(m2.group(1)), 0.50 + 1e-9,
                             '收敛判据被放宽，等于牺牲定位正确性')

        # sticky 拒绝语义与全局定位补救路径必须保留
        self.assertIn("'refused', 'seed_acceptance_timeout'", source)
        self.assertIn('sticky=True', source)
        self.assertIn('_request_global_localization()', source)

    def test_seed_dock_evidence_stops_applying_when_far_from_dock(self):
        """2026-10-12 N-4b: 离桩超过阈值后，桩证据不得再否决播种。

        实测 15:15：auto-nav 已正确拉起 wheeltec_nav2（amcl/map_server/limiter
        都在跑），但 voice_recharge_bridge 以
          'dock/charging evidence says the robot is on the charger
           (within 0.97m of the recharge point) but the candidate seed
           (-1.01, 0.70) is 1.23m away'
        拒绝播种，导航栈永久停在 starting（AMCL 无初值 -> 无 map->odom）。

        该判据的 limit = seed_dock_max_distance_m + max(dock_offset_distance_m,
        undock_min_travel_m) 只覆盖"机器人还贴在桩上"；离桩 1m 以上时桩证据
        本就不适用（建图后返航回原点的场景很常见）。
        """
        import re

        text = read_source(BRIDGE_PATH)
        m = re.search(
            r"declare_parameter\('seed_dock_evidence_max_away_m',\s*([0-9.]+)\)",
            text)
        self.assertIsNotNone(m, '未找到 seed_dock_evidence_max_away_m 声明')
        away_m = float(m.group(1))
        self.assertGreaterEqual(
            away_m, 1.5,
            '离桩阈值仅 %.1fm，仍会误否决正常的播种候选' % away_m)
        self.assertLessEqual(
            away_m, 10.0,
            '离桩阈值 %.1fm 过大，等于放弃桩一致性检查' % away_m)

        # 阈值内必须保留原判据与原文案（不得顺手删掉）
        self.assertIn('if away <= self.seed_dock_evidence_max_away_m:', text)
        self.assertIn('dock/charging evidence says the robot is on the ', text)
        self.assertIn('within %.2fm of the recharge point', text)

        # undock_min_travel_m 必须保持原值：它同时是脱桩检测门限（688 行）
        # 与 dock→origin 行程还原判据（1243 行），抬高副作用更大。
        m2 = re.search(
            r"declare_parameter\('undock_min_travel_m',\s*([0-9.]+)\)", text)
        self.assertIsNotNone(m2, '未找到 undock_min_travel_m 声明')
        self.assertLessEqual(
            float(m2.group(1)), 0.20 + 1e-9,
            'undock_min_travel_m 被抬高到 %s，会破坏脱桩检测与行程还原'
            % m2.group(1))


    def test_timeout_reports_blockers_and_failure_feedback(self):
        source = method_source(
            MONITOR_PATH, 'Nav2StartFeedbackMonitor', '_check_nav2')
        self.assertIn("'timeout after %.1fs: %s'", source)
        self.assertIn("'; '.join(blockers)", source)
        self.assertIn('self._publish_status(detail)', source)
        self.assertIn('self._publish_failure_feedback()', source)

    def test_navigation_cancel_is_graceful_then_bounded_and_exact(self):
        cancel = method_source(
            BRIDGE_PATH, 'VoiceRechargeBridge', '_cancel_navigation')
        begin = method_source(
            BRIDGE_PATH, 'VoiceRechargeBridge',
            '_begin_navigation_shutdown')
        advance = method_source(
            BRIDGE_PATH, 'VoiceRechargeBridge',
            '_advance_navigation_shutdown')
        roots = method_source(
            BRIDGE_PATH, 'VoiceRechargeBridge',
            '_navigation_launch_roots')
        exact = method_source(
            BRIDGE_PATH, 'VoiceRechargeBridge',
            '_is_exact_navigation_launch_process')

        self.assertIn('self._begin_navigation_shutdown()', cancel)
        self.assertNotIn('_kill_named_process_tree', cancel)
        self.assertNotIn('_terminate_navigate_process', cancel)
        self.assertIn('ManageLifecycleNodes.Request.SHUTDOWN', begin)
        self.assertIn("'phase': 'lifecycle'", begin)
        self.assertIn('signal.SIGINT', advance)
        self.assertIn('signal.SIGTERM', advance)
        self.assertIn('signal.SIGKILL', advance)
        self.assertLess(advance.index("phase == 'lifecycle'"),
                        advance.index("phase == 'sigint'"))
        self.assertLess(advance.index("phase == 'sigint'"),
                        advance.index("phase == 'sigterm'"))
        self.assertNotIn('nav2_container', roots)
        self.assertIn('self.navigation_launch_package', exact)
        self.assertIn('self.navigation_launch_file', exact)
        self.assertIn("tokens[index] == 'launch'", exact)

    def test_manifest_declares_runtime_dependencies(self):
        root = ET.parse(MANIFEST_PATH).getroot()
        dependencies = {
            (element.tag, (element.text or '').strip())
            for element in root
            if element.tag in {
                'depend', 'exec_depend', 'build_depend', 'test_depend'
            }
        }
        self.assertIn(('exec_depend', 'lifecycle_msgs'), dependencies)
        self.assertIn(('exec_depend', 'std_msgs'), dependencies)
        self.assertTrue(any(
            name == 'nav2_msgs' and tag in {'depend', 'exec_depend'}
            for tag, name in dependencies
        ))
        self.assertTrue(any(
            name == 'tf2_ros' and tag in {'depend', 'exec_depend'}
            for tag, name in dependencies
        ))


if __name__ == '__main__':
    unittest.main()
