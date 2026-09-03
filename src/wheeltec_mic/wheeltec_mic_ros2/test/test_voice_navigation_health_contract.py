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
