import ast
import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py'
SOURCE = LAUNCH_FILE.read_text(encoding='utf-8')
TREE = ast.parse(SOURCE)


def _function(name):
    for node in TREE.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f'missing function {name}')


def _load_classifier():
    classifier = _function('_classify_navigation_process')
    module = ast.Module(body=[classifier], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {'os': os}
    exec(compile(module, str(LAUNCH_FILE), 'exec'), namespace)
    return namespace['_classify_navigation_process']


def test_launch_claims_singleton_before_any_ros_or_package_setup():
    generate = _function('generate_launch_description')
    first = generate.body[0]
    assert isinstance(first, ast.Expr)
    assert isinstance(first.value, ast.Call)
    assert isinstance(first.value.func, ast.Name)
    assert first.value.func.id == '_claim_nav2_launch_singleton'


def test_lock_is_nonblocking_process_scoped_and_symlink_safe():
    claim_source = ast.get_source_segment(
        SOURCE, _function('_claim_nav2_launch_singleton'))
    assert 'fcntl.LOCK_EX | fcntl.LOCK_NB' in claim_source
    assert 'os.O_CLOEXEC' in claim_source
    assert "getattr(os, 'O_NOFOLLOW', 0)" in claim_source
    assert 'stat.S_ISREG' in claim_source
    assert 'lock_stat.st_uid != os.getuid()' in claim_source
    assert '_NAV2_SINGLETON_FD = fd' in claim_source
    assert '_find_stale_navigation_processes()' in claim_source


def test_lock_path_is_fixed_by_real_uid_not_caller_environment():
    path_function = _function('_nav2_lock_path')
    path_source = ast.get_source_segment(SOURCE, path_function)
    string_literals = {
        node.value for node in ast.walk(path_function)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert 'os.getuid()' in path_source
    assert "Path('/tmp')" in path_source
    assert 'XDG_RUNTIME_DIR' not in path_source
    assert '/run/user' not in string_literals


def test_cleanup_scan_covers_old_container_guard_interlock_and_launch():
    scan_source = ast.get_source_segment(
        SOURCE, _function('_find_stale_navigation_processes'))
    assert 'current_pid = os.getpid()' in scan_source
    assert 'int(proc_dir.name) == current_pid' in scan_source

    classifier = _load_classifier()

    assert classifier([
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
        '--ros-args', '-r', '__node:=nav2_container',
    ]) == 'nav2_container'
    assert classifier([
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
        '--ros-args', '-r', '__node:=localization_container',
    ]) == 'localization_container'
    assert classifier([
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
        '--ros-args', '-r', '__node:=navigation_container',
    ]) == 'navigation_container'
    assert classifier([
        '/home/wheeltec/install/wheeltec_nav2/lib/wheeltec_nav2/amcl_tf_guard',
    ]) == 'amcl_tf_guard'
    assert classifier([
        '/home/wheeltec/install/wheeltec_nav2/lib/wheeltec_nav2/nav2_guard_interlock',
    ]) == 'nav2_guard_interlock'
    assert classifier([
        '/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
        'wheeltec_nav2', 'wheeltec_nav2.launch.py',
    ]) == 'wheeltec_nav2_launch'


def test_classifier_ignores_diagnostic_text_and_unrelated_containers():
    classifier = _load_classifier()
    assert classifier([
        '/bin/bash', '-lc',
        'ps aux | grep "ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py"',
    ]) is None
    assert classifier([
        '/opt/ros/humble/lib/rclcpp_components/component_container_isolated',
        '--ros-args', '-r', '__node:=unrelated_container',
    ]) is None


def test_preflight_is_observational_and_never_kills_or_restarts_processes():
    assert "Path('/proc')" in SOURCE
    assert 'subprocess' not in SOURCE
    assert 'os.kill' not in SOURCE
    assert 'SIGTERM' not in SOURCE
    assert 'SIGKILL' not in SOURCE
    assert 'LoadComposableNodes' in SOURCE
