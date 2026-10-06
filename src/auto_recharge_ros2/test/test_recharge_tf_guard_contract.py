import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'auto_recharge_ros2'
    / 'recharge_manager.py'
)
if not SOURCE.exists():
    # The local patch-staging layout keeps this contract beside the source.
    SOURCE = Path(__file__).with_name('recharge_manager.py')


def _class_node(tree, name):
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )


def _method(class_node, name):
    return next(
        node
        for node in class_node.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_recharge_manager_fails_closed_on_tf_guard_state():
    source = SOURCE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = _class_node(tree, 'RechargeManager')

    assert "TF_GUARD_ANOMALY_TOPIC = '/amcl_tf_guard/anomaly'" in source
    assert 'DurabilityPolicy.TRANSIENT_LOCAL' in source
    assert 'self.tf_guard_anomaly = True' in source
    assert 'self.tf_guard_state_received = False' in source

    start_source = ast.get_source_segment(source, _method(manager, '_start_callback'))
    tick_source = ast.get_source_segment(source, _method(manager, '_manager_tick'))
    callback_source = ast.get_source_segment(
        source, _method(manager, '_tf_guard_anomaly_callback')
    )

    assert 'if not self._tf_guard_is_safe()' in start_source
    assert 'response.success = False' in start_source
    assert 'if not self._tf_guard_is_safe()' in tick_source
    assert "self._cancel_for_safety('TF_GUARD')" in tick_source
    assert "self._cancel_for_safety('TF_GUARD')" in callback_source


def test_guard_safety_cancel_stops_motion_and_clears_task_state():
    source = SOURCE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = _class_node(tree, 'RechargeManager')
    cancel_source = ast.get_source_segment(
        source, _method(manager, '_cancel_for_safety')
    )

    assert 'self.Cancel_Current_Task()' in cancel_source
    assert 'self.Stop_Motion(' in cancel_source
    assert 'self.manager_active = False' in cancel_source
    assert 'self.waiting_for_nav2 = False' in cancel_source


def test_recharge_manager_requires_live_nav2_motion_interlock():
    source = SOURCE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    manager = _class_node(tree, 'RechargeManager')

    assert (
        "NAV2_INTERLOCK_BLOCKED_TOPIC = '/nav2_guard_interlock/blocked'"
        in source
    )
    assert 'self.nav2_interlock_blocked = True' in source
    assert 'self.nav2_interlock_state_received = False' in source

    start_source = ast.get_source_segment(source, _method(manager, '_start_callback'))
    begin_source = ast.get_source_segment(source, _method(manager, '_begin_recharge'))
    tick_source = ast.get_source_segment(source, _method(manager, '_manager_tick'))
    callback_source = ast.get_source_segment(
        source, _method(manager, '_nav2_interlock_callback')
    )

    assert 'if not self._nav2_interlock_is_safe()' in start_source
    assert 'response.success = False' in start_source
    assert 'if not self._nav2_interlock_is_safe()' in begin_source
    assert "self._cancel_for_safety('NAV2_INTERLOCK')" in begin_source
    assert 'if not self._nav2_interlock_is_safe()' in tick_source
    assert "self._cancel_for_safety('NAV2_INTERLOCK')" in tick_source
    assert "self._cancel_for_safety('NAV2_INTERLOCK')" in callback_source
