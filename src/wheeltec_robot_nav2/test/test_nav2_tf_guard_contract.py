import ast
from pathlib import Path
import re


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _keyword(call, name):
    return next(
        (keyword.value for keyword in call.keywords if keyword.arg == name),
        None,
    )


def _string_constant(node):
    return node.value if isinstance(node, ast.Constant) else None


def test_four_wheel_diff_amcl_never_publishes_map_to_odom():
    params = (
        PACKAGE_ROOT
        / 'param'
        / 'wheeltec_params'
        / 'param_four_wheel_diff.yaml'
    ).read_text(encoding='utf-8')

    amcl_match = re.search(
        r'^amcl:\s*\n(?P<body>.*?)(?=^[A-Za-z_][A-Za-z0-9_]*:\s*$)',
        params,
        flags=re.MULTILINE | re.DOTALL,
    )
    assert amcl_match is not None
    assert re.search(
        r'^\s{4}tf_broadcast:\s*false\s*(?:#.*)?$',
        amcl_match.group('body'),
        flags=re.MULTILINE,
    )


def test_four_wheel_diff_enables_amcl_tf_guard_by_default():
    launch_path = PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py'
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))

    guard_default = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == 'guard_default'
            for target in node.targets
        )
    )
    assert isinstance(guard_default, ast.IfExp)
    assert _string_constant(guard_default.body) == 'true'
    assert _string_constant(guard_default.orelse) == 'false'
    assert isinstance(guard_default.test, ast.Compare)
    assert isinstance(guard_default.test.left, ast.Name)
    assert guard_default.test.left.id == 'car_mode'
    assert [_string_constant(item) for item in guard_default.test.comparators] == [
        'four_wheel_diff'
    ]

    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    declaration = next(
        call
        for call in calls
        if isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
        and call.args
        and _string_constant(call.args[0]) == 'enable_amcl_tf_guard'
    )
    assert isinstance(_keyword(declaration, 'default_value'), ast.Name)
    assert _keyword(declaration, 'default_value').id == 'guard_default'

    guard_node = next(
        call
        for call in calls
        if isinstance(call.func, ast.Name)
        and call.func.id == 'Node'
        and _string_constant(_keyword(call, 'name')) == 'amcl_tf_guard'
    )
    assert _string_constant(_keyword(guard_node, 'package')) == 'wheeltec_nav2'
    assert _string_constant(_keyword(guard_node, 'executable')) == 'amcl_tf_guard'
    condition = _keyword(guard_node, 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == 'enable_amcl_tf_guard'
