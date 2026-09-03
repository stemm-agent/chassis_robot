import ast
from pathlib import Path
import re
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _keyword(call, name):
    return next(
        (keyword.value for keyword in call.keywords if keyword.arg == name),
        None,
    )


def _constant(node):
    return node.value if isinstance(node, ast.Constant) else None


def _dict_value(node, key):
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'items'
    ):
        node = node.func.value
    assert isinstance(node, ast.Dict)
    return next(
        value
        for item_key, value in zip(node.keys, node.values)
        if _constant(item_key) == key
    )


def _call_named(tree, function_name):
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == function_name
    ]


def _node_named(tree, node_name):
    return next(
        call
        for call in _call_named(tree, 'Node')
        if _constant(_keyword(call, 'name')) == node_name
    )


def _include_for(tree, launch_filename):
    return next(
        call
        for call in _call_named(tree, 'IncludeLaunchDescription')
        if launch_filename
        in {
            item.value
            for item in ast.walk(call)
            if isinstance(item, ast.Constant) and isinstance(item.value, str)
        }
    )


def test_interlock_source_is_fail_closed_and_cancels_motion_actions():
    source = (PACKAGE_ROOT / 'src' / 'nav2_guard_interlock.cpp').read_text(
        encoding='utf-8'
    )

    assert 'bool blocked_{true};' in source
    assert 'bool guard_healthy_{false};' in source
    assert 'publish_blocked(true);' in source
    assert 'publish_zero_velocity();' in source
    assert 'create_wall_timer' in source

    assert 'rclcpp::KeepLast(1)' in source
    assert '.reliable().transient_local()' in source
    assert 'guard_message_timeout_sec' in source
    assert 'last_guard_receipt_' in source

    assert 'ManageLifecycleNodes::Request::STARTUP' in source
    assert 'ManageLifecycleNodes::Request::PAUSE' in source
    assert 'ManageLifecycleNodes::Request::RESUME' in source
    assert 'request_epoch == state_epoch_' in source
    assert 'pause_confirmed_ = false;' in source
    assert 'manager_request_started_at_' in source
    assert 'manager_request_sequence_counter_' in source
    assert 'active_manager_request_sequence_' in source
    assert 'remove_pending_request' in source

    expected_actions = {
        'navigate_to_pose',
        'navigate_through_poses',
        'follow_waypoints',
        'follow_path',
        'compute_path_to_pose',
        'compute_path_through_poses',
        'smooth_path',
        'spin',
        'backup',
        'drive_on_heading',
        'assisted_teleop',
        'wait',
    }
    constructed_actions = set(
        re.findall(r'make_action_client<[^>]+>\(\s*"([^"]+)"\s*\)', source)
    )
    assert expected_actions <= constructed_actions
    assert 'async_cancel_all_goals()' in source

    control_tick = source[source.index('void control_tick()'):
                          source.index('void try_manager_command')]
    assert 'publish_zero_velocity();' not in control_tick
    assert 'service_zero_burst(now);' in control_tick
    assert 'cancel_all_navigation_goals();' in control_tick
    assert control_tick.index('publish_blocked(blocked);') < control_tick.index(
        'if (!blocked)'
    )


def test_startup_waits_for_current_fresh_stable_guard_and_epoch():
    source = (PACKAGE_ROOT / 'src' / 'nav2_guard_interlock.cpp').read_text(
        encoding='utf-8'
    )
    control_tick = source[
        source.index('void control_tick()'):
        source.index('void try_manager_command')
    ]
    startup_choice = re.search(
        r'if \(!navigation_started_\)\s*\{(?P<body>.*?)\n\s*\} else if '
        r'\(!pause_confirmed_\)',
        control_tick,
        flags=re.DOTALL,
    )
    assert startup_choice is not None
    assert 'desired_command = ManagerCommand::kStartup;' in startup_choice.group('body')
    assert 'guard_healthy_' in startup_choice.group('body')
    assert 'guard_sample_received_' in startup_choice.group('body')
    assert 'have_guard_receipt_' in startup_choice.group('body')
    assert 'guard_message_timeout_sec_' in startup_choice.group('body')
    assert 'have_healthy_since_' in startup_choice.group('body')
    assert 'healthy_stability_sec_' in startup_choice.group('body')

    try_manager = source[
        source.index('void try_manager_command'):
        source.index('void on_manager_response')
    ]
    startup_eligibility = re.search(
        r'if \(command == ManagerCommand::kStartup &&(?P<body>.*?)\)\s*\{\s*return;',
        try_manager,
        flags=re.DOTALL,
    )
    assert startup_eligibility is not None
    assert 'navigation_started_' in startup_eligibility.group('body')
    assert 'guard_sample_received_' in startup_eligibility.group('body')
    assert 'guard_healthy_' in startup_eligibility.group('body')
    assert 'have_guard_receipt_' in startup_eligibility.group('body')
    assert 'guard_message_timeout_sec_' in startup_eligibility.group('body')
    assert 'have_healthy_since_' in startup_eligibility.group('body')
    assert 'healthy_stability_sec_' in startup_eligibility.group('body')

    graph_recheck = try_manager[try_manager.index('// Recheck after graph discovery'):]
    assert 'state_epoch_ != request_epoch' in graph_recheck
    assert 'guard_sample_received_' in graph_recheck
    assert 'guard_healthy_' in graph_recheck
    assert 'have_guard_receipt_' in graph_recheck
    assert 'guard_message_timeout_sec_' in graph_recheck

    response_start = source.index('void on_manager_response')
    manager_response = source[
        response_start:
        source.index('template<typename ActionT>', response_start)
    ]
    startup_response = manager_response[
        manager_response.index('if (command == ManagerCommand::kStartup) {'):
        manager_response.index('} else if (command == ManagerCommand::kPause) {')
    ]
    assert 'navigation_started_ = true;' in startup_response
    assert 'blocked_ = true;' in startup_response
    assert 'pause_confirmed_ = false;' in startup_response
    assert 'const bool guard_is_fresh' in startup_response
    assert 'const bool guard_is_stable' in startup_response
    assert 'success && blocked_' in startup_response
    assert '!activation_cleanup_pending_' in startup_response
    assert 'seconds_between(healthy_since_, now) >= healthy_stability_sec_' in (
        startup_response
    )
    assert 'request_epoch == state_epoch_' in startup_response
    assert 'if (can_skip_pause)' in startup_response
    assert 'activation_cleanup_pending_ = true;' in startup_response
    assert 'activation_cleanup_epoch_ = state_epoch_;' in startup_response
    assert 'activation_cleanup_started_at_ = now;' in startup_response
    assert 'activation_cleanup_pending_ = false;' in startup_response
    assert 'have_last_manager_attempt_ = false;' in startup_response
    assert 'startup_requires_pause = true;' in startup_response
    assert 'skipping' in manager_response
    assert 'immediately forcing PAUSE' in manager_response

    reconciliation = manager_response[
        manager_response.index('void reconcile_manager_state_after_timeout'):
    ]
    assert 'const bool safe_epoch = request_epoch == state_epoch_;' in reconciliation

    cleanup_watchdog = control_tick[
        control_tick.index('if (activation_cleanup_pending_)'):
        control_tick.index('blocked = blocked_;')
    ]
    assert '!guard_healthy_' in cleanup_watchdog
    assert 'activation_cleanup_epoch_ != state_epoch_' in cleanup_watchdog
    assert 'activation_cleanup_pending_ = false;' in cleanup_watchdog
    assert 'pause_confirmed_ = false;' in cleanup_watchdog


def test_nav2_velocity_is_remapped_and_hard_gated():
    source = (PACKAGE_ROOT / 'src' / 'nav2_guard_interlock.cpp').read_text(
        encoding='utf-8'
    )
    assert '"raw_cmd_vel_topic", "/nav2_cmd_vel"' in source
    assert 'create_subscription<geometry_msgs::msg::Twist>' in source
    velocity_callback = source[
        source.index('void on_nav2_velocity'):
        source.index('void control_tick()')
    ]
    assert 'if (!blocked_)' in velocity_callback
    assert 'zero_velocity_publisher_->publish(*message);' in velocity_callback
    assert 'publish_zero_velocity();' not in velocity_callback
    assert 'dropped silently' in velocity_callback

    bringup_path = PACKAGE_ROOT / 'launch' / 'bringup_launch.py'
    bringup_tree = ast.parse(bringup_path.read_text(encoding='utf-8'))
    remaps = {
        (_constant(_keyword(call, 'src')), _constant(_keyword(call, 'dst')))
        for call in _call_named(bringup_tree, 'SetRemap')
    }
    assert ('cmd_vel', '/nav2_cmd_vel') in remaps
    assert ('/cmd_vel', '/nav2_cmd_vel') in remaps
    for remap in _call_named(bringup_tree, 'SetRemap'):
        if _constant(_keyword(remap, 'dst')) != '/nav2_cmd_vel':
            continue
        condition = _keyword(remap, 'condition')
        assert isinstance(condition, ast.Call)
        assert isinstance(condition.func, ast.Name)
        assert condition.func.id == 'IfCondition'
        assert isinstance(condition.args[0], ast.Name)
        assert condition.args[0].id == 'enable_nav2_cmd_vel_gate'

    gate_declaration = next(
        call
        for call in _call_named(bringup_tree, 'DeclareLaunchArgument')
        if call.args and _constant(call.args[0]) == 'enable_nav2_cmd_vel_gate'
    )
    assert _constant(_keyword(gate_declaration, 'default_value')) == 'false'

    navigation_include = _include_for(bringup_tree, 'navigation_launch.py')
    containing_groups = [
        call
        for call in _call_named(bringup_tree, 'GroupAction')
        if navigation_include in set(ast.walk(call))
    ]
    assert containing_groups
    scoped_group = containing_groups[0]
    assert sum(
        1
        for item in ast.walk(scoped_group)
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == 'SetRemap'
    ) >= 2


def test_navigation_startup_is_owned_by_interlock_only_when_guard_is_enabled():
    main_launch_path = PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py'
    main_source = main_launch_path.read_text(encoding='utf-8')
    main_tree = ast.parse(main_source)

    interlock = _node_named(main_tree, 'nav2_guard_interlock')
    assert _constant(_keyword(interlock, 'package')) == 'wheeltec_nav2'
    assert _constant(_keyword(interlock, 'executable')) == 'nav2_guard_interlock'
    condition = _keyword(interlock, 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == 'enable_amcl_tf_guard'

    guard = _node_named(main_tree, 'amcl_tf_guard')
    guard_condition = _keyword(guard, 'condition')
    assert isinstance(guard_condition.args[0], ast.Name)
    assert guard_condition.args[0].id == 'enable_amcl_tf_guard'

    parameters = _keyword(interlock, 'parameters')
    assert isinstance(parameters, ast.List) and len(parameters.elts) == 1
    interlock_params = parameters.elts[0]
    assert _constant(_dict_value(interlock_params, 'manage_initial_startup')) is True
    assert _constant(_dict_value(interlock_params, 'healthy_stability_sec')) == 1.0
    assert _constant(_dict_value(interlock_params, 'post_activation_cancel_sec')) == 0.5
    assert _constant(_dict_value(interlock_params, 'anomaly_topic')) == (
        '/amcl_tf_guard/anomaly'
    )
    assert _constant(_dict_value(interlock_params, 'raw_cmd_vel_topic')) == (
        '/nav2_cmd_vel'
    )
    assert _constant(_dict_value(interlock_params, 'lifecycle_manager_service')) == (
        '/lifecycle_manager_navigation/manage_nodes'
    )

    guard_parameters = _keyword(guard, 'parameters')
    assert isinstance(guard_parameters, ast.List) and len(guard_parameters.elts) == 1
    guard_params = guard_parameters.elts[0]
    assert _constant(_dict_value(guard_params, 'nomotion_update_period_sec')) == 0.5
    assert _constant(_dict_value(guard_params, 'bootstrap_good_samples')) == 6
    assert _constant(_dict_value(
        guard_params, 'bootstrap_min_confirm_duration_sec'
    )) == 3.0
    assert _constant(_dict_value(
        guard_params, 'bootstrap_max_nomotion_requests'
    )) == 90
    assert _constant(_dict_value(guard_params, 'score_max_beams')) == 180

    navigation_autostart_assignment = next(
        node
        for node in ast.walk(main_tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == 'navigation_autostart'
            for target in node.targets
        )
    )
    expression = navigation_autostart_assignment.value
    assert isinstance(expression, ast.Call)
    assert isinstance(expression.func, ast.Name)
    assert expression.func.id == 'PythonExpression'
    expression_text = ''.join(
        item.value
        for item in ast.walk(expression)
        if isinstance(item, ast.Constant) and isinstance(item.value, str)
    )
    assert ".lower() != 'true'" in expression_text
    assert any(
        isinstance(item, ast.Name) and item.id == 'enable_amcl_tf_guard'
        for item in ast.walk(expression)
    )

    bringup_include = next(
        call
        for call in _call_named(main_tree, 'IncludeLaunchDescription')
        if isinstance(_keyword(call, 'launch_arguments'), ast.Call)
        and any(
            isinstance(item, ast.Constant) and item.value == 'navigation_autostart'
            for item in ast.walk(_keyword(call, 'launch_arguments'))
        )
    )
    launch_arguments_dict = _keyword(call=bringup_include, name='launch_arguments').func.value
    passed_autostart = _dict_value(launch_arguments_dict, 'navigation_autostart')
    assert isinstance(passed_autostart, ast.Name)
    assert passed_autostart.id == 'navigation_autostart'
    passed_velocity_gate = _dict_value(
        launch_arguments_dict, 'enable_nav2_cmd_vel_gate'
    )
    assert isinstance(passed_velocity_gate, ast.Name)
    assert passed_velocity_gate.id == 'enable_amcl_tf_guard'


def test_blocked_zero_burst_is_bounded_and_status_heartbeat_is_ten_hz():
    source = (PACKAGE_ROOT / 'src' / 'nav2_guard_interlock.cpp').read_text(
        encoding='utf-8'
    )

    # Final /cmd_vel zero publication has exactly one call site: the bounded
    # burst service. It must never be refreshed by raw Nav2 input, cancellation,
    # lifecycle responses, or every control tick.
    assert len(re.findall(
        r'^\s*publish_zero_velocity\(\);\s*$', source, flags=re.MULTILINE
    )) == 1
    burst_service = source[
        source.index('void service_zero_burst'):
        source.index('void publish_zero_velocity')
    ]
    assert 'now >= zero_burst_until_' in burst_service
    assert 'zero_burst_active_ = false;' in burst_service
    assert 'zero_burst_period_ms_' in burst_service
    assert 'publish_zero_velocity();' in burst_service

    anomaly_callback = source[
        source.index('void on_anomaly'):
        source.index('void on_nav2_velocity')
    ]
    assert 'if (newly_blocked)' in anomaly_callback
    assert 'start_zero_burst_locked(now);' in anomaly_callback
    assert 'service_zero_burst(now);' in anomaly_callback

    control_tick = source[
        source.index('void control_tick()'):
        source.index('void try_manager_command')
    ]
    assert 'start_zero_burst_locked(now);' in control_tick
    assert 'service_zero_burst(now);' in control_tick
    assert 'if (heartbeat_due)' in control_tick
    assert 'blocked_heartbeat_period_sec_' in control_tick
    assert 'zero_burst_active_ = false;' in control_tick

    manager_response = source[
        source.index('void on_manager_response'):
        source.index('static const char * manager_command_name')
    ]
    assert 'publish_zero_velocity();' not in manager_response

    assert '"blocked_zero_burst_sec", 0.20' in source
    assert '"zero_burst_period_ms", 20' in source
    assert '"blocked_heartbeat_hz", 10.0' in source
    assert 'rclcpp::KeepLast(1)' in source
    assert 'latched_qos.reliable().transient_local();' in source

    launch_tree = ast.parse(
        (PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py').read_text(
            encoding='utf-8'
        )
    )
    interlock = _node_named(launch_tree, 'nav2_guard_interlock')
    params = _keyword(interlock, 'parameters').elts[0]
    assert _constant(_dict_value(params, 'blocked_zero_burst_sec')) == 0.20
    assert _constant(_dict_value(params, 'zero_burst_period_ms')) == 20
    assert _constant(_dict_value(params, 'blocked_heartbeat_hz')) == 10.0


def test_manager_timeout_verifies_state_and_cannot_be_cleared_by_late_response():
    source = (PACKAGE_ROOT / 'src' / 'nav2_guard_interlock.cpp').read_text(
        encoding='utf-8'
    )

    assert '#include "lifecycle_msgs/srv/get_state.hpp"' in source
    assert '#include "lifecycle_msgs/msg/state.hpp"' in source
    for node_name in (
        '/controller_server',
        '/smoother_server',
        '/planner_server',
        '/behavior_server',
        '/bt_navigator',
        '/waypoint_follower',
    ):
        assert f'"{node_name}"' in source

    manager_send = source[
        source.index('void try_manager_command'):
        source.index('void on_manager_response')
    ]
    assert 'request_sequence = ++manager_request_sequence_counter_;' in manager_send
    assert 'manager_request_started_at_ = now;' in manager_send
    assert 'manager_request_epoch_ = request_epoch;' in manager_send
    assert 'future_and_request_id.request_id' in manager_send

    manager_response = source[
        source.index('void on_manager_response'):
        source.index('static const char * manager_command_name')
    ]
    assert 'active_manager_request_sequence_ != request_sequence' in manager_response
    assert 'Ignoring late Nav2 lifecycle' in manager_response

    request_timeout = source[
        source.index('void handle_manager_request_timeout'):
        source.index('void start_manager_state_verification')
    ]
    assert 'manager_request_in_flight_ = false;' in request_timeout
    assert 'active_manager_request_sequence_ = 0;' in request_timeout
    assert 'remove_pending_request(timed_out_request_id)' in request_timeout
    assert 'start_manager_state_verification' in request_timeout

    state_record = source[
        source.index('void record_manager_state_response'):
        source.index('void handle_manager_state_verification_timeout')
    ]
    assert 'PRIMARY_STATE_ACTIVE' in state_record
    assert 'PRIMARY_STATE_INACTIVE' in state_record
    assert 'active_manager_state_verification_sequence_' in state_record

    verification_timeout = source[
        source.index('void handle_manager_state_verification_timeout'):
        source.index('void reconcile_manager_state_after_timeout')
    ]
    assert 'manager_state_verification_in_flight_ = false;' in verification_timeout
    assert 'remove_pending_request(request_ids[index])' in verification_timeout

    reconciliation_start = source.index(
        'void reconcile_manager_state_after_timeout'
    )
    reconciliation = source[
        reconciliation_start:
        source.index('template<typename ActionT>', reconciliation_start)
    ]
    assert 'all_known && all_inactive' in reconciliation
    assert 'pause_confirmed_ = true;' in reconciliation
    assert 'all_known && all_active' in reconciliation
    assert 'request_epoch == state_epoch_' in reconciliation
    assert 'activation_cleanup_pending_ = true;' in reconciliation
    assert 'have_last_manager_attempt_ = false;' in reconciliation
    assert 'pause_confirmed_ = false;' in reconciliation

    launch_tree = ast.parse(
        (PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py').read_text(
            encoding='utf-8'
        )
    )
    interlock = _node_named(launch_tree, 'nav2_guard_interlock')
    params = _keyword(interlock, 'parameters').elts[0]
    assert _constant(_dict_value(params, 'manager_startup_timeout_sec')) == 30.0
    assert _constant(_dict_value(params, 'manager_transition_timeout_sec')) == 10.0
    assert _constant(
        _dict_value(params, 'manager_state_verification_timeout_sec')
    ) == 5.0


def test_interactive_waypoint_cycle_is_disabled_by_default():
    launch_path = PACKAGE_ROOT / 'launch' / 'wheeltec_nav2.launch.py'
    tree = ast.parse(launch_path.read_text(encoding='utf-8'))

    declaration = next(
        call
        for call in _call_named(tree, 'DeclareLaunchArgument')
        if call.args and _constant(call.args[0]) == 'enable_waypoint_cycle'
    )
    assert _constant(_keyword(declaration, 'default_value')) == 'false'

    waypoint_cycle = _node_named(tree, 'waypoint_cycle')
    condition = _keyword(waypoint_cycle, 'condition')
    assert isinstance(condition, ast.Call)
    assert isinstance(condition.func, ast.Name)
    assert condition.func.id == 'IfCondition'
    assert len(condition.args) == 1
    assert isinstance(condition.args[0], ast.Name)
    assert condition.args[0].id == 'enable_waypoint_cycle'

    lasertracker_declaration = next(
        call
        for call in _call_named(tree, 'DeclareLaunchArgument')
        if call.args and _constant(call.args[0]) == 'enable_lasertracker'
    )
    assert _constant(_keyword(lasertracker_declaration, 'default_value')) == 'false'


def test_bringup_keeps_localization_autostart_separate_from_navigation():
    bringup_path = PACKAGE_ROOT / 'launch' / 'bringup_launch.py'
    tree = ast.parse(bringup_path.read_text(encoding='utf-8'))

    declaration = next(
        call
        for call in _call_named(tree, 'DeclareLaunchArgument')
        if call.args and _constant(call.args[0]) == 'navigation_autostart'
    )
    default_value = _keyword(declaration, 'default_value')
    assert isinstance(default_value, ast.Name)
    assert default_value.id == 'autostart'

    localization_include = _include_for(tree, 'localization_launch.py')
    navigation_include = _include_for(tree, 'navigation_launch.py')
    localization_arguments = _keyword(localization_include, 'launch_arguments')
    navigation_arguments = _keyword(navigation_include, 'launch_arguments')

    localization_autostart = _dict_value(localization_arguments, 'autostart')
    navigation_autostart = _dict_value(navigation_arguments, 'autostart')
    assert isinstance(localization_autostart, ast.Name)
    assert localization_autostart.id == 'autostart'
    assert isinstance(navigation_autostart, ast.Name)
    assert navigation_autostart.id == 'navigation_autostart'


def test_composed_bringup_separates_localization_and_navigation_containers():
    bringup_path = PACKAGE_ROOT / 'launch' / 'bringup_launch.py'
    tree = ast.parse(bringup_path.read_text(encoding='utf-8'))

    component_containers = [
        call
        for call in _call_named(tree, 'Node')
        if _constant(_keyword(call, 'package')) == 'rclcpp_components'
        and _constant(_keyword(call, 'executable')) == 'component_container_isolated'
    ]
    assert {
        _constant(_keyword(call, 'name')) for call in component_containers
    } == {'localization_container', 'navigation_container'}
    for container in component_containers:
        condition = _keyword(container, 'condition')
        assert isinstance(condition, ast.Call)
        assert isinstance(condition.func, ast.Name)
        assert condition.func.id == 'IfCondition'
        assert isinstance(condition.args[0], ast.Name)
        assert condition.args[0].id == 'use_composition'

    localization_include = _include_for(tree, 'localization_launch.py')
    navigation_include = _include_for(tree, 'navigation_launch.py')
    assert _constant(_dict_value(
        _keyword(localization_include, 'launch_arguments'), 'container_name'
    )) == 'localization_container'
    assert _constant(_dict_value(
        _keyword(navigation_include, 'launch_arguments'), 'container_name'
    )) == 'navigation_container'

    composition_declaration = next(
        call
        for call in _call_named(tree, 'DeclareLaunchArgument')
        if call.args and _constant(call.args[0]) == 'use_composition'
    )
    assert _constant(_keyword(composition_declaration, 'default_value')) == 'True'


def test_build_and_manifest_install_the_interlock_dependencies():
    cmake = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'find_package(nav2_msgs REQUIRED)' in cmake
    assert 'find_package(lifecycle_msgs REQUIRED)' in cmake
    assert 'find_package(rclcpp_action REQUIRED)' in cmake
    assert 'add_executable(nav2_guard_interlock src/nav2_guard_interlock.cpp)' in cmake
    assert re.search(
        r'ament_target_dependencies\(nav2_guard_interlock.*?lifecycle_msgs.*?nav2_msgs.*?'
        r'rclcpp_action.*?\)',
        cmake,
        flags=re.DOTALL,
    )
    assert re.search(
        r'install\(\s*TARGETS.*?nav2_guard_interlock.*?DESTINATION '
        r'lib/\$\{PROJECT_NAME\}',
        cmake,
        flags=re.DOTALL,
    )

    manifest = ET.parse(PACKAGE_ROOT / 'package.xml').getroot()
    dependencies = {
        element.text
        for element in manifest
        if element.tag in {'depend', 'build_depend', 'exec_depend'}
    }
    assert {'lifecycle_msgs', 'nav2_msgs', 'rclcpp_action'} <= dependencies
