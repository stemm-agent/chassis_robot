import json
import os
import signal
import subprocess
import threading
import time

import paho.mqtt.client as mqtt

BROKER_HOST = os.getenv('MQTT_BROKER_HOST', '0.0.0.0')
BROKER_PORT = int(os.getenv('MQTT_BROKER_PORT', '1883'))
DEVICE_ID = os.getenv('MQTT_DEVICE_ID', 'robot-001')
COMMAND_TOPIC = os.getenv('MQTT_COMMAND_TOPIC', f'robot/{DEVICE_ID}/launch/command')
STATUS_TOPIC = os.getenv('MQTT_STATUS_TOPIC', f'robot/{DEVICE_ID}/launch/status')
HEARTBEAT_TOPIC = os.getenv('MQTT_HEARTBEAT_TOPIC', f'robot/{DEVICE_ID}/heartbeat')
ERROR_TOPIC = os.getenv('MQTT_ERROR_TOPIC', f'robot/{DEVICE_ID}/error')
HEARTBEAT_INTERVAL = float(os.getenv('MQTT_HEARTBEAT_INTERVAL', '5'))
SERVICE_CALL_TIMEOUT = float(os.getenv('MQTT_SERVICE_CALL_TIMEOUT', '30'))
DEFAULT_PACKAGE = os.getenv('MQTT_LAUNCH_PACKAGE', 'wheeltec_nav2')
DEFAULT_LAUNCH_FILE = os.getenv('MQTT_LAUNCH_FILE', 'wheeltec_nav2.launch.py')
DEFAULT_NODE_PACKAGE = os.getenv('MQTT_NODE_PACKAGE', 'auto_recharge_ros2')
DEFAULT_NODE_EXECUTABLE = os.getenv('MQTT_NODE_EXECUTABLE', 'navigate_to_charger')


def build_target_key(package_name: str, entry_name: str) -> str:
    return f'{package_name}:{entry_name}'


def parse_allowed_targets(raw_targets: str, fallback_targets: list[str]) -> set[str]:
    source = raw_targets or ','.join(fallback_targets)
    return {
        item.strip()
        for item in source.split(',')
        if item.strip()
    }


ALLOWED_LAUNCH_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_LAUNCH_TARGETS', ''),
    [
        build_target_key('wheeltec_mic_ros2', 'mic_init.launch.py'),
        build_target_key('wheeltec_mic_ros2', 'base.launch.py'),
        build_target_key('stemm_nav2_manager', 'stemm_21_auto_mapping.launch.py'),
        build_target_key('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py'),
        build_target_key('bodyreader', 'bodyfollow_no_avoider.launch.py'),
        build_target_key('wheeltec_nav2', 'wheeltec_nav2.launch.py'),
        build_target_key('turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'),
        build_target_key('cloud_video_pusher', 'rtmp_pusher.launch.py'),
        build_target_key('cloud_video_pusher', 'local_hls_server.launch.py'),
    ],
)
ALLOWED_NODE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_NODE_TARGETS', ''),
    [
        build_target_key('wheeltec_robot_keyboard', 'wheeltec_keyboard'),
        build_target_key('auto_recharge_ros2', 'navigate_to_charger'),
        build_target_key('auto_recharge_ros2', 'cancel_charge'),
        build_target_key('web_video_server', 'web_video_server'),
    ],
)
ONE_SHOT_NODE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ONE_SHOT_NODE_TARGETS', ''),
    [],
)
ALLOWED_SERVICE_TARGETS = parse_allowed_targets(
    os.getenv('MQTT_ALLOWED_SERVICE_TARGETS', ''),
    [
        build_target_key('/stemm/set_mapping_done', 'std_srvs/srv/SetBool'),
        build_target_key('/stemm/save_map', 'std_srvs/srv/Trigger'),
    ],
)

state = {
    'launchStatus': 'idle',
    'currentTask': '-',
    'currentTaskType': '-',
    'lastError': '',
    'requestId': None,
    'runningLaunches': [],
    'runningNodes': [],
}
state_lock = threading.Lock()
launch_processes: dict[str, dict] = {}
node_processes: dict[str, dict] = {}
one_shot_processes: dict[str, dict] = {}
stop_event = threading.Event()


def publish_json(client: mqtt.Client, topic: str, payload: dict) -> None:
    client.publish(topic, json.dumps(payload, ensure_ascii=True), qos=1)


def serialize_process_entry(entry: dict) -> dict:
    payload = {
        'taskId': entry['taskId'],
        'requestId': entry['requestId'],
        'package': entry['package'],
        'startedAt': entry['startedAt'],
        'targetKey': entry['targetKey'],
    }
    if entry['kind'] == 'launch':
        payload['launchFile'] = entry['entryName']
    else:
        payload['nodeExecutable'] = entry['entryName']
        payload['oneShot'] = entry['oneShot']
    return payload


def refresh_state_locked() -> None:
    running_launches = [serialize_process_entry(entry) for entry in launch_processes.values()]
    running_nodes = [serialize_process_entry(entry) for entry in node_processes.values()]
    running_nodes.extend(serialize_process_entry(entry) for entry in one_shot_processes.values())

    state['runningLaunches'] = running_launches
    state['runningNodes'] = running_nodes

    if running_nodes:
        current = running_nodes[-1]
        state['currentTask'] = f"{current['package']}/{current['nodeExecutable']}"
        state['currentTaskType'] = 'node'
    elif running_launches:
        current = running_launches[-1]
        state['currentTask'] = f"{current['package']}/{current['launchFile']}"
        state['currentTaskType'] = 'launch'
    else:
        state['currentTask'] = '-'
        state['currentTaskType'] = '-'

    if running_launches or running_nodes or one_shot_processes:
        state['launchStatus'] = 'running'
    elif state['lastError']:
        state['launchStatus'] = 'failed'
    elif state['requestId']:
        state['launchStatus'] = 'stopped'
    else:
        state['launchStatus'] = 'idle'


def build_status_payload(message: str, **extra: object) -> dict:
    with state_lock:
        payload = {
            'deviceId': DEVICE_ID,
            'requestId': state['requestId'],
            'launchStatus': state['launchStatus'],
            'currentTask': state['currentTask'],
            'currentTaskType': state.get('currentTaskType', '-'),
            'lastError': state['lastError'],
            'runningLaunches': list(state.get('runningLaunches', [])),
            'runningNodes': list(state.get('runningNodes', [])),
            'message': message,
            'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
    payload.update(extra)
    return payload


def set_state(**kwargs: object) -> None:
    with state_lock:
        state.update(kwargs)
        refresh_state_locked()


def make_process_entry(kind: str, package_name: str, entry_name: str, request_id: str, process: subprocess.Popen, one_shot: bool = False) -> dict:
    return {
        'taskId': request_id,
        'requestId': request_id,
        'kind': kind,
        'package': package_name,
        'entryName': entry_name,
        'targetKey': build_target_key(package_name, entry_name),
        'process': process,
        'startedAt': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'oneShot': one_shot,
    }


def build_display_name(entry: dict) -> str:
    return f"{entry['package']}/{entry['entryName']}"


def remove_process_entry(kind: str, target_key: str) -> dict | None:
    if kind == 'launch':
        return launch_processes.pop(target_key, None)
    if kind == 'node':
        return node_processes.pop(target_key, None)
    return one_shot_processes.pop(target_key, None)


def monitor_process(client: mqtt.Client, kind: str, target_key: str, pid: int) -> None:
    if kind == 'launch':
        registry = launch_processes
    elif kind == 'node':
        registry = node_processes
    else:
        registry = one_shot_processes

    with state_lock:
        entry = registry.get(target_key)
    if not entry or entry['process'].pid != pid:
        return

    return_code = entry['process'].wait()

    with state_lock:
        current_entry = registry.get(target_key)
        if not current_entry or current_entry['process'].pid != pid:
            refresh_state_locked()
            return
        registry.pop(target_key, None)
        state['requestId'] = entry['requestId']
        if return_code == 0:
            if kind == 'oneshot':
                state['lastError'] = ''
            elif not state['lastError']:
                state['lastError'] = ''
        else:
            state['lastError'] = f"{kind} exited with code {return_code}: {build_display_name(entry)}"
        refresh_state_locked()

    if return_code == 0:
        if kind == 'oneshot':
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f"One-shot node completed successfully: {build_display_name(entry)}",
                    status='completed',
                ),
            )
        else:
            publish_json(
                client,
                STATUS_TOPIC,
                build_status_payload(
                    f"{kind.title()} exited successfully: {build_display_name(entry)}",
                    status='stopped',
                ),
            )
        return

    publish_json(
        client,
        ERROR_TOPIC,
        build_status_payload(
            f"{kind.title()} exited with code {return_code}: {build_display_name(entry)}",
            error=state['lastError'],
            status='failed',
        ),
    )


def start_process_monitor(client: mqtt.Client, kind: str, entry: dict) -> None:
    threading.Thread(
        target=monitor_process,
        args=(client, kind, entry['targetKey'], entry['process'].pid),
        daemon=True,
    ).start()


def stop_process_entry(entry: dict) -> None:
    process = entry['process']
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)


def launch_command(package_name: str, launch_file: str, args: dict) -> list[str]:
    command = ['ros2', 'launch', package_name, launch_file]
    for key, value in args.items():
        command.append(f'{key}:={value}')
    return command


def node_command(package_name: str, executable_name: str, args: dict) -> list[str]:
    command = ['ros2', 'run', package_name, executable_name]
    for key, value in args.items():
        command.extend([f'--{key}', str(value)])
    return command


def service_command(service_path: str, service_type: str, service_args: dict) -> list[str]:
    return ['ros2', 'service', 'call', service_path, service_type, json.dumps(service_args, ensure_ascii=True)]


def start_launch(client: mqtt.Client, payload: dict) -> None:
    launch_file = (payload.get('launchFile') or DEFAULT_LAUNCH_FILE).strip()
    package_name = (payload.get('package') or DEFAULT_PACKAGE).strip()
    launch_args = payload.get('args') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, launch_file)

    if target_key not in ALLOWED_LAUNCH_TARGETS:
        raise ValueError(f'Launch target not allowed: {package_name}/{launch_file}')
    if not isinstance(launch_args, dict):
        raise ValueError('args must be an object')

    with state_lock:
        existing_entry = launch_processes.get(target_key)
        if existing_entry and existing_entry['process'].poll() is None:
            raise RuntimeError(f'Launch task already running: {package_name}/{launch_file}')

    command = launch_command(package_name, launch_file, launch_args)
    process = subprocess.Popen(command)
    entry = make_process_entry('launch', package_name, launch_file, request_id, process)

    with state_lock:
        launch_processes[target_key] = entry
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    start_process_monitor(client, 'launch', entry)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(f'Launch started successfully: {package_name}/{launch_file}', status='running'),
    )


def start_node(client: mqtt.Client, payload: dict) -> None:
    package_name = (payload.get('package') or DEFAULT_NODE_PACKAGE).strip()
    node_executable = (payload.get('nodeExecutable') or DEFAULT_NODE_EXECUTABLE).strip()
    node_args = payload.get('args') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, node_executable)

    if target_key not in ALLOWED_NODE_TARGETS:
        raise ValueError(f'Node target not allowed: {package_name}/{node_executable}')
    if not isinstance(node_args, dict):
        raise ValueError('args must be an object')

    with state_lock:
        existing_entry = node_processes.get(target_key)
        if existing_entry and existing_entry['process'].poll() is None:
            raise RuntimeError(f'Node task already running: {package_name}/{node_executable}')
        existing_one_shot = one_shot_processes.get(target_key)
        if existing_one_shot and existing_one_shot['process'].poll() is None:
            raise RuntimeError(f'One-shot node already running: {package_name}/{node_executable}')

    command = node_command(package_name, node_executable, node_args)
    process = subprocess.Popen(command)
    one_shot = target_key in ONE_SHOT_NODE_TARGETS
    kind = 'oneshot' if one_shot else 'node'
    entry = make_process_entry('node', package_name, node_executable, request_id, process, one_shot=one_shot)

    with state_lock:
        if one_shot:
            one_shot_processes[target_key] = entry
        else:
            node_processes[target_key] = entry
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    start_process_monitor(client, kind, entry)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            f"{'One-shot node' if one_shot else 'Node'} started successfully: {package_name}/{node_executable}",
            status='running',
        ),
    )


# Business stop only targets the managed launch process instead of arbitrary commands.
def stop_launch(client: mqtt.Client, payload: dict) -> None:
    launch_file = (payload.get('launchFile') or DEFAULT_LAUNCH_FILE).strip()
    package_name = (payload.get('package') or DEFAULT_PACKAGE).strip()
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, launch_file)

    with state_lock:
        entry = launch_processes.get(target_key)

    if not entry or entry['process'].poll() is not None:
        set_state(requestId=request_id)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(f'No active launch task to stop: {package_name}/{launch_file}', status='idle'),
        )
        return

    stop_process_entry(entry)

    with state_lock:
        remove_process_entry('launch', target_key)
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(f'Launch stopped successfully: {package_name}/{launch_file}', status='stopped'),
    )


def stop_node(client: mqtt.Client, payload: dict) -> None:
    package_name = (payload.get('package') or DEFAULT_NODE_PACKAGE).strip()
    node_executable = (payload.get('nodeExecutable') or DEFAULT_NODE_EXECUTABLE).strip()
    request_id = payload.get('requestId')
    target_key = build_target_key(package_name, node_executable)

    with state_lock:
        entry = node_processes.get(target_key)

    if not entry or entry['process'].poll() is not None:
        set_state(requestId=request_id)
        publish_json(
            client,
            STATUS_TOPIC,
            build_status_payload(f'No active node task to stop: {package_name}/{node_executable}', status='idle'),
        )
        return

    stop_process_entry(entry)

    with state_lock:
        remove_process_entry('node', target_key)
        state['requestId'] = request_id
        state['lastError'] = ''
        refresh_state_locked()

    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(f'Node stopped successfully: {package_name}/{node_executable}', status='stopped'),
    )


def call_service(client: mqtt.Client, payload: dict) -> None:
    service_path = str(payload.get('servicePath') or '').strip()
    service_type = str(payload.get('serviceType') or '').strip()
    service_args = payload.get('serviceArgs') or {}
    request_id = payload.get('requestId')
    target_key = build_target_key(service_path, service_type)

    if not service_path:
        raise ValueError('servicePath is required')
    if not service_type:
        raise ValueError('serviceType is required')
    if target_key not in ALLOWED_SERVICE_TARGETS:
        raise ValueError(f'Service target not allowed: {service_path}/{service_type}')
    if not isinstance(service_args, dict):
        raise ValueError('serviceArgs must be an object')

    set_state(
        requestId=request_id,
        currentTask=service_path,
        currentTaskType='service',
        lastError='',
    )
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(f'Service call started: {service_path}', status='starting'),
    )

    completed = subprocess.run(
        service_command(service_path, service_type, service_args),
        capture_output=True,
        text=True,
        timeout=SERVICE_CALL_TIMEOUT,
        check=False,
    )

    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    details = stdout or stderr or 'Service call completed.'

    if completed.returncode != 0:
        raise RuntimeError(f'Service call failed ({completed.returncode}): {service_path}; {details}')

    set_state(
        requestId=request_id,
        currentTask=service_path,
        currentTaskType='service',
        lastError='',
    )
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload(
            f'Service call completed: {service_path}',
            status='completed',
            serviceResult=details,
        ),
    )


def heartbeat_loop(client: mqtt.Client) -> None:
    while not stop_event.wait(HEARTBEAT_INTERVAL):
        with state_lock:
            refresh_state_locked()
        publish_json(
            client,
            HEARTBEAT_TOPIC,
            {
                'deviceId': DEVICE_ID,
                'jetsonOnline': True,
                'launchStatus': state['launchStatus'],
                'currentTask': state['currentTask'],
                'currentTaskType': state.get('currentTaskType', '-'),
                'lastError': state['lastError'],
                'runningLaunches': list(state.get('runningLaunches', [])),
                'runningNodes': list(state.get('runningNodes', [])),
                'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
            },
        )


def on_connect(client: mqtt.Client, userdata, flags, reason_code, properties=None) -> None:
    print(f'MQTT connected: reason_code={reason_code}')
    client.subscribe(COMMAND_TOPIC, qos=1)
    publish_json(
        client,
        STATUS_TOPIC,
        build_status_payload('Jetson MQTT bridge connected.', status='idle'),
    )


def on_message(client: mqtt.Client, userdata, msg: mqtt.MQTTMessage) -> None:
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        action = payload.get('action')

        if action == 'start_launch':
            start_launch(client, payload)
            return
        if action == 'stop_launch':
            stop_launch(client, payload)
            return
        if action == 'start_node':
            start_node(client, payload)
            return
        if action == 'stop_node':
            stop_node(client, payload)
            return
        if action == 'call_service':
            call_service(client, payload)
            return

        raise ValueError(f'Unsupported action: {action}')
    except Exception as exc:  # noqa: BLE001
        set_state(lastError=str(exc), launchStatus='failed')
        publish_json(
            client,
            ERROR_TOPIC,
            build_status_payload(str(exc), error=str(exc)),
        )


def main() -> None:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f'jetson-launch-{DEVICE_ID}')
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=30)

    heartbeat_thread = threading.Thread(target=heartbeat_loop, args=(client,), daemon=True)
    heartbeat_thread.start()

    try:
        client.loop_forever()
    finally:
        stop_event.set()


if __name__ == '__main__':
    main()
