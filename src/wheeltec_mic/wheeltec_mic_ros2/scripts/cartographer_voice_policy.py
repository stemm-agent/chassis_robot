"""Pure state and command policy for the Cartographer voice bridge."""

IDLE = 'IDLE'
STARTING = 'STARTING'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
ENDING = 'ENDING'

START_COMMAND = '开始建图'
PAUSE_COMMAND = '暂停建图'
END_COMMAND = '结束建图'
SAVE_COMMAND = '保存地图'
CONTINUE_COMMAND = '继续建图'

COMMAND_ACTIONS = {
    START_COMMAND: 'start',
    PAUSE_COMMAND: 'pause',
    END_COMMAND: 'end',
    SAVE_COMMAND: 'save',
    CONTINUE_COMMAND: 'continue',
}

ALLOWED_STATES = {
    'start': {IDLE},
    'pause': {RUNNING},
    'end': {RUNNING, PAUSED},
    'save': {RUNNING, PAUSED},
    'continue': {PAUSED},
}

ACTIVE_UNIT_STATES = {'active', 'activating', 'deactivating', 'reloading'}
ENDING_MODES = {'returning_home', 'shutdown'}
PAUSED_MODES = {'paused', 'stopped'}
WAIT = 'wait'
SEND = 'send'
COMPLETE = 'complete'
TIMEOUT = 'timeout'


def command_action(command):
    return COMMAND_ACTIONS.get(command)


def command_is_allowed(action, state):
    return state in ALLOWED_STATES.get(action, set())


def command_in_cooldown(history, command, now, cooldown_sec):
    last = history.get(command)
    if last is not None and now - last < cooldown_sec:
        return True
    history[command] = now
    return False


def service_discovery_decision(service_ready, now, deadline):
    if service_ready:
        return SEND
    return TIMEOUT if now >= deadline else WAIT


def service_request_decision(request_done, now, deadline):
    if request_done:
        return COMPLETE
    return TIMEOUT if now >= deadline else WAIT


def classify_external_state(unit_state, lock_held, nav_state, nav_state_fresh):
    """Derive recoverable voice state from independently observable state."""
    session_active = unit_state in ACTIVE_UNIT_STATES or bool(lock_held)
    if not session_active:
        return IDLE
    if unit_state == 'deactivating':
        return ENDING
    if not nav_state_fresh or not isinstance(nav_state, dict):
        return STARTING

    mode = str(nav_state.get('mode') or '').strip().lower()
    navigation_state = str(
        nav_state.get('navigation_state') or '').strip().lower()
    if mode in PAUSED_MODES:
        return PAUSED
    if bool(nav_state.get('mapping_done')):
        return ENDING
    if mode in ENDING_MODES or navigation_state.startswith('mapping_done'):
        return ENDING
    if mode == 'idle':
        return STARTING
    return RUNNING


def motion_is_blocked(state, unit_state='inactive', lock_held=False):
    return (
        state != IDLE
        or unit_state in ACTIVE_UNIT_STATES
        or bool(lock_held)
    )
