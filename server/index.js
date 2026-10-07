const http = require('http');
const https = require('https');
const fs = require('fs');
const path = require('path');
const os = require('os');
const { execFile } = require('child_process');
const mqtt = require('mqtt');
const { WebSocketServer } = require('ws');

const PORT = Number(process.env.PORT || '3000');
const HOST = process.env.HOST || '0.0.0.0';
const WEB_ROOT = path.join(__dirname, '..', 'web');
const JETSON_AGENT_URL = process.env.JETSON_AGENT_URL || '';
// 2026-10-07: 建图恢复转发的超时保护（见 /api/feature/mapping/recover 路由）
const RECOVER_BODY_TIMEOUT_MS = 5000;
const RECOVER_PROXY_TIMEOUT_MS = 8000;
const MANUAL_LINEAR_SPEED = Number(process.env.MANUAL_LINEAR_SPEED || '0.1');
const MANUAL_ANGULAR_SPEED = Number(process.env.MANUAL_ANGULAR_SPEED || '0.5');
const CONTROL_MODE = JETSON_AGENT_URL ? 'proxy' : 'mock';
const MQTT_BROKER_URL = process.env.MQTT_BROKER_URL || 'mqtt://127.0.0.1:1883';
const MQTT_DEVICE_ID = process.env.MQTT_DEVICE_ID || 'robot-001';
const MQTT_CLIENT_ID = process.env.MQTT_CLIENT_ID || `demo-control-${MQTT_DEVICE_ID}`;
const MQTT_ENABLED = process.env.MQTT_ENABLED !== 'false';
const MQTT_COMMAND_TOPIC = process.env.MQTT_COMMAND_TOPIC || `robot/${MQTT_DEVICE_ID}/launch/command`;
const MQTT_STATUS_TOPIC = process.env.MQTT_STATUS_TOPIC || `robot/${MQTT_DEVICE_ID}/launch/status`;
const MQTT_HEARTBEAT_TOPIC = process.env.MQTT_HEARTBEAT_TOPIC || `robot/${MQTT_DEVICE_ID}/heartbeat`;
const MQTT_ERROR_TOPIC = process.env.MQTT_ERROR_TOPIC || `robot/${MQTT_DEVICE_ID}/error`;
const MQTT_FEATURE_STATUS_TOPIC = process.env.MQTT_FEATURE_STATUS_TOPIC ||
  `robot/${MQTT_DEVICE_ID}/feature/status`;
const DEFAULT_LAUNCH_PACKAGE = process.env.MQTT_LAUNCH_PACKAGE || 'wheeltec_nav2';
const DEFAULT_LAUNCH_FILE = process.env.MQTT_LAUNCH_FILE || 'wheeltec_nav2.launch.py';
const DEFAULT_NODE_PACKAGE = process.env.MQTT_NODE_PACKAGE || 'auto_recharge_ros2';
const DEFAULT_NODE_EXECUTABLE = process.env.MQTT_NODE_EXECUTABLE || 'navigate_to_charger';
const MAP_FILE_PATH = process.env.MAP_FILE_PATH || path.join(os.homedir(), '.local', 'share', 'stemm', 'maps', 'stemm_map.pgm');
const MAP_SERVICE_PATH = process.env.MAP_SERVICE_PATH || '/stemm/save_map';
const MAP_SERVICE_URL = process.env.MAP_SERVICE_URL || 'http://127.0.0.1:8092';
const STATUS_MAP_PROBE_TIMEOUT_MS = (() => {
  const value = Number(process.env.STATUS_MAP_PROBE_TIMEOUT_MS || '500');
  return Number.isFinite(value) ? Math.max(100, value) : 500;
})();
const LOCAL_HLS_URL = process.env.LOCAL_HLS_URL || 'http://127.0.0.1:18080';
const LOCAL_VIDEO_PLAY_TYPE = process.env.LOCAL_VIDEO_PLAY_TYPE || 'hls';
const LOCAL_VIDEO_POLL_INTERVAL = Number(process.env.LOCAL_VIDEO_POLL_INTERVAL || '150');
const LOCAL_VIDEO_FRAME_PATH = process.env.LOCAL_VIDEO_FRAME_PATH || '/live/robot001.jpg';
const LOCAL_VIDEO_HLS_PATH = process.env.LOCAL_VIDEO_HLS_PATH || '/live/robot001.m3u8';
const LOCAL_VIDEO_HEALTH_TIMEOUT_MS = Number(process.env.LOCAL_VIDEO_HEALTH_TIMEOUT_MS || '1500');
const MAP_FORMAT = 'pgm';
const TASK_STATUS_STORE_PATH = process.env.TASK_STATUS_STORE_PATH || path.join(os.homedir(), '.local', 'state', 'demo-control', 'task-status.json');
const MAX_TASK_STATUS_RECORDS = Number(process.env.MAX_TASK_STATUS_RECORDS || '200');
const CARTOGRAPHER_SESSION_STATUS_PATH = process.env.CARTOGRAPHER_SESSION_STATUS_PATH || path.join(os.homedir(), '.local', 'state', 'stemm-cartographer-session.json');
const CARTOGRAPHER_TASK_STALE_GRACE_MS = Number(process.env.CARTOGRAPHER_TASK_STALE_GRACE_MS || '15000');
const CARTOGRAPHER_SESSION_FRESHNESS_MS = Number(process.env.CARTOGRAPHER_SESSION_FRESHNESS_MS || '10000');
const CARTOGRAPHER_TERMINAL_SESSION_RETENTION_MS = Number(process.env.CARTOGRAPHER_TERMINAL_SESSION_RETENTION_MS || '600000');
const CARTOGRAPHER_TRANSITION_SESSION_FRESHNESS_MS = Number(process.env.CARTOGRAPHER_TRANSITION_SESSION_FRESHNESS_MS || '180000');

function readCurrentBootId() {
  try {
    return fs.readFileSync('/proc/sys/kernel/random/boot_id', 'utf8').trim();
  } catch (error) {
    return '';
  }
}

const CURRENT_BOOT_ID = readCurrentBootId();


const DEVICE_RESTART_STATUS_PATH =
  process.env.DEVICE_RESTART_STATUS_PATH ||
  '/var/lib/wheeltec/device-restart/state.json';
const DEVICE_RESTART_LAUNCHER =
  process.env.DEVICE_RESTART_LAUNCHER ||
  '/usr/local/sbin/stemm-device-restart-launch';

// 2026-09-28: car-side provisioning helper (see the /api/network/* routes below).
const NETWORK_AGENT_LAUNCHER =
  process.env.NETWORK_AGENT_LAUNCHER ||
  '/usr/local/sbin/stemm-network-agent-launch';
const DEVICE_RESTART_STATES = new Set([
  'idle', 'queued', 'preparing', 'stopping', 'restarting',
  'recovering', 'succeeded', 'degraded', 'failed',
]);
const DEVICE_RESTART_ACTIVE_STATES = new Set([
  'queued', 'preparing', 'stopping', 'restarting', 'recovering',
]);

function deviceRestartString(value, maxLength = 160) {
  return typeof value === 'string' ? value.slice(0, maxLength) : '';
}

function deviceRestartEpochMs(value) {
  const number = Number(value);
  return Number.isFinite(number) && number >= 0 ? Math.floor(number) : 0;
}

function buildIdleDeviceRestartStatus() {
  return {
    restartId: '',
    scope: 'service_stack',
    state: 'idle',
    phase: 'idle',
    requestedAtEpochMs: 0,
    startedAtEpochMs: 0,
    updatedAtEpochMs: 0,
    finishedAtEpochMs: null,
    bootIdBefore: '',
    bootIdAfter: CURRENT_BOOT_ID,
    lastErrorCode: '',
    health: {
      ready: false,
      managedServicesReady: false,
      robotAgentReady: false,
      featureStatusReady: false,
    },
  };
}

function normalizeDeviceRestartStatus(value) {
  const fallback = buildIdleDeviceRestartStatus();
  if (!value || typeof value !== 'object' || Array.isArray(value)) {
    return fallback;
  }
  const state = deviceRestartString(value.state, 32);
  if (!DEVICE_RESTART_STATES.has(state)) {
    return fallback;
  }
  const sourceHealth = value.health && typeof value.health === 'object'
    ? value.health
    : {};
  const finishedAtEpochMs = value.finishedAtEpochMs === null
    ? null
    : deviceRestartEpochMs(value.finishedAtEpochMs);
  return {
    restartId: deviceRestartString(value.restartId, 160),
    scope: value.scope === 'service_stack' ? 'service_stack' : 'service_stack',
    state,
    phase: deviceRestartString(value.phase, 80),
    requestedAtEpochMs: deviceRestartEpochMs(value.requestedAtEpochMs),
    startedAtEpochMs: deviceRestartEpochMs(value.startedAtEpochMs),
    updatedAtEpochMs: deviceRestartEpochMs(value.updatedAtEpochMs),
    finishedAtEpochMs,
    bootIdBefore: deviceRestartString(value.bootIdBefore, 160),
    bootIdAfter: deviceRestartString(value.bootIdAfter, 160),
    lastErrorCode: deviceRestartString(value.lastErrorCode, 80),
    health: {
      ready: sourceHealth.ready === true,
      managedServicesReady: sourceHealth.managedServicesReady === true,
      robotAgentReady: sourceHealth.robotAgentReady === true,
      featureStatusReady: sourceHealth.featureStatusReady === true,
    },
  };
}

function getDeviceRestartStatus() {
  try {
    const raw = fs.readFileSync(DEVICE_RESTART_STATUS_PATH, 'utf8');
    return normalizeDeviceRestartStatus(JSON.parse(raw));
  } catch (error) {
    return buildIdleDeviceRestartStatus();
  }
}

function isDeviceRestartActive() {
  return DEVICE_RESTART_ACTIVE_STATES.has(getDeviceRestartStatus().state);
}

function normalizeDeviceRestartIdempotencyKey(value) {
  const key = deviceRestartString(value, 160).trim();
  return /^restart-[A-Za-z0-9_-]{8,128}$/.test(key) ? key : '';
}

function parseDeviceRestartLauncherResult(value) {
  const text = String(value || '').trim();
  if (!text) {
    return null;
  }
  try {
    const parsed = JSON.parse(text);
    return parsed && typeof parsed === 'object' && !Array.isArray(parsed)
      ? parsed
      : null;
  } catch (error) {
    return null;
  }
}

function requestDeviceRestart(idempotencyKey) {
  return new Promise(resolve => {
    execFile(
      'sudo',
      ['-n', DEVICE_RESTART_LAUNCHER, '--idempotency-key', idempotencyKey],
      { timeout: 6000, maxBuffer: 64 * 1024 },
      (error, stdout) => {
        const parsed = parseDeviceRestartLauncherResult(stdout);
        if (parsed) {
          resolve({
            ok: parsed.ok === true,
            accepted: parsed.accepted === true,
            reused: parsed.reused === true,
            code: deviceRestartString(parsed.code, 80),
            restartId: deviceRestartString(parsed.restartId, 160),
            state: deviceRestartString(parsed.state, 32),
            deviceRestart: normalizeDeviceRestartStatus(
              parsed.deviceRestart || getDeviceRestartStatus()
            ),
          });
          return;
        }
        const deviceRestart = getDeviceRestartStatus();
        resolve({
          ok: false,
          accepted: false,
          reused: false,
          code: 'DEVICE_RESTART_UNAVAILABLE',
          restartId: '',
          state: deviceRestart.state,
          deviceRestart,
        });
      }
    );
  });
}

function deviceRestartBlockedResponse() {
  return {
    ok: false,
    code: 'DEVICE_RESTART_IN_PROGRESS',
    deviceRestart: getDeviceRestartStatus(),
  };
}

function parseAllowedTargets(rawTargets, fallbackTargets) {
  return new Set(
    (rawTargets || fallbackTargets.join(','))
      .split(',')
      .map(item => item.trim())
      .filter(Boolean)
  );
}

function buildTargetKey(packageName, entryName) {
  return `${packageName}:${entryName}`;
}

const ALLOWED_LAUNCH_TARGETS = parseAllowedTargets(
  process.env.MQTT_ALLOWED_LAUNCH_TARGETS,
  [
    buildTargetKey('wheeltec_mic_ros2', 'mic_init.launch.py'),
    buildTargetKey('wheeltec_mic_ros2', 'base.launch.py'),
    // The mini program starts a mapping session with action=start_launch on this
    // target; it was missing from the whitelist, so every "开始建图" tap was
    // rejected with HTTP 400 (observed 2026-09-16).
    buildTargetKey('stemm_cartographer_exploration', 'stemm_cartographer_session.launch.py'),
    buildTargetKey('stemm_nav2_manager', 'stemm_21_auto_mapping.launch.py'),
    buildTargetKey('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py'),
    buildTargetKey('bodyreader', 'bodyfollow_no_avoider.launch.py'),
    buildTargetKey('wheeltec_nav2', 'wheeltec_nav2.launch.py'),
    buildTargetKey('turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'),
    buildTargetKey('cloud_video_pusher', 'rtmp_pusher.launch.py'),
    buildTargetKey('cloud_video_pusher', 'local_hls_server.launch.py'),
  ]
);

const VIDEO_LAUNCH_TARGETS = new Set([
  buildTargetKey('cloud_video_pusher', 'rtmp_pusher.launch.py'),
  buildTargetKey('cloud_video_pusher', 'local_hls_server.launch.py'),
]);

const ALLOWED_NODE_TARGETS = parseAllowedTargets(
  process.env.MQTT_ALLOWED_NODE_TARGETS,
  [
    buildTargetKey('wheeltec_robot_keyboard', 'wheeltec_keyboard'),
    buildTargetKey('auto_recharge_ros2', 'navigate_to_charger'),
    buildTargetKey('auto_recharge_ros2', 'cancel_charge'),
    buildTargetKey('web_video_server', 'web_video_server'),
  ]
);

const ALLOWED_SERVICE_TARGETS = parseAllowedTargets(
  process.env.MQTT_ALLOWED_SERVICE_TARGETS,
  [
    buildTargetKey('/stemm/set_mapping_done', 'std_srvs/srv/SetBool'),
    // The mini program addresses the modern service names; the legacy /stemm/*
    // entries above are kept for older clients.
    buildTargetKey('/stemm_cartographer/set_mapping_done', 'std_srvs/srv/SetBool'),
    buildTargetKey('/stemm_cartographer/save_map', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm_cartographer/save_state', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm_cartographer/cancel_mapping', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm/save_map', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm_cartographer/stop_navigation', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm_cartographer/start_mapping', 'std_srvs/srv/Trigger'),
    buildTargetKey('/stemm_cartographer/cancel_mapping', 'std_srvs/srv/Trigger'),
    buildTargetKey('/voice_summon/set_body_follow_enabled', 'std_srvs/srv/SetBool'),
    buildTargetKey('/voice_summon/get_body_follow_status', 'std_srvs/srv/Trigger'),
  ]
);

const robotState = {
  deviceId: MQTT_DEVICE_ID,
  status: 'idle',
  activeTaskId: null,
  movement: 'stopped',
  controlMode: CONTROL_MODE,
  mqttEnabled: MQTT_ENABLED,
  mqttConnected: false,
  mqttBrokerUrl: MQTT_BROKER_URL,
  mqttCommandTopic: MQTT_COMMAND_TOPIC,
  launchStatus: 'idle',
  currentTask: '-',
  currentTaskType: '-',
  runningLaunches: [],
  runningNodes: [],
  jetsonOnline: false,
  lastError: '',
  featureStatus: null,
  history: [],
};

const mapState = {
  mapPath: MAP_FILE_PATH,
  lastAttemptAt: '',
  lastSuccessAt: '',
  lastError: '',
  lastCommandStatus: 'idle',
};

const taskStatusRecords = new Map();

function loadTaskStatusRecords() {
  try {
    const rawRecords = JSON.parse(fs.readFileSync(TASK_STATUS_STORE_PATH, 'utf8'));
    if (!Array.isArray(rawRecords)) {
      return;
    }
    rawRecords.slice(-MAX_TASK_STATUS_RECORDS).forEach(record => {
      if (record && typeof record.taskId === 'string' && record.taskId) {
        taskStatusRecords.set(record.taskId, record);
      }
    });
  } catch (error) {
    if (error.code !== 'ENOENT') {
      console.warn('Unable to load task status store: ' + error.message);
    }
  }
}

function persistTaskStatusRecords() {
  try {
    fs.mkdirSync(path.dirname(TASK_STATUS_STORE_PATH), { recursive: true });
    const records = Array.from(taskStatusRecords.values()).slice(-MAX_TASK_STATUS_RECORDS);
    const tempPath = TASK_STATUS_STORE_PATH + '.tmp';
    fs.writeFileSync(tempPath, JSON.stringify(records), 'utf8');
    fs.renameSync(tempPath, TASK_STATUS_STORE_PATH);
  } catch (error) {
    console.warn('Unable to persist task status store: ' + error.message);
  }
}

function recordTaskStatus(payload) {
  if (!payload || typeof payload.taskId !== 'string' || !payload.taskId) {
    return;
  }

  const previous = taskStatusRecords.get(payload.taskId) || {};
  const isHeartbeat = payload.message === 'Jetson heartbeat received.';
  taskStatusRecords.set(payload.taskId, isHeartbeat && previous.taskId
    ? {
      ...previous,
      runningLaunches: payload.runningLaunches || previous.runningLaunches || [],
      runningNodes: payload.runningNodes || previous.runningNodes || [],
    }
    : {
      ...previous,
      ...payload,
      bootId: CURRENT_BOOT_ID,
      time: payload.timestamp || new Date().toISOString(),
    });
  while (taskStatusRecords.size > MAX_TASK_STATUS_RECORDS) {
    taskStatusRecords.delete(taskStatusRecords.keys().next().value);
  }
  persistTaskStatusRecords();
}

function isCartographerLaunch(entry) {
  return Boolean(entry && entry.package === 'stemm_cartographer_exploration' && entry.launchFile === 'stemm_cartographer_session.launch.py');
}

function readCartographerSessionStatus() {
  try {
    const session = JSON.parse(fs.readFileSync(CARTOGRAPHER_SESSION_STATUS_PATH, 'utf8'));
    return session && typeof session === 'object' ? session : null;
  } catch (error) {
    return null;
  }
}

function findActiveCartographerTaskId() {
  const entry = (robotState.runningLaunches || []).find(isCartographerLaunch);
  if (entry && typeof entry.taskId === 'string') return entry.taskId;
  const record = Array.from(taskStatusRecords.values()).reverse().find(item =>
    (item.runningLaunches || []).some(isCartographerLaunch)
  );
  return record && typeof record.taskId === 'string' ? record.taskId : '';
}

function isTerminalCartographerSession(session) {
  return Boolean(session) && (
    session.terminal === true || String(session.phase || '') === 'closed'
  );
}

function isTransitioningCartographerSession(session) {
  const phase = String((session && session.phase) || '');
  return [
    'return_home_failed',
    'canceling',
    'finalizing',
    'recovery_required',
  ].includes(phase);
}

function isFreshCartographerSession(session) {
  const updatedAtEpochMs = Number(session && session.updatedAtEpochMs);
  const ageMs = Date.now() - updatedAtEpochMs;
  if (!Number.isFinite(updatedAtEpochMs) || ageMs < 0) return false;
  const maxAgeMs = isTerminalCartographerSession(session)
    ? CARTOGRAPHER_TERMINAL_SESSION_RETENTION_MS
    : (isTransitioningCartographerSession(session)
      ? CARTOGRAPHER_TRANSITION_SESSION_FRESHNESS_MS
      : CARTOGRAPHER_SESSION_FRESHNESS_MS);
  return ageMs <= maxAgeMs;
}

function reconcileCartographerSessionStatus() {
  const session = readCartographerSessionStatus();
  if (!session || !isFreshCartographerSession(session)) return null;
  const taskId = (typeof session.taskId === 'string' && session.taskId)
    ? session.taskId
    : findActiveCartographerTaskId();
  if (!taskId) return session;

  const previous = taskStatusRecords.get(taskId) || {};
  const previousSessionEpoch = previous.mappingSession &&
    previous.mappingSession.updatedAtEpochMs;
  const sessionChanged = previousSessionEpoch !== session.updatedAtEpochMs;
  const phase = String(session.phase || '');
  const reportedTerminalStatus = String(
    session.taskStatus || session.terminationStatus || ''
  ).toLowerCase();
  let nextRecord = null;

  if (phase === 'closed' && session.terminal === true) {
    const canceled = reportedTerminalStatus === 'canceled';
    const succeeded = [
      'succeeded',
      'completed',
      'success',
    ].includes(reportedTerminalStatus);
    if (canceled || succeeded) {
      const status = canceled ? 'canceled' : 'completed';
      const message = canceled
        ? 'Cartographer session canceled after return-home failure.'
        : 'Cartographer session closed successfully.';
      if (
        normalizeTaskStatus(previous) !== (canceled ? 'canceled' : 'succeeded') ||
        previous.terminal !== true ||
        previousSessionEpoch !== session.updatedAtEpochMs
      ) {
        nextRecord = {
          ...previous,
          taskId,
          bootId: CURRENT_BOOT_ID,
          status,
          terminal: true,
          message,
          // Keep the original return-home failure as diagnostic session data;
          // a safely canceled mapping task is not an active task failure.
          lastError: '',
          mappingSession: session,
          time: session.updatedAt || new Date().toISOString(),
          runningLaunches: [],
          runningNodes: [],
        };
      }
    }
  } else if (
    phase === 'return_home_failed' &&
    session.terminal !== true &&
    session.manualTakeoverRequired === true
  ) {
    const failureReason = session.returnHomeFailureReason || session.lastError ||
      'Cartographer return-home failed; cancellation is pending';
    if (
      previous.status !== 'running' ||
      previous.terminal === true ||
      previous.cancellationReason !== failureReason ||
      sessionChanged
    ) {
      nextRecord = {
        ...previous,
        taskId,
        bootId: CURRENT_BOOT_ID,
        status: 'running',
        terminal: false,
        message: 'Cartographer return-home failed; waiting for controlled cancellation.',
        lastError: '',
        cancellationReason: failureReason,
        mappingSession: session,
        time: session.updatedAt || new Date().toISOString(),
      };
    }
  } else if (
    session.terminal !== true &&
    ['canceling', 'finalizing', 'recovery_required'].includes(phase)
  ) {
    const cancellationReason = session.returnHomeFailureReason ||
      session.terminationReason || session.lastError || '';
    if (
      previous.status !== 'running' ||
      previous.terminal === true ||
      previous.cancellationReason !== cancellationReason ||
      sessionChanged
    ) {
      nextRecord = {
        ...previous,
        taskId,
        bootId: CURRENT_BOOT_ID,
        status: 'running',
        terminal: false,
        message: phase === 'recovery_required'
          ? 'Cartographer session recovery is required before control can be released.'
          : 'Cartographer session is shutting down and restoring functional services.',
        lastError: '',
        cancellationReason,
        mappingSession: session,
        time: session.updatedAt || new Date().toISOString(),
      };
    }
  }

  if (nextRecord) {
    taskStatusRecords.set(taskId, nextRecord);
    persistTaskStatusRecords();
  }
  return session;
}

function normalizeTaskStatus(task) {
  // A process-exit fallback is a lifecycle fact even when it retains a
  // diagnostic message, and must not be surfaced as an active failure.
  if (task && task.mappingProcessExited === true) return 'canceled';
  if (String(task.lastError || '').trim()) return 'failed';
  const rawStatus = String(task.status || task.launchStatus || '').trim();
  if (rawStatus === 'failed') return 'failed';
  if (rawStatus === 'canceled') return 'canceled';
  if (rawStatus === 'completed') return 'succeeded';
  if (rawStatus === 'stopped') {
    return /exited successfully/i.test(String(task.message || '')) ? 'succeeded' : 'canceled';
  }
  if (['launch_command_sent', 'node_command_sent', 'service_command_sent', 'received', 'starting', 'idle'].includes(rawStatus)) {
    return 'pending';
  }
  return 'running';
}

function isTerminalTaskStatus(status) {
  return ['succeeded', 'failed', 'canceled'].includes(status);
}

function isCartographerTaskRecord(task) {
  if (!task || typeof task !== 'object') return false;
  if (String(task.currentTask || '').includes('stemm_cartographer_exploration/stemm_cartographer_session.launch.py')) {
    return true;
  }
  return (task.runningLaunches || []).some(isCartographerLaunch);
}

function setCartographerTaskExited(taskId, task, message) {
  const now = new Date().toISOString();
  const mappingSession = {
    schemaVersion: 2,
    taskId,
    updatedAt: now,
    updatedAtEpochMs: Date.now(),
    phase: 'closed',
    status: 'canceled',
    taskStatus: 'canceled',
    terminal: true,
    sessionActive: false,
    processExited: true,
    // Recovery remains diagnostic only. Modal close is based solely on
    // completion of the mapping process, per the device-control contract.
    functionalServicesRestored: false,
    cancelRequested: true,
    terminationStatus: 'canceled',
    terminationReason: 'mapping_process_exited_without_live_session',
    returnHomeFailureReason: message,
    mappingDone: true,
    lastError: message,
  };
  const updated = {
    ...task,
    taskId,
    bootId: CURRENT_BOOT_ID,
    status: 'canceled',
    terminal: true,
    mappingProcessExited: true,
    mappingSession,
    lastError: message,
    message,
    time: now,
    runningLaunches: [],
    runningNodes: [],
  };
  taskStatusRecords.set(taskId, updated);
  persistTaskStatusRecords();
  return updated;
}

function invalidateCartographerTasksFromOtherBoots() {
  let changed = false;
  for (const [taskId, task] of taskStatusRecords.entries()) {
    if (!isCartographerTaskRecord(task) || isTerminalTaskStatus(normalizeTaskStatus(task))) continue;
    if (!CURRENT_BOOT_ID || task.bootId === CURRENT_BOOT_ID) continue;
    taskStatusRecords.set(taskId, {
      ...task,
      taskId,
      bootId: CURRENT_BOOT_ID,
      status: 'failed',
      terminal: true,
      lastError: 'Cartographer task was invalidated because the robot rebooted.',
      message: 'Cartographer task was invalidated because the robot rebooted.',
      time: new Date().toISOString(),
      runningLaunches: [],
      runningNodes: [],
    });
    changed = true;
  }
  if (changed) persistTaskStatusRecords();
}

function isSessionBackedCartographerTask(session, taskId) {
  if (!session || session.taskId !== taskId) return false;
  const phase = String(session.phase || '');
  return session.sessionActive === true || [
    'mapping',
    'returning_home',
    'return_home_failed',
    'canceling',
    'finalizing',
    'recovery_required',
    'closed',
  ].includes(phase);
}

function reconcileInactiveCartographerTask(taskId, task) {
  if (!task || !isCartographerTaskRecord(task)) {
    return task;
  }

  // Reconcile the authoritative session before treating a bridge process
  // exit (or an old failed record) as terminal.
  const session = reconcileCartographerSessionStatus();
  const refreshedTask = taskStatusRecords.get(taskId) || task;
  if (isSessionBackedCartographerTask(session, taskId)) {
    return refreshedTask;
  }
  if (isTerminalTaskStatus(normalizeTaskStatus(refreshedTask))) {
    return refreshedTask;
  }

  const bridgeReportsTask = (robotState.runningLaunches || []).some(entry =>
    isCartographerLaunch(entry) && entry.taskId === taskId
  );
  const updatedAt = Date.parse(refreshedTask.time || refreshedTask.timestamp || '');
  const oldEnough = Number.isFinite(updatedAt) &&
    Date.now() - updatedAt >= CARTOGRAPHER_TASK_STALE_GRACE_MS;
  if (robotState.jetsonOnline && oldEnough && !bridgeReportsTask) {
    return setCartographerTaskExited(
      taskId, refreshedTask,
      'Cartographer task is not backed by a live session or launch process.'
    );
  }
  return refreshedTask;
}

function getActiveCartographerSessionForSave() {
  const session = reconcileCartographerSessionStatus();
  return session && session.phase === 'mapping' && session.mappingDone !== true ? session : null;
}

function getMappingSessionSummary() {
  const session = reconcileCartographerSessionStatus();
  if (!session) {
    return {
      taskId: '',
      phase: 'idle',
      status: 'idle',
      terminal: true,
      manualTakeoverRequired: false,
      sessionActive: false,
      processExited: false,
      functionalServicesRestored: false,
      cancelRequested: false,
      terminationStatus: '',
      terminationReason: '',
      returnHomeFailureReason: '',
      returnHomeFailureConfirmed: false,
      returnHomeFailureKind: '',
      finishRequestedAt: '',
      finishRequestedAtEpochMs: 0,
      returnHomeStartedAt: '',
      returnHomeStartedAtEpochMs: 0,
      mappingDone: false,
      finalMapSaved: false,
      finalMapSavedAt: '',
      finalMapSaveError: '',
      lastError: '',
      updatedAt: '',
      updatedAtEpochMs: 0,
    };
  }
  const phase = String(session.phase || 'unknown');
  const terminal = session.terminal === true || phase === 'closed';
  const reportedStatus = String(
    session.taskStatus || session.terminationStatus ||
    (terminal ? (phase === 'closed' ? 'succeeded' : 'canceled') : 'running')
  );
  return {
    taskId: typeof session.taskId === 'string' ? session.taskId : '',
    phase,
    status: reportedStatus,
    terminal,
    manualTakeoverRequired: session.manualTakeoverRequired === true,
    sessionActive: typeof session.sessionActive === 'boolean'
      ? session.sessionActive
      : !terminal,
    processExited: session.processExited === true,
    functionalServicesRestored: session.functionalServicesRestored === true,
    cancelRequested: session.cancelRequested === true,
    terminationStatus: session.terminationStatus || '',
    terminationReason: session.terminationReason || '',
    returnHomeFailureReason: session.returnHomeFailureReason || '',
    returnHomeFailureConfirmed: session.returnHomeFailureConfirmed === true,
    returnHomeFailureKind: session.returnHomeFailureKind || '',
    finishRequestedAt: session.finishRequestedAt || '',
    finishRequestedAtEpochMs: Number(session.finishRequestedAtEpochMs || 0),
    returnHomeStartedAt: session.returnHomeStartedAt || '',
    returnHomeStartedAtEpochMs: Number(session.returnHomeStartedAtEpochMs || 0),
    mappingDone: session.mappingDone === true,
    finalMapSaved: session.finalMapSaved === true,
    finalMapSavedAt: session.finalMapSavedAt || '',
    finalMapSaveError: session.finalMapSaveError || '',
    navigationState: session.navigationState || '',
    returnHomeRetry: Number(session.returnHomeRetry || 0),
    returnHomeMaxRetries: Number(session.returnHomeMaxRetries || 0),
    positionErrorMeters: Object.prototype.hasOwnProperty.call(session, 'positionErrorMeters') ? session.positionErrorMeters : null,
      lastError: session.lastError || '',
      updatedAt: session.updatedAt || '',
      updatedAtEpochMs: Number(session.updatedAtEpochMs || 0),
  };
}

loadTaskStatusRecords();
invalidateCartographerTasksFromOtherBoots();

let taskCounter = 0;
let startTimer = null;
let runTimer = null;
let mqttClient = null;
let heartbeatTimer = null;

function sendJson(res, statusCode, payload) {
  res.writeHead(statusCode, { 'Content-Type': 'application/json; charset=utf-8' });
  res.end(JSON.stringify(payload));
}

function buildBaseUrl(req) {
  const hostHeader = req.headers.host || `${HOST}:${PORT}`;
  const forwardedProto = String(req.headers['x-forwarded-proto'] || '').split(',')[0].trim();
  const protocol = forwardedProto || (req.socket.encrypted ? 'https' : 'http');
  return `${protocol}://${hostHeader}`;
}

function getMapFileUrl(req) {
  return `${buildBaseUrl(req)}/api/map/latest.pgm`;
}

function getLiveMapFileUrl(req) {
  return `${buildBaseUrl(req)}/api/map/live.pgm`;
}

function getMapStatusUrl(req) {
  return `${buildBaseUrl(req)}/api/status`;
}

function getMapMetadataUrl(req) {
  return `${buildBaseUrl(req)}/api/map/metadata`;
}

function getMapSnapshot(req) {
  let stats = null;
  try {
    stats = fs.statSync(MAP_FILE_PATH);
  } catch (error) {
    stats = null;
  }

  const mapUpdatedAt = stats ? stats.mtime.toISOString() : '';
  const mapReady = Boolean(stats && stats.isFile());

  if (mapReady && (!mapState.lastSuccessAt || mapState.lastSuccessAt < mapUpdatedAt)) {
    mapState.lastSuccessAt = mapUpdatedAt;
    if (mapState.lastCommandStatus === 'pending') {
      mapState.lastCommandStatus = 'ok';
    }
  }

  return {
    ok: true,
    mapReady,
    mapFormat: MAP_FORMAT,
    mapPath: MAP_FILE_PATH,
    mapUpdatedAt,
    lastAttemptAt: mapState.lastAttemptAt,
    lastSuccessAt: mapState.lastSuccessAt,
    lastError: mapState.lastError,
    lastCommandStatus: mapState.lastCommandStatus,
    fileUrl: getMapFileUrl(req),
    metadataUrl: getMapMetadataUrl(req),
    deviceId: robotState.deviceId,
  };
}

function buildStatePayload(message, extra = {}) {
  return {
    type: 'robot_status',
    deviceId: robotState.deviceId,
    taskId: robotState.activeTaskId,
    status: robotState.status,
    movement: robotState.movement,
    controlMode: robotState.controlMode,
    mqttEnabled: robotState.mqttEnabled,
    mqttConnected: robotState.mqttConnected,
    mqttBrokerUrl: robotState.mqttBrokerUrl,
    mqttCommandTopic: robotState.mqttCommandTopic,
    launchStatus: robotState.launchStatus,
    currentTask: robotState.currentTask,
    currentTaskType: robotState.currentTaskType,
    runningLaunches: robotState.runningLaunches,
    runningNodes: robotState.runningNodes,
    jetsonOnline: robotState.jetsonOnline,
    lastError: robotState.lastError,
    featureStatus: robotState.featureStatus,
    deviceRestart: getDeviceRestartStatus(),
    message,
    timestamp: new Date().toISOString(),
    ...extra,
  };
}

function broadcast(payload) {
  const message = JSON.stringify(payload);
  robotState.history.push({
    time: new Date().toISOString(),
    ...payload,
  });
  if (robotState.history.length > 20) {
    robotState.history.shift();
  }
  recordTaskStatus(payload);

  for (const client of wss.clients) {
    if (client.readyState === 1) {
      client.send(message);
    }
  }
}

function pushState(status, message, extra = {}) {
  robotState.status = status;
  if (Object.prototype.hasOwnProperty.call(extra, 'movement')) {
    robotState.movement = extra.movement;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'launchStatus')) {
    robotState.launchStatus = extra.launchStatus;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'currentTask')) {
    robotState.currentTask = extra.currentTask;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'currentTaskType')) {
    robotState.currentTaskType = extra.currentTaskType;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'runningLaunches')) {
    robotState.runningLaunches = extra.runningLaunches;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'runningNodes')) {
    robotState.runningNodes = extra.runningNodes;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'jetsonOnline')) {
    robotState.jetsonOnline = extra.jetsonOnline;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'lastError')) {
    robotState.lastError = extra.lastError;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'mqttConnected')) {
    robotState.mqttConnected = extra.mqttConnected;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'taskId')) {
    robotState.activeTaskId = extra.taskId;
  }
  if (Object.prototype.hasOwnProperty.call(extra, 'featureStatus')) {
    robotState.featureStatus = extra.featureStatus;
  }

  broadcast(buildStatePayload(message, extra));
}

function getPayloadField(payload, fieldName, fallbackValue) {
  return Object.prototype.hasOwnProperty.call(payload, fieldName) ? payload[fieldName] : fallbackValue;
}

function normalizeFeatureActionAllowed(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value) ||
      typeof value.start !== 'boolean' || typeof value.stop !== 'boolean' ||
      typeof value.targetNavigation !== 'boolean') {
    return null;
  }
  return {
    start: value.start,
    stop: value.stop,
    targetNavigation: value.targetNavigation,
  };
}

function normalizeBodyFollowFeatureStatus(bodyFollow) {
  // It is intentionally optional during the rolling deployment.  A missing
  // status is not equivalent to "follow disabled" and consumers must keep
  // their existing safety fallback until the bridge has been updated.
  if (typeof bodyFollow === 'undefined') return { value: undefined, error: '' };
  if (!bodyFollow || typeof bodyFollow !== 'object' || Array.isArray(bodyFollow)) {
    return { value: null, error: 'Body-follow feature status must be an object.' };
  }

  const state = String(bodyFollow.state || '').trim().toLowerCase();
  const effectiveEnabled = bodyFollow.effectiveEnabled;
  const canEnable = bodyFollow.canEnable;
  const canDisable = bodyFollow.canDisable;
  const source = String(bodyFollow.source || '').trim().toLowerCase();
  const requestedEnabled = bodyFollow.requestedEnabled;
  const controllerApplied = bodyFollow.controllerApplied;
  const pending = bodyFollow.pending;
  const modeSubscriberCount = bodyFollow.modeSubscriberCount;
  const controllerState = bodyFollow.controllerState;
  const lastError = bodyFollow.lastError;
  if (!['stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'].includes(state) ||
      typeof effectiveEnabled !== 'boolean' || typeof canEnable !== 'boolean' ||
      typeof canDisable !== 'boolean' || source !== 'ros_service' ||
      typeof requestedEnabled !== 'boolean' || typeof controllerApplied !== 'boolean' ||
      typeof pending !== 'boolean' || !Number.isInteger(modeSubscriberCount) ||
      modeSubscriberCount < 0 || typeof controllerState !== 'string' ||
      typeof lastError !== 'string') {
    return { value: null, error: 'Body-follow feature status fields are invalid.' };
  }
  if ((state === 'ready' && !effectiveEnabled) ||
      (state !== 'ready' && effectiveEnabled)) {
    return { value: null, error: 'Body-follow feature status contains an inconsistent ready state.' };
  }
  if ((canEnable && state !== 'stopped') || (canDisable && state !== 'ready')) {
    return { value: null, error: 'Body-follow feature status contains inconsistent action permissions.' };
  }
  return {
    value: {
      state,
      effectiveEnabled,
      canEnable,
      canDisable,
      source,
      requestedEnabled,
      controllerApplied,
      pending,
      modeSubscriberCount,
      controllerState,
      lastError,
    },
    error: '',
  };
}

function normalizeRechargeFeatureStatus(recharge) {
  // It is optional during rolling deployment.  Absence is deliberately not
  // converted to "recharge stopped"; the mini-program must stay fail-closed.
  if (typeof recharge === 'undefined') return { value: undefined, error: '' };
  if (!recharge || typeof recharge !== 'object' || Array.isArray(recharge)) {
    return { value: null, error: 'Recharge feature status must be an object.' };
  }

  const state = String(recharge.state || '').trim().toLowerCase();
  const effectiveEnabled = recharge.effectiveEnabled;
  const canEnable = recharge.canEnable;
  const canDisable = recharge.canDisable;
  const source = String(recharge.source || '').trim().toLowerCase();
  const managerStatus = recharge.managerStatus;
  const graphProbeAvailable = recharge.graphProbeAvailable;
  const managerNodeObserved = recharge.managerNodeObserved;
  const statusPublisherObserved = recharge.statusPublisherObserved;
  const statusSourceObserved = recharge.statusSourceObserved;
  const mappingMotionBlocked = recharge.mappingMotionBlocked;
  const mappingGateReceived = recharge.mappingGateReceived;
  const chargingSignalFresh = recharge.chargingSignalFresh;
  const chargingConfirmed = recharge.chargingConfirmed;
  const chargingCurrentAmps = recharge.chargingCurrentAmps;
  const chargingCurrentFresh = recharge.chargingCurrentFresh;
  const lastResult = recharge.lastResult;
  const lastError = recharge.lastError;
  const actionBlockedReason = recharge.actionBlockedReason;
  if (!['stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'].includes(state) ||
      typeof effectiveEnabled !== 'boolean' || typeof canEnable !== 'boolean' ||
      typeof canDisable !== 'boolean' || source !== 'recharge_manager' ||
      typeof managerStatus !== 'string' || typeof graphProbeAvailable !== 'boolean' ||
      typeof managerNodeObserved !== 'boolean' ||
      typeof statusPublisherObserved !== 'boolean' ||
      typeof statusSourceObserved !== 'boolean' ||
      typeof mappingMotionBlocked !== 'boolean' ||
      typeof mappingGateReceived !== 'boolean' ||
      typeof chargingSignalFresh !== 'boolean' ||
      typeof chargingConfirmed !== 'boolean' ||
      !(chargingCurrentAmps === null || typeof chargingCurrentAmps === 'number') ||
      typeof chargingCurrentFresh !== 'boolean' || typeof lastResult !== 'string' ||
      typeof lastError !== 'string' || typeof actionBlockedReason !== 'string') {
    return { value: null, error: 'Recharge feature status fields are invalid.' };
  }
  if ((state === 'ready' && !effectiveEnabled) ||
      (state !== 'ready' && effectiveEnabled)) {
    return { value: null, error: 'Recharge feature status contains an inconsistent ready state.' };
  }
  if ((canEnable && !['stopped', 'failed'].includes(state)) ||
      (canDisable && !['starting', 'ready'].includes(state))) {
    return { value: null, error: 'Recharge feature status contains inconsistent action permissions.' };
  }
  if (!statusSourceObserved && (canEnable || canDisable)) {
    return { value: null, error: 'Recharge actions require an observed manager status source.' };
  }
  return {
    value: {
      state,
      effectiveEnabled,
      canEnable,
      canDisable,
      source,
      managerStatus,
      graphProbeAvailable,
      managerNodeObserved,
      statusPublisherObserved,
      statusSourceObserved,
      mappingMotionBlocked,
      mappingGateReceived,
      chargingSignalFresh,
      chargingConfirmed,
      chargingCurrentAmps,
      chargingCurrentFresh,
      lastResult,
      lastError,
      actionBlockedReason,
    },
    error: '',
  };
}

function normalizeVehicleReadiness(vehicleReadiness) {
  // Optional during a rolling car-side deployment.  Absence must remain
  // distinguishable from a normal ready/stopped mapping state so clients can
  // fail closed instead of silently bypassing the cold-start gate.
  if (typeof vehicleReadiness === 'undefined') return { value: undefined, error: '' };
  if (!vehicleReadiness || typeof vehicleReadiness !== 'object' || Array.isArray(vehicleReadiness)) {
    return { value: null, error: 'Vehicle readiness must be an object.' };
  }
  const state = String(vehicleReadiness.state || '').trim().toLowerCase();
  const mappingStartAllowed = vehicleReadiness.mappingStartAllowed;
  const baseServiceActive = vehicleReadiness.baseServiceActive;
  const scanFresh = vehicleReadiness.scanFresh;
  const odomFresh = vehicleReadiness.odomFresh;
  const odomAdvancing = vehicleReadiness.odomAdvancing;
  const imuFresh = vehicleReadiness.imuFresh;
  const baseToImuTfObserved = vehicleReadiness.baseToImuTfObserved;
  const odomToBaseTfFresh = vehicleReadiness.odomToBaseTfFresh;
  const odomToBaseTfAdvancing = vehicleReadiness.odomToBaseTfAdvancing;
  const message = vehicleReadiness.message;
  const serviceProbeError = vehicleReadiness.serviceProbeError;
  const stableSinceEpochMs = Number(vehicleReadiness.stableSinceEpochMs);
  const readySinceEpochMs = Number(vehicleReadiness.readySinceEpochMs);
  const startedAtEpochMs = Number(vehicleReadiness.startedAtEpochMs);
  const observedAtEpochMs = Number(vehicleReadiness.observedAtEpochMs);
  if (!['initializing', 'ready', 'failed', 'unknown'].includes(state) ||
      typeof mappingStartAllowed !== 'boolean' ||
      typeof baseServiceActive !== 'boolean' || typeof scanFresh !== 'boolean' ||
      typeof odomFresh !== 'boolean' || typeof odomAdvancing !== 'boolean' ||
      typeof imuFresh !== 'boolean' || typeof baseToImuTfObserved !== 'boolean' ||
      typeof odomToBaseTfFresh !== 'boolean' ||
      typeof odomToBaseTfAdvancing !== 'boolean' ||
      typeof message !== 'string' || typeof serviceProbeError !== 'string' ||
      !Number.isInteger(stableSinceEpochMs) || stableSinceEpochMs < 0 ||
      !Number.isInteger(readySinceEpochMs) || readySinceEpochMs < 0 ||
      !Number.isInteger(startedAtEpochMs) || startedAtEpochMs < 0 ||
      !Number.isInteger(observedAtEpochMs) || observedAtEpochMs <= 0 ||
      (state === 'ready') !== mappingStartAllowed) {
    return { value: null, error: 'Vehicle readiness fields are invalid.' };
  }
  return {
    value: {
      state,
      mappingStartAllowed,
      baseServiceActive,
      scanFresh,
      odomFresh,
      odomAdvancing,
      imuFresh,
      baseToImuTfObserved,
      odomToBaseTfFresh,
      odomToBaseTfAdvancing,
      message,
      serviceProbeError,
      stableSinceEpochMs,
      readySinceEpochMs,
      startedAtEpochMs,
      observedAtEpochMs,
    },
    error: '',
  };
}

function normalizeMappingFeatureActionAllowed(value) {
  if (!value || typeof value !== 'object' || Array.isArray(value) ||
      typeof value.start !== 'boolean' || typeof value.pause !== 'boolean' ||
      typeof value.resume !== 'boolean' || typeof value.finish !== 'boolean') {
    return null;
  }
  return {
    start: value.start,
    pause: value.pause,
    resume: value.resume,
    finish: value.finish,
  };
}

function normalizeMappingSessionFeatureStatus(mappingSession) {
  // Optional during rolling deployment.  Missing does not mean that mapping
  // stopped; it preserves the existing mapping UI fallback until car-side
  // status support is present.
  if (typeof mappingSession === 'undefined') return { value: undefined, error: '' };
  if (!mappingSession || typeof mappingSession !== 'object' || Array.isArray(mappingSession)) {
    return { value: null, error: 'Mapping feature status must be an object.' };
  }
  const state = String(mappingSession.state || '').trim().toLowerCase();
  const effectiveEnabled = mappingSession.effectiveEnabled;
  const source = String(mappingSession.source || '').trim().toLowerCase();
  const taskId = mappingSession.taskId;
  const phase = mappingSession.phase;
  const rawVoiceState = String(mappingSession.voiceState || '').trim();
  const voiceState = rawVoiceState.toUpperCase() === 'UNKNOWN'
    ? 'unknown'
    : rawVoiceState.toUpperCase();
  const voiceStateObserved = mappingSession.voiceStateObserved;
  const voiceStatePublisherObserved = mappingSession.voiceStatePublisherObserved;
  const sessionObserved = mappingSession.sessionObserved;
  const sessionFresh = mappingSession.sessionFresh;
  const sessionActive = mappingSession.sessionActive;
  const terminal = mappingSession.terminal;
  const functionalServicesRestored = mappingSession.functionalServicesRestored;
  const motionBlocked = mappingSession.motionBlocked;
  const paused = mappingSession.paused;
  const mappingDone = mappingSession.mappingDone;
  const finalMapSaved = mappingSession.finalMapSaved;
  const actionAllowed = normalizeMappingFeatureActionAllowed(mappingSession.actionAllowed);
  const lastError = mappingSession.lastError;
  const actionBlockedReason = mappingSession.actionBlockedReason;
  const sourceUpdatedAtEpochMs = Number(mappingSession.sourceUpdatedAtEpochMs);
  if (!['stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'].includes(state) ||
      typeof effectiveEnabled !== 'boolean' ||
      !['combined', 'cartographer_session', 'voice_cartographer_bridge', 'unknown'].includes(source) ||
      typeof taskId !== 'string' || typeof phase !== 'string' ||
      !['IDLE', 'STARTING', 'RUNNING', 'PAUSED', 'ENDING', 'unknown'].includes(voiceState) ||
      typeof voiceStateObserved !== 'boolean' ||
      typeof voiceStatePublisherObserved !== 'boolean' ||
      typeof sessionObserved !== 'boolean' || typeof sessionFresh !== 'boolean' ||
      typeof sessionActive !== 'boolean' || typeof terminal !== 'boolean' ||
      typeof functionalServicesRestored !== 'boolean' ||
      typeof motionBlocked !== 'boolean' || typeof paused !== 'boolean' ||
      typeof mappingDone !== 'boolean' || typeof finalMapSaved !== 'boolean' ||
      !actionAllowed || typeof lastError !== 'string' ||
      typeof actionBlockedReason !== 'string' ||
      !Number.isInteger(sourceUpdatedAtEpochMs) || sourceUpdatedAtEpochMs <= 0) {
    return { value: null, error: 'Mapping feature status fields are invalid.' };
  }
  if ((state === 'ready' && !effectiveEnabled) ||
      (state !== 'ready' && effectiveEnabled)) {
    return { value: null, error: 'Mapping feature status contains an inconsistent ready state.' };
  }
  if ((actionAllowed.start && !['stopped', 'failed'].includes(state)) ||
      (actionAllowed.pause && (state !== 'ready' || paused)) ||
      (actionAllowed.resume && (state !== 'ready' || !paused)) ||
      (actionAllowed.finish && state !== 'ready')) {
    return { value: null, error: 'Mapping feature status contains inconsistent action permissions.' };
  }
  if (source === 'unknown' && Object.values(actionAllowed).some(Boolean)) {
    return { value: null, error: 'Unknown mapping source cannot grant an action permission.' };
  }
  return {
    value: {
      state,
      effectiveEnabled,
      source,
      taskId,
      phase,
      voiceState,
      voiceStateObserved,
      voiceStatePublisherObserved,
      sessionObserved,
      sessionFresh,
      sessionActive,
      terminal,
      functionalServicesRestored,
      motionBlocked,
      paused,
      mappingDone,
      finalMapSaved,
      actionAllowed,
      lastError,
      actionBlockedReason,
      sourceUpdatedAtEpochMs,
    },
    error: '',
  };
}

function normalizeTargetNavigationFeatureStatus(targetNavigation) {
  // Only normal I/J/K voice goals appear here.  It is intentionally optional
  // while the command-recognition publisher is being rolled out.
  if (typeof targetNavigation === 'undefined') return { value: undefined, error: '' };
  if (!targetNavigation || typeof targetNavigation !== 'object' || Array.isArray(targetNavigation)) {
    return { value: null, error: 'Target-navigation feature status must be an object.' };
  }
  const state = String(targetNavigation.state || '').trim().toLowerCase();
  const active = targetNavigation.active;
  const interlockActive = targetNavigation.interlockActive;
  const recoveryRequired = targetNavigation.recoveryRequired;
  const sessionObserved = targetNavigation.sessionObserved;
  const sessionTerminal = targetNavigation.sessionTerminal;
  const source = String(targetNavigation.source || '').trim().toLowerCase();
  const goalId = targetNavigation.goalId;
  const goal = targetNavigation.goal;
  const canCancel = targetNavigation.canCancel;
  const publisherObserved = targetNavigation.publisherObserved;
  const statusFresh = targetNavigation.statusFresh;
  const message = targetNavigation.message;
  const lastError = targetNavigation.lastError;
  const sourceUpdatedAtEpochMs = Number(targetNavigation.sourceUpdatedAtEpochMs);
  if (!['unknown', 'starting', 'navigating', 'succeeded', 'failed', 'canceled'].includes(state) ||
       typeof active !== 'boolean' ||
      typeof interlockActive !== 'boolean' || typeof recoveryRequired !== 'boolean' ||
      typeof sessionObserved !== 'boolean' || typeof sessionTerminal !== 'boolean' ||
      !['voice_command_recognition', 'unknown'].includes(source) ||
      typeof goalId !== 'string' || typeof canCancel !== 'boolean' || canCancel !== false ||
      typeof publisherObserved !== 'boolean' || typeof statusFresh !== 'boolean' ||
      typeof message !== 'string' || typeof lastError !== 'string' ||
       !Number.isInteger(sourceUpdatedAtEpochMs) || sourceUpdatedAtEpochMs <= 0 ||
       (['starting', 'navigating'].includes(state)) !== active ||
      (active && !interlockActive) ||
      (recoveryRequired && !interlockActive) ||
      (['succeeded', 'failed', 'canceled'].includes(state) &&
        (interlockActive || recoveryRequired)) ||
      (source === 'unknown' && state !== 'unknown')) {
    return { value: null, error: 'Target-navigation feature status fields are invalid.' };
  }
  let normalizedGoal = null;
  if (goal !== null) {
    if (!goal || typeof goal !== 'object' || Array.isArray(goal)) {
      return { value: null, error: 'Target-navigation goal must be an object.' };
    }
    const values = [goal.x, goal.y, goal.orientationZ, goal.orientationW];
    if (typeof goal.frameId !== 'string' || !goal.frameId.trim() ||
        typeof goal.label !== 'string' ||
        values.some(value => typeof value !== 'number' || !Number.isFinite(value))) {
      return { value: null, error: 'Target-navigation goal fields are invalid.' };
    }
    normalizedGoal = {
      frameId: goal.frameId.trim().replace(/^\/+/, ''),
      x: goal.x,
      y: goal.y,
      orientationZ: goal.orientationZ,
      orientationW: goal.orientationW,
      label: goal.label,
    };
  }
  return {
    value: {
      state,
      active,
      interlockActive,
      recoveryRequired,
      sessionObserved,
      sessionTerminal,
      source,
      goalId,
      goal: normalizedGoal,
      canCancel,
      publisherObserved,
      statusFresh,
      message,
      lastError,
      sourceUpdatedAtEpochMs,
    },
    error: '',
  };
}

function normalizeNavigationFeatureStatus(payload = {}) {
  if (!payload || typeof payload !== 'object' || Array.isArray(payload)) {
    return { value: null, error: 'Feature status payload must be an object.' };
  }

  const schemaVersion = Number(payload.schemaVersion);
  const deviceId = String(payload.deviceId || '').trim();
  const bootId = String(payload.bootId || '').trim();
  const revision = Number(payload.revision);
  const availability = String(payload.availability || '').trim().toLowerCase();
  const observedAtEpochMs = Number(payload.observedAtEpochMs);
  const navigation = payload.navigationSystem;
  const bodyFollow = payload.bodyFollow;
  const recharge = payload.recharge;
  const vehicleReadiness = payload.vehicleReadiness;
  const mappingSession = payload.mappingSession;
  const targetNavigation = payload.targetNavigation;

  if (schemaVersion !== 1 || deviceId !== MQTT_DEVICE_ID || !bootId ||
      !Number.isInteger(revision) || revision < 0 ||
      !['ready', 'offline', 'unknown'].includes(availability) ||
      !Number.isInteger(observedAtEpochMs) || observedAtEpochMs <= 0 ||
      !navigation || typeof navigation !== 'object' || Array.isArray(navigation)) {
    return { value: null, error: 'Feature status envelope is invalid.' };
  }

  const state = String(navigation.state || '').trim().toLowerCase();
  const effectiveEnabled = navigation.effectiveEnabled;
  const launchObserved = navigation.launchObserved;
  const actionServerReady = navigation.actionServerReady;
  const poseReady = navigation.poseReady;
  // Optional diagnostic fields are accepted with legacy fallbacks so a bridge
  // and control service can be rolled out independently.
  const poseAvailable = Object.prototype.hasOwnProperty.call(navigation, 'poseAvailable')
    ? navigation.poseAvailable
    : poseReady;
  const poseFresh = Object.prototype.hasOwnProperty.call(navigation, 'poseFresh')
    ? navigation.poseFresh
    : poseReady;
  const localizationHealth = String(
    navigation.localizationHealth || (poseReady === true ? 'ready' : 'unknown')
  ).trim().toLowerCase();
  const localizationProbeAvailable = Object.prototype.hasOwnProperty.call(
    navigation, 'localizationProbeAvailable'
  ) ? navigation.localizationProbeAvailable : false;
  const amclNodeObserved = Object.prototype.hasOwnProperty.call(
    navigation, 'amclNodeObserved'
  ) ? navigation.amclNodeObserved : false;
  const posePublisherObserved = Object.prototype.hasOwnProperty.call(
    navigation, 'posePublisherObserved'
  ) ? navigation.posePublisherObserved : false;
  const localizationSourceObserved = Object.prototype.hasOwnProperty.call(
    navigation, 'localizationSourceObserved'
  ) ? navigation.localizationSourceObserved : poseReady;
  const safetyFieldPresent = [
    'safetyReady', 'guardHealthy', 'guardStatusFresh',
    'interlockReady', 'interlockStatusFresh',
  ].some(field => Object.prototype.hasOwnProperty.call(navigation, field));
  const safetyProbeAvailable = Object.prototype.hasOwnProperty.call(
    navigation, 'safetyProbeAvailable'
  ) ? navigation.safetyProbeAvailable : safetyFieldPresent;
  const safetyReady = Object.prototype.hasOwnProperty.call(navigation, 'safetyReady')
    ? navigation.safetyReady : false;
  const guardHealthy = Object.prototype.hasOwnProperty.call(navigation, 'guardHealthy')
    ? navigation.guardHealthy : false;
  const guardStatusFresh = Object.prototype.hasOwnProperty.call(navigation, 'guardStatusFresh')
    ? navigation.guardStatusFresh : false;
  const guardAnomaly = Object.prototype.hasOwnProperty.call(navigation, 'guardAnomaly')
    ? navigation.guardAnomaly : null;
  const guardMode = Object.prototype.hasOwnProperty.call(navigation, 'guardMode')
    ? navigation.guardMode : '';
  const guardTrustedOverride = Object.prototype.hasOwnProperty.call(
    navigation, 'guardTrustedOverride'
  ) ? navigation.guardTrustedOverride : false;
  const interlockReady = Object.prototype.hasOwnProperty.call(navigation, 'interlockReady')
    ? navigation.interlockReady : false;
  const interlockStatusFresh = Object.prototype.hasOwnProperty.call(
    navigation, 'interlockStatusFresh'
  ) ? navigation.interlockStatusFresh : false;
  const interlockBlocked = Object.prototype.hasOwnProperty.call(
    navigation, 'interlockBlocked'
  ) ? navigation.interlockBlocked : null;
  const safetyError = Object.prototype.hasOwnProperty.call(navigation, 'safetyError')
    ? navigation.safetyError : String(navigation.lastError || '');
  const navigationMessage = Object.prototype.hasOwnProperty.call(navigation, 'message')
    ? navigation.message : String(navigation.lastError || '');
  const canEnable = Object.prototype.hasOwnProperty.call(navigation, 'canEnable')
    ? navigation.canEnable : false;
  const canDisable = Object.prototype.hasOwnProperty.call(navigation, 'canDisable')
    ? navigation.canDisable : false;
  const navigationStateSinceEpochMs = Object.prototype.hasOwnProperty.call(
    navigation, 'stateSinceEpochMs'
  ) ? Number(navigation.stateSinceEpochMs) : 0;
  const source = String(navigation.source || 'unknown').trim().toLowerCase();
  const actionAllowed = Object.prototype.hasOwnProperty.call(navigation, 'actionAllowed')
    ? normalizeFeatureActionAllowed(navigation.actionAllowed)
    : { start: false, stop: false, targetNavigation: false };
  const actionBlockedReason = Object.prototype.hasOwnProperty.call(navigation, 'actionBlockedReason')
    ? navigation.actionBlockedReason : '';
  const lastError = navigation.lastError;
  if (!['stopped', 'starting', 'ready', 'stopping', 'failed', 'unknown'].includes(state) ||
      typeof effectiveEnabled !== 'boolean' ||
      typeof launchObserved !== 'boolean' ||
      typeof actionServerReady !== 'boolean' ||
      typeof poseReady !== 'boolean' ||
      typeof poseAvailable !== 'boolean' ||
      typeof poseFresh !== 'boolean' ||
      typeof localizationProbeAvailable !== 'boolean' ||
      typeof amclNodeObserved !== 'boolean' ||
      typeof posePublisherObserved !== 'boolean' ||
      typeof localizationSourceObserved !== 'boolean' ||
      typeof safetyProbeAvailable !== 'boolean' ||
      typeof safetyReady !== 'boolean' ||
      typeof guardHealthy !== 'boolean' ||
      typeof guardStatusFresh !== 'boolean' ||
      (guardAnomaly !== null && typeof guardAnomaly !== 'boolean') ||
      typeof guardMode !== 'string' ||
      typeof guardTrustedOverride !== 'boolean' ||
      (guardTrustedOverride && guardMode !== 'TRUSTED_OVERRIDE') ||
      typeof interlockReady !== 'boolean' ||
      typeof interlockStatusFresh !== 'boolean' ||
      (interlockBlocked !== null && typeof interlockBlocked !== 'boolean') ||
      typeof safetyError !== 'string' ||
      typeof navigationMessage !== 'string' ||
      typeof canEnable !== 'boolean' ||
      typeof canDisable !== 'boolean' ||
      !['bridge', 'voice', 'none', 'unknown'].includes(source) ||
      !actionAllowed ||
      typeof actionBlockedReason !== 'string' ||
      (navigationStateSinceEpochMs !== 0 &&
       (!Number.isInteger(navigationStateSinceEpochMs) ||
        navigationStateSinceEpochMs <= 0)) ||
      typeof lastError !== 'string') {
    return { value: null, error: 'Navigation feature status fields are invalid.' };
  }
  if (!['unknown', 'starting', 'ready', 'failed'].includes(localizationHealth)) {
    return { value: null, error: 'Navigation localization health is invalid.' };
  }
  if (safetyReady && (!guardHealthy || !guardStatusFresh ||
      !interlockReady || !interlockStatusFresh || safetyError)) {
    return { value: null, error: 'Navigation safety status contains an inconsistent ready state.' };
  }

  const safetyReadyForProtocol = !safetyFieldPresent ||
    (safetyProbeAvailable && safetyReady);
  const isReady = state === 'ready' && effectiveEnabled &&
    actionServerReady && poseReady && safetyReadyForProtocol && !lastError;
  if ((state === 'ready' && !isReady) || (state !== 'ready' && effectiveEnabled)) {
    return { value: null, error: 'Navigation feature status contains an inconsistent ready state.' };
  }
  if ((canEnable && state !== 'stopped') ||
      (canDisable && !['starting', 'ready', 'failed'].includes(state)) ||
      (actionAllowed.start && !canEnable) ||
      (actionAllowed.stop && !canDisable)) {
    return { value: null, error: 'Navigation feature status contains inconsistent action permissions.' };
  }
  if (availability !== 'ready' &&
      (canEnable || canDisable || actionAllowed.start || actionAllowed.stop ||
       actionAllowed.targetNavigation)) {
    return { value: null, error: 'Offline or unknown navigation status cannot grant an action permission.' };
  }
  if (actionAllowed.targetNavigation && !isReady) {
    return { value: null, error: 'Target navigation requires a fully ready navigation system.' };
  }
  const normalizedBodyFollow = normalizeBodyFollowFeatureStatus(bodyFollow);
  if (normalizedBodyFollow.error) return normalizedBodyFollow;
  const normalizedRecharge = normalizeRechargeFeatureStatus(recharge);
  if (normalizedRecharge.error) return normalizedRecharge;
  const normalizedVehicleReadiness = normalizeVehicleReadiness(vehicleReadiness);
  if (normalizedVehicleReadiness.error) return normalizedVehicleReadiness;
  const normalizedMappingSession = normalizeMappingSessionFeatureStatus(mappingSession);
  if (normalizedMappingSession.error) return normalizedMappingSession;
  const normalizedTargetNavigation = normalizeTargetNavigationFeatureStatus(targetNavigation);
  if (normalizedTargetNavigation.error) return normalizedTargetNavigation;

  return {
    value: {
      schemaVersion,
      deviceId,
      bootId,
      revision,
      availability,
      observedAtEpochMs,
      navigationSystem: {
        state,
        effectiveEnabled,
        launchObserved,
        actionServerReady,
        poseReady,
        poseAvailable,
        poseFresh,
        localizationHealth,
        localizationProbeAvailable,
        amclNodeObserved,
        posePublisherObserved,
        localizationSourceObserved,
        safetyProbeAvailable,
        safetyReady,
        guardHealthy,
        guardStatusFresh,
        guardAnomaly,
        guardMode,
        guardTrustedOverride,
        interlockReady,
        interlockStatusFresh,
        interlockBlocked,
        safetyError,
        message: navigationMessage,
        stateSinceEpochMs: navigationStateSinceEpochMs,
        canEnable,
        canDisable,
        source,
        actionAllowed,
        actionBlockedReason,
        lastError,
      },
      ...(typeof normalizedBodyFollow.value === 'undefined'
        ? {}
        : { bodyFollow: normalizedBodyFollow.value }),
      ...(typeof normalizedRecharge.value === 'undefined'
        ? {}
        : { recharge: normalizedRecharge.value }),
      ...(typeof normalizedVehicleReadiness.value === 'undefined'
        ? {}
        : { vehicleReadiness: normalizedVehicleReadiness.value }),
      ...(typeof normalizedMappingSession.value === 'undefined'
        ? {}
        : { mappingSession: normalizedMappingSession.value }),
      ...(typeof normalizedTargetNavigation.value === 'undefined'
        ? {}
        : { targetNavigation: normalizedTargetNavigation.value }),
    },
    error: '',
  };
}

function navigationFeatureSignature(featureStatus) {
  if (!featureStatus) return '';
  const navigation = featureStatus.navigationSystem || {};
  const vehicleReadiness = featureStatus.vehicleReadiness || null;
  const mappingSession = featureStatus.mappingSession || null;
  const targetNavigation = featureStatus.targetNavigation || null;
  // Source timestamps intentionally do not participate in the semantic
  // signature.  The bridge may refresh them on every observation while the
  // task state remains unchanged; including them would manufacture revisions
  // and WebSocket events continuously.
  const mappingSemantic = mappingSession ? {
    ...mappingSession,
    sourceUpdatedAtEpochMs: undefined,
  } : null;
  const targetSemantic = targetNavigation ? {
    ...targetNavigation,
    sourceUpdatedAtEpochMs: undefined,
  } : null;
  const vehicleReadinessSemantic = vehicleReadiness ? {
    ...vehicleReadiness,
    // This timestamp is a heartbeat only.  It must not manufacture a new
    // revision while readiness semantics are unchanged.
    observedAtEpochMs: undefined,
  } : null;
  return JSON.stringify({
    availability: featureStatus.availability,
    state: navigation.state,
    effectiveEnabled: navigation.effectiveEnabled,
    launchObserved: navigation.launchObserved,
    actionServerReady: navigation.actionServerReady,
    poseReady: navigation.poseReady,
    poseAvailable: navigation.poseAvailable,
    poseFresh: navigation.poseFresh,
    localizationHealth: navigation.localizationHealth,
    localizationProbeAvailable: navigation.localizationProbeAvailable,
    amclNodeObserved: navigation.amclNodeObserved,
    posePublisherObserved: navigation.posePublisherObserved,
    localizationSourceObserved: navigation.localizationSourceObserved,
    safetyProbeAvailable: navigation.safetyProbeAvailable,
    safetyReady: navigation.safetyReady,
    guardHealthy: navigation.guardHealthy,
    guardStatusFresh: navigation.guardStatusFresh,
    guardAnomaly: navigation.guardAnomaly,
    guardMode: navigation.guardMode,
    guardTrustedOverride: navigation.guardTrustedOverride,
    interlockReady: navigation.interlockReady,
    interlockStatusFresh: navigation.interlockStatusFresh,
    interlockBlocked: navigation.interlockBlocked,
    safetyError: navigation.safetyError,
    message: navigation.message,
    canEnable: navigation.canEnable,
    canDisable: navigation.canDisable,
    source: navigation.source,
    actionAllowed: navigation.actionAllowed,
    actionBlockedReason: navigation.actionBlockedReason,
    lastError: navigation.lastError,
    bodyFollow: featureStatus.bodyFollow || null,
    recharge: featureStatus.recharge || null,
    vehicleReadiness: vehicleReadinessSemantic,
    mappingSession: mappingSemantic,
    targetNavigation: targetSemantic,
  });
}

function shouldAcceptNavigationFeatureStatus(candidate, current) {
  if (!current || typeof current !== 'object') return true;
  if (candidate.bootId !== current.bootId) return true;
  // A broker Last Will is configured when the bridge connects and therefore
  // may carry an older revision than the last live retained snapshot.  Its
  // availability transition is still safer than presenting stale readiness.
  if (candidate.availability === 'offline') return true;
  if (current.availability === 'offline' && candidate.availability !== 'offline') return true;
  if (candidate.revision > current.revision) return true;
  if (candidate.revision < current.revision) return false;

  // A reconnect can legitimately keep the same revision while changing only
  // availability.  Accept the transition so the HTTP API never reports a
  // stale "ready" state.
  if (candidate.availability !== current.availability) return true;

  // Equal revision but different content: the bridge bumps its revision only for
  // its own (narrower) semantic field set, while this cache signs a wider set, so
  // a content change could otherwise be rejected as "stale" forever.  Observed
  // 2026-09-16 as a permanent "[navigation-feature] ignored stale status" loop
  // that froze the mini-program's mapping gate and made "开始建图" fail.
  // The newest observation always wins; strictly older revisions are still
  // rejected by the checks above.
  return true;
}

function getMqttServiceInfo(payload = {}) {
  const servicePath = String(payload.servicePath || '').trim();
  return {
    isServiceCall: payload.action === 'call_service' && Boolean(servicePath),
    servicePath,
    serviceType: String(payload.serviceType || '').trim(),
  };
}

function createTaskId() {
  taskCounter = (taskCounter + 1) % 1000;
  return 'task-' + Date.now().toString(36) + '-' + String(taskCounter).padStart(3, '0');
}

function buildLaunchCommandPayload(action, body = {}) {
  const packageName = (body.package || DEFAULT_LAUNCH_PACKAGE || '').trim();
  const launchFile = (body.launchFile || DEFAULT_LAUNCH_FILE || '').trim();
  if (!launchFile) {
    throw new Error('No launch file configured for MQTT launch command');
  }
  if (!packageName) {
    throw new Error('No launch package configured for MQTT launch command');
  }
  if (!ALLOWED_LAUNCH_TARGETS.has(buildTargetKey(packageName, launchFile))) {
    throw new Error(`Launch target is not in whitelist: ${packageName}/${launchFile}`);
  }

  const requestId = createTaskId();
  const targetKey = buildTargetKey(packageName, launchFile);
  const isVideoLaunch = VIDEO_LAUNCH_TARGETS.has(targetKey);
  const suppliedVideoSessionId = String(
    body.videoSessionId || body.sessionId || body.ownerTaskId || ''
  ).trim();
  const videoSessionId = isVideoLaunch
    ? (suppliedVideoSessionId || (action === 'start_launch' ? requestId : ''))
    : '';

  return {
    requestId,
    deviceId: robotState.deviceId,
    action,
    package: packageName,
    launchFile,
    args: body.args || {},
    ...(isVideoLaunch ? {
      videoSessionId,
      forceStop: body.forceStop === true || body.force === true,
    } : {}),
    source: 'demo-control',
    timestamp: new Date().toISOString(),
  };
}

function buildNodeCommandPayload(action, body = {}) {
  const packageName = (body.package || DEFAULT_NODE_PACKAGE || '').trim();
  const nodeExecutable = (body.nodeExecutable || DEFAULT_NODE_EXECUTABLE || '').trim();
  if (!packageName) {
    throw new Error('No node package configured for MQTT node command');
  }
  if (!nodeExecutable) {
    throw new Error('No node executable configured for MQTT node command');
  }
  if (!ALLOWED_NODE_TARGETS.has(buildTargetKey(packageName, nodeExecutable))) {
    throw new Error(`Node target is not in whitelist: ${packageName}/${nodeExecutable}`);
  }

  return {
    requestId: createTaskId(),
    deviceId: robotState.deviceId,
    action,
    package: packageName,
    nodeExecutable,
    args: body.args || {},
    source: 'demo-control',
    timestamp: new Date().toISOString(),
  };
}

function buildServiceCommandPayload(body = {}) {
  const servicePath = String(body.servicePath || '').trim();
  const serviceType = String(body.serviceType || '').trim();
  const serviceArgs = body.serviceArgs || {};
  if (!servicePath) {
    throw new Error('No service path configured for MQTT service command');
  }
  if (!serviceType) {
    throw new Error('No service type configured for MQTT service command');
  }
  if (!ALLOWED_SERVICE_TARGETS.has(buildTargetKey(servicePath, serviceType))) {
    throw new Error(`Service target is not in whitelist: ${servicePath}/${serviceType}`);
  }
  if (serviceArgs === null || typeof serviceArgs !== 'object' || Array.isArray(serviceArgs)) {
    throw new Error('serviceArgs must be an object');
  }

  return {
    requestId: createTaskId(),
    deviceId: robotState.deviceId,
    action: 'call_service',
    servicePath,
    serviceType,
    serviceArgs,
    source: 'demo-control',
    timestamp: new Date().toISOString(),
  };
}

function markMapCommandAttempt() {
  mapState.lastAttemptAt = new Date().toISOString();
  mapState.lastCommandStatus = 'pending';
  mapState.lastError = '';
}

function markMapCommandFailed(message) {
  mapState.lastCommandStatus = 'failed';
  mapState.lastError = message;
}

function markMapCommandCompleted() {
  mapState.lastCommandStatus = 'ok';
  mapState.lastError = '';
  try {
    const stats = fs.statSync(MAP_FILE_PATH);
    if (stats.isFile()) {
      mapState.lastSuccessAt = stats.mtime.toISOString();
    }
  } catch (error) {
    // The service can report completion slightly before the file is visible.
  }
}

function publishMqttCommand(payload) {
  return new Promise((resolve, reject) => {
    if (!MQTT_ENABLED) {
      reject(new Error('MQTT is disabled'));
      return;
    }
    if (!mqttClient || !robotState.mqttConnected) {
      reject(new Error('MQTT broker is not connected'));
      return;
    }

    mqttClient.publish(MQTT_COMMAND_TOPIC, JSON.stringify(payload), { qos: 1 }, error => {
      if (error) {
        reject(error);
        return;
      }
      resolve();
    });
  });
}

function resetHeartbeatTimer() {
  if (heartbeatTimer) {
    clearTimeout(heartbeatTimer);
  }
  heartbeatTimer = setTimeout(() => {
    pushState(robotState.status, 'Jetson heartbeat timeout.', {
      jetsonOnline: false,
    });
  }, 15000);
}

function connectMqtt() {
  if (!MQTT_ENABLED) {
    return;
  }

  mqttClient = mqtt.connect(MQTT_BROKER_URL, {
    clientId: MQTT_CLIENT_ID,
    reconnectPeriod: 2000,
  });

  mqttClient.on('connect', () => {
    const topics = [
      MQTT_STATUS_TOPIC,
      MQTT_HEARTBEAT_TOPIC,
      MQTT_ERROR_TOPIC,
      MQTT_FEATURE_STATUS_TOPIC,
    ];
    mqttClient.subscribe(topics, { qos: 1 }, error => {
      if (error) {
        pushState(robotState.status, `MQTT subscribe failed: ${error.message}`, {
          mqttConnected: true,
          lastError: error.message,
        });
        return;
      }

      pushState(robotState.status, 'MQTT broker connected.', {
        mqttConnected: true,
      });
    });
  });

  mqttClient.on('reconnect', () => {
    pushState(robotState.status, 'MQTT reconnecting...', {
      mqttConnected: false,
    });
  });

  mqttClient.on('close', () => {
    pushState(robotState.status, 'MQTT connection closed.', {
      mqttConnected: false,
      jetsonOnline: false,
    });
  });

  mqttClient.on('error', error => {
    pushState(robotState.status, `MQTT error: ${error.message}`, {
      mqttConnected: false,
      lastError: error.message,
    });
  });

  mqttClient.on('message', (topic, buffer) => {
    let payload;
    try {
      payload = JSON.parse(buffer.toString());
    } catch (error) {
      pushState(robotState.status, 'Received invalid MQTT JSON payload.', {
        lastError: error.message,
      });
      return;
    }

    if (topic === MQTT_FEATURE_STATUS_TOPIC) {
      const normalized = normalizeNavigationFeatureStatus(payload);
      if (!normalized.value) {
        // Do not overwrite a previously valid retained snapshot with malformed
        // MQTT data.  The mini-program will naturally mark it stale if no
        // subsequent valid probe arrives.
        console.warn('[navigation-feature] ignored invalid status:', normalized.error);
        return;
      }
      if (!shouldAcceptNavigationFeatureStatus(normalized.value, robotState.featureStatus)) {
        console.warn('[navigation-feature] ignored stale status:', {
          bootId: normalized.value.bootId,
          revision: normalized.value.revision,
        });
        return;
      }

      const current = robotState.featureStatus;
      const semanticChanged = !current ||
        normalized.value.bootId !== current.bootId ||
        normalized.value.revision !== current.revision ||
        navigationFeatureSignature(normalized.value) !== navigationFeatureSignature(current);
      if (semanticChanged) {
        pushState(robotState.status, 'Navigation feature status updated.', {
          jetsonOnline: true,
          featureStatus: normalized.value,
        });
      } else {
        // Same version and semantics: retain its new observation time without
        // adding a WebSocket/history event every probe interval.
        robotState.featureStatus = normalized.value;
      }
      return;
    }

    if (topic === MQTT_HEARTBEAT_TOPIC) {
      resetHeartbeatTimer();
      pushState(robotState.status, payload.message || 'Jetson heartbeat received.', {
        jetsonOnline: true,
        runningLaunches: getPayloadField(payload, 'runningLaunches', robotState.runningLaunches),
        runningNodes: getPayloadField(payload, 'runningNodes', robotState.runningNodes),
        currentTask: getPayloadField(payload, 'currentTask', robotState.currentTask),
        currentTaskType: getPayloadField(payload, 'currentTaskType', robotState.currentTaskType),
        launchStatus: getPayloadField(payload, 'launchStatus', robotState.launchStatus),
        lastError: getPayloadField(payload, 'lastError', robotState.lastError),
      });
      return;
    }

    if (topic === MQTT_ERROR_TOPIC) {
      const service = getMqttServiceInfo(payload);
      if (service.isServiceCall) {
        const errorMessage = payload.error || payload.message || 'Unknown MQTT service error';
        if (service.servicePath === MAP_SERVICE_PATH) {
          markMapCommandFailed(errorMessage);
        }
        pushState('failed', errorMessage, {
          jetsonOnline: true,
          currentTask: service.servicePath,
          currentTaskType: 'service',
          launchStatus: 'failed',
          lastError: errorMessage,
          runningLaunches: getPayloadField(payload, 'runningLaunches', robotState.runningLaunches),
          runningNodes: getPayloadField(payload, 'runningNodes', robotState.runningNodes),
          taskId: getPayloadField(payload, 'requestId', robotState.activeTaskId),
          action: 'call_service',
          servicePath: service.servicePath,
          serviceType: service.serviceType,
          serviceSuccess: false,
          serviceResult: String(payload.serviceResult || ''),
        });
        return;
      }
      if (robotState.currentTaskType === 'service' && robotState.currentTask === MAP_SERVICE_PATH) {
        markMapCommandFailed(payload.error || payload.message || 'Unknown MQTT error');
      }
      pushState(robotState.status, payload.message || 'Jetson reported an error.', {
        jetsonOnline: true,
        lastError: payload.error || payload.message || 'Unknown MQTT error',
        currentTask: getPayloadField(payload, 'currentTask', robotState.currentTask),
        currentTaskType: getPayloadField(payload, 'currentTaskType', robotState.currentTaskType),
        launchStatus: getPayloadField(payload, 'launchStatus', 'failed'),
        runningLaunches: getPayloadField(payload, 'runningLaunches', robotState.runningLaunches),
        runningNodes: getPayloadField(payload, 'runningNodes', robotState.runningNodes),
      });
      return;
    }

    if (topic === MQTT_STATUS_TOPIC) {
      const service = getMqttServiceInfo(payload);
      if (service.isServiceCall) {
        const reportedStatus = String(payload.status || '').trim().toLowerCase();
        const serviceError = String(payload.lastError || '').trim();
        if (service.servicePath === MAP_SERVICE_PATH) {
          if (reportedStatus === 'completed' && payload.serviceSuccess === true) {
            markMapCommandCompleted();
          } else if (reportedStatus === 'failed' || serviceError) {
            markMapCommandFailed(serviceError || payload.message || 'Save map failed');
          }
        }
        pushState(payload.status || robotState.status, payload.message || 'Service status updated.', {
          jetsonOnline: true,
          currentTask: service.servicePath,
          currentTaskType: 'service',
          launchStatus: getPayloadField(payload, 'launchStatus', robotState.launchStatus),
          runningLaunches: getPayloadField(payload, 'runningLaunches', robotState.runningLaunches),
          runningNodes: getPayloadField(payload, 'runningNodes', robotState.runningNodes),
          lastError: serviceError,
          taskId: getPayloadField(payload, 'requestId', robotState.activeTaskId),
          action: 'call_service',
          servicePath: service.servicePath,
          serviceType: service.serviceType,
          serviceSuccess: payload.serviceSuccess === true,
          serviceResult: String(payload.serviceResult || ''),
        });
        return;
      }
      if (payload.currentTaskType === 'service' && payload.currentTask === MAP_SERVICE_PATH) {
        if (payload.status === 'completed') {
          markMapCommandCompleted();
        } else if (payload.status === 'failed' || payload.lastError) {
          markMapCommandFailed(payload.lastError || payload.message || 'Save map failed');
        }
      }
      pushState(payload.status || robotState.status, payload.message || 'Launch status updated.', {
        jetsonOnline: true,
        currentTask: getPayloadField(payload, 'currentTask', robotState.currentTask),
        currentTaskType: getPayloadField(payload, 'currentTaskType', robotState.currentTaskType),
        launchStatus: getPayloadField(payload, 'launchStatus', robotState.launchStatus),
        runningLaunches: getPayloadField(payload, 'runningLaunches', robotState.runningLaunches),
        runningNodes: getPayloadField(payload, 'runningNodes', robotState.runningNodes),
        lastError: getPayloadField(payload, 'lastError', ''),
        taskId: getPayloadField(payload, 'requestId', robotState.activeTaskId),
      });
    }
  });
}

function requestJson(urlString, payload) {
  return requestUrl(urlString, {
    method: 'POST',
    jsonBody: payload,
  });
}

function requestUrl(urlString, options = {}) {
  return new Promise((resolve, reject) => {
    const targetUrl = new URL(urlString);
    const transport = targetUrl.protocol === 'https:' ? https : http;
    const method = options.method || 'GET';
    const timeoutMs = Number(options.timeoutMs);
    const headers = { ...(options.headers || {}) };
    let body = null;

    if (Object.prototype.hasOwnProperty.call(options, 'jsonBody')) {
      body = JSON.stringify(options.jsonBody);
      headers['Content-Type'] = 'application/json';
    } else if (Object.prototype.hasOwnProperty.call(options, 'body')) {
      body = options.body;
    }

    if (body !== null && !Object.prototype.hasOwnProperty.call(headers, 'Content-Length')) {
      headers['Content-Length'] = Buffer.byteLength(body);
    }

    const request = transport.request({
      protocol: targetUrl.protocol,
      hostname: targetUrl.hostname,
      port: targetUrl.port,
      path: `${targetUrl.pathname}${targetUrl.search}`,
      method,
      headers,
    }, response => {
      const chunks = [];
      response.on('data', chunk => {
        chunks.push(chunk);
      });
      response.on('end', () => {
        const rawBuffer = Buffer.concat(chunks);
        const rawText = rawBuffer.toString('utf8');

        if (options.expectBuffer) {
          if (response.statusCode < 200 || response.statusCode >= 300) {
            reject(new Error(`HTTP ${response.statusCode}`));
            return;
          }

          resolve({
            statusCode: response.statusCode,
            headers: response.headers,
            body: rawBuffer,
          });
          return;
        }

        let result = {};
        if (rawText) {
          try {
            result = JSON.parse(rawText);
          } catch (error) {
            reject(new Error(`Upstream returned invalid JSON: ${error.message}`));
            return;
          }
        }

        if (options.allowErrorResult && response.statusCode >= 200 && response.statusCode < 300) {
          resolve(result);
          return;
        }

        if (response.statusCode < 200 || response.statusCode >= 300 || result.ok === false) {
          const upstreamError = new Error(
            result.message
            || result.reason
            || (result.detail && result.detail.message)
            || `Upstream error (${response.statusCode})`
          );
          upstreamError.statusCode = Number(response.statusCode) || 502;
          upstreamError.payload = result;
          reject(upstreamError);
          return;
        }

        resolve(result);
      });
    });

    request.on('error', reject);
    if (Number.isFinite(timeoutMs) && timeoutMs > 0) {
      request.setTimeout(timeoutMs, () => {
        request.destroy(new Error(`Upstream request timed out after ${timeoutMs}ms`));
      });
    }
    if (body !== null) {
      request.write(body);
    }
    request.end();
  });
}

async function callJetsonAgent(endpoint, payload) {
  return requestJson(`${JETSON_AGENT_URL}${endpoint}`, payload);
}

async function getJetsonAgent(endpoint) {
  if (!JETSON_AGENT_URL) {
    return { ok: false, message: 'JETSON_AGENT_URL is not configured' };
  }
  return requestUrl(`${JETSON_AGENT_URL}${endpoint}`, { allowErrorResult: true });
}

async function postJetsonAgent(endpoint, payload, timeoutMs) {
  if (!JETSON_AGENT_URL) {
    return { ok: false, message: 'JETSON_AGENT_URL is not configured' };
  }
  return requestUrl(`${JETSON_AGENT_URL}${endpoint}`, {
    method: 'POST',
    jsonBody: payload,
    allowErrorResult: true,
    // 2026-10-07: 之前不传 timeoutMs，而 requestUrl 在该值非有限数时不设超时，
    // 上游一旦不回响应就会永久挂住。
    timeoutMs: Number.isFinite(Number(timeoutMs)) && Number(timeoutMs) > 0
      ? Number(timeoutMs)
      : 10000,
  });
}

async function sendNavigationGoal(body = {}) {
  const x = Number(body.x);
  const y = Number(body.y);
  const yaw = body.yaw === undefined || body.yaw === null ? 0 : Number(body.yaw);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yaw)) {
    return { ok: false, message: 'x, y and yaw must be valid numbers' };
  }
  const result = await postJetsonAgent('/api/navigation/goal', {
    deviceId: body.deviceId || robotState.deviceId,
    x,
    y,
    yaw,
  });
  const navigationState = String(
    result.navigationState || result.status || ''
  ).toLowerCase();
  const accepted = result.accepted === true || navigationState === 'navigating';
  if (!accepted) return result;

  // Once Nav2 has accepted the goal, optional post-submit diagnostics must not
  // turn the action into an HTTP failure.  Preserve those diagnostics without
  // hiding the durable, pollable navigation state from the mini-program.
  return {
    ...result,
    ok: true,
    accepted: true,
    navigationState: 'navigating',
    diagnostic: result.diagnostic || (result.ok === false ? result.message || '' : ''),
    message: 'Navigation goal accepted',
  };
}

async function fetchRobotPose() {
  return getJetsonAgent('/api/robot/pose');
}

function navigationStatusEndpoint(goalId = '') {
  const normalizedGoalId = String(goalId || '').trim();
  return normalizedGoalId
    ? `/api/navigation/status?goalId=${encodeURIComponent(normalizedGoalId)}`
    : '/api/navigation/status';
}

async function fetchNavigationStatus(goalId = '') {
  return getJetsonAgent(navigationStatusEndpoint(goalId));
}

function navigationStatusHttpStatus(navigation) {
  // A failed or canceled Nav2 goal is a successfully observed terminal state,
  // not an upstream transport failure.  Missing/malformed status envelopes
  // remain 502 so loss of the agent still fails closed.
  if (!navigation || typeof navigation !== 'object' || Array.isArray(navigation)) {
    return 502;
  }
  const status = String(navigation.status || '').trim().toLowerCase();
  return ['idle', 'navigating', 'succeeded', 'canceled', 'failed'].includes(status)
    ? 200
    : 502;
}

function normalizeMapSnapshot(req, snapshot = {}) {
  return {
    ok: snapshot.ok !== false,
    mapReady: Boolean(snapshot.mapReady),
    mapFormat: snapshot.mapFormat || MAP_FORMAT,
    mapPath: snapshot.mapPath || MAP_FILE_PATH,
    mapUpdatedAt: snapshot.mapUpdatedAt || '',
    lastAttemptAt: snapshot.lastAttemptAt || '',
    lastSuccessAt: snapshot.lastSuccessAt || '',
    lastError: snapshot.lastError || '',
    lastCommandStatus: snapshot.lastCommandStatus || 'idle',
    saveInProgress: Boolean(snapshot.saveInProgress),
    rosReady: snapshot.rosReady !== false,
    mapServiceReady: Object.prototype.hasOwnProperty.call(snapshot, 'mapServiceReady')
      ? Boolean(snapshot.mapServiceReady)
      : true,
    statusUrl: `${buildBaseUrl(req)}/api/map/status`,
    fileUrl: getMapFileUrl(req),
    metadataUrl: getMapMetadataUrl(req),
    mapMetadataPath: snapshot.mapMetadataPath || '',
    mapWidth: Object.prototype.hasOwnProperty.call(snapshot, 'mapWidth') ? snapshot.mapWidth : null,
    mapHeight: Object.prototype.hasOwnProperty.call(snapshot, 'mapHeight') ? snapshot.mapHeight : null,
    resolution: Object.prototype.hasOwnProperty.call(snapshot, 'resolution') ? snapshot.resolution : null,
    originX: Object.prototype.hasOwnProperty.call(snapshot, 'originX') ? snapshot.originX : null,
    originY: Object.prototype.hasOwnProperty.call(snapshot, 'originY') ? snapshot.originY : null,
    originYaw: Object.prototype.hasOwnProperty.call(snapshot, 'originYaw') ? snapshot.originYaw : 0,
    deviceId: snapshot.deviceId || robotState.deviceId,
  };
}

async function fetchMapServiceStatus(req, options = {}) {
  const snapshot = await requestUrl(`${MAP_SERVICE_URL}/api/map/status`, {
    timeoutMs: options.timeoutMs,
  });
  return normalizeMapSnapshot(req, snapshot);
}

async function fetchMapServiceMetadata(req) {
  const snapshot = await requestUrl(`${MAP_SERVICE_URL}/api/map/metadata`);
  return normalizeMapSnapshot(req, snapshot);
}

async function fetchLiveMapServiceStatus(req) {
  const snapshot = await requestUrl(`${MAP_SERVICE_URL}/api/map/live/status`);
  return {
    ok: snapshot.ok !== false,
    deviceId: snapshot.deviceId || robotState.deviceId,
    liveMapReady: Boolean(snapshot.liveMapReady),
    liveMapTopic: snapshot.liveMapTopic || '/map',
    // Preserve upstream diagnostics so the mini-program can distinguish
    // "Cartographer has not published a map yet" from a bridge failure.
    // These are additive fields and do not change the existing contract.
    liveMapService: snapshot.liveMapService || '',
    liveMapSource: snapshot.liveMapSource || '',
    liveMapCallbackCount: Number.isFinite(snapshot.liveMapCallbackCount)
      ? snapshot.liveMapCallbackCount
      : 0,
    liveMapServiceFallbackCount: Number.isFinite(snapshot.liveMapServiceFallbackCount)
      ? snapshot.liveMapServiceFallbackCount
      : 0,
    liveMapServiceReady: snapshot.liveMapServiceReady === true,
    liveMapLastError: snapshot.liveMapLastError || '',
    mapFormat: snapshot.mapFormat || MAP_FORMAT,
    mapUpdatedAt: snapshot.mapUpdatedAt || '',
    mapWidth: Object.prototype.hasOwnProperty.call(snapshot, 'mapWidth') ? snapshot.mapWidth : null,
    mapHeight: Object.prototype.hasOwnProperty.call(snapshot, 'mapHeight') ? snapshot.mapHeight : null,
    resolution: Object.prototype.hasOwnProperty.call(snapshot, 'resolution') ? snapshot.resolution : null,
    originX: Object.prototype.hasOwnProperty.call(snapshot, 'originX') ? snapshot.originX : null,
    originY: Object.prototype.hasOwnProperty.call(snapshot, 'originY') ? snapshot.originY : null,
    originYaw: Object.prototype.hasOwnProperty.call(snapshot, 'originYaw') ? snapshot.originYaw : 0,
    rosReady: snapshot.rosReady !== false,
    mapServiceReady: true,
    statusUrl: `${buildBaseUrl(req)}/api/map/live/status`,
    fileUrl: getLiveMapFileUrl(req),
    lastError: snapshot.lastError || '',
  };
}

async function triggerMapSave(req, body = {}) {
  markMapCommandAttempt();
  if (!getActiveCartographerSessionForSave()) {
    const message = 'Cartographer mapping session is not active; map save was not sent.';
    markMapCommandFailed(message);
    let snapshot;
    try {
      snapshot = await fetchMapServiceStatus(req);
    } catch (error) {
      snapshot = normalizeMapSnapshot(req, {
        ok: false, mapReady: false, rosReady: false, mapServiceReady: false,
      });
    }
    return {
      ...snapshot, ok: false, lastCommandStatus: 'failed', lastError: message, message,
    };
  }
  try {
    const snapshot = await requestUrl(`${MAP_SERVICE_URL}/api/map/save`, {
      method: 'POST',
      jsonBody: {
        deviceId: body.deviceId || robotState.deviceId,
      },
      allowErrorResult: true,
    });
    if (snapshot.ok === false || snapshot.lastCommandStatus === 'failed') {
      markMapCommandFailed(snapshot.lastError || 'Save map failed');
    } else if (snapshot.saveInProgress || snapshot.lastCommandStatus === 'pending') {
      mapState.lastCommandStatus = 'pending';
      mapState.lastError = '';
    } else {
      markMapCommandCompleted();
    }
    return normalizeMapSnapshot(req, snapshot);
  } catch (error) {
    markMapCommandFailed(error.message);
    throw error;
  }
}

async function getMapStatusSnapshot(req, options = {}) {
  try {
    return await fetchMapServiceStatus(req, options);
  } catch (error) {
    return {
      ...getMapSnapshot(req),
      mapServiceReady: false,
      rosReady: false,
      saveInProgress: false,
      statusUrl: `${buildBaseUrl(req)}/api/map/status`,
      fileUrl: getMapFileUrl(req),
      mapServiceError: error.message,
    };
  }
}

async function proxyMapFile(res) {
  const upstream = await requestUrl(`${MAP_SERVICE_URL}/api/map/latest.pgm`, {
    expectBuffer: true,
  });

  const headers = {
    'Content-Type': upstream.headers['content-type'] || 'image/x-portable-graymap',
    'Content-Disposition': upstream.headers['content-disposition'] || 'inline; filename="stemm_map.pgm"',
    'Cache-Control': upstream.headers['cache-control'] || 'no-store',
    'X-Map-Format': upstream.headers['x-map-format'] || MAP_FORMAT,
  };

  if (upstream.headers['x-map-updated-at']) {
    headers['X-Map-Updated-At'] = upstream.headers['x-map-updated-at'];
  }

  res.writeHead(upstream.statusCode, headers);
  res.end(upstream.body);
}

async function proxyLiveMapFile(res) {
  const upstream = await requestUrl(`${MAP_SERVICE_URL}/api/map/live.pgm`, {
    expectBuffer: true,
  });

  const headers = {
    'Content-Type': upstream.headers['content-type'] || 'image/x-portable-graymap',
    'Content-Disposition': upstream.headers['content-disposition'] || 'inline; filename="live_map.pgm"',
    'Cache-Control': upstream.headers['cache-control'] || 'no-store',
    'X-Map-Format': upstream.headers['x-map-format'] || MAP_FORMAT,
  };

  if (upstream.headers['x-map-source']) {
    headers['X-Map-Source'] = upstream.headers['x-map-source'];
  }
  if (upstream.headers['x-map-updated-at']) {
    headers['X-Map-Updated-At'] = upstream.headers['x-map-updated-at'];
  }

  res.writeHead(upstream.statusCode, headers);
  res.end(upstream.body);
}

function proxyLocalHls(req, res) {
  const targetUrl = new URL(req.url, LOCAL_HLS_URL);
  const transport = targetUrl.protocol === 'https:' ? https : http;

  const upstreamReq = transport.request({
    protocol: targetUrl.protocol,
    hostname: targetUrl.hostname,
    port: targetUrl.port,
    path: `${targetUrl.pathname}${targetUrl.search}`,
    method: 'GET',
  }, upstreamRes => {
    const headers = {
      ...upstreamRes.headers,
      'Access-Control-Allow-Origin': '*',
    };

    res.writeHead(upstreamRes.statusCode || 502, headers);
    upstreamRes.pipe(res);
  });

  upstreamReq.on('error', error => {
    sendJson(res, 502, {
      ok: false,
      message: 'Local HLS service unavailable',
      upstream: LOCAL_HLS_URL,
      lastError: error.message,
    });
  });

  upstreamReq.end();
}

function probeLocalVideoFrame() {
  return new Promise(resolve => {
    const targetUrl = new URL(LOCAL_VIDEO_FRAME_PATH, LOCAL_HLS_URL);
    const transport = targetUrl.protocol === 'https:' ? https : http;
    let settled = false;
    const finish = result => {
      if (settled) return;
      settled = true;
      resolve(result);
    };
    const upstreamReq = transport.request({
      protocol: targetUrl.protocol,
      hostname: targetUrl.hostname,
      port: targetUrl.port,
      path: `${targetUrl.pathname}${targetUrl.search}`,
      method: 'GET',
      headers: { 'Cache-Control': 'no-cache' },
    }, upstreamRes => {
      const statusCode = Number(upstreamRes.statusCode || 502);
      const streamActive = statusCode >= 200 && statusCode < 300;
      upstreamRes.resume();
      finish({
        streamActive,
        statusCode,
        contentType: String(upstreamRes.headers['content-type'] || ''),
        reason: streamActive ? '' : `upstream_http_${statusCode}`,
        lastError: '',
      });
    });
    upstreamReq.setTimeout(LOCAL_VIDEO_HEALTH_TIMEOUT_MS, () => {
      upstreamReq.destroy(new Error('Local video upstream health check timed out'));
    });
    upstreamReq.on('error', error => {
      finish({
        streamActive: false,
        statusCode: 0,
        contentType: '',
        reason: 'upstream_unavailable',
        lastError: error.message,
      });
    });
    upstreamReq.end();
  });
}

function clearTimers() {
  if (startTimer) {
    clearTimeout(startTimer);
    startTimer = null;
  }
  if (runTimer) {
    clearTimeout(runTimer);
    runTimer = null;
  }
}

function startNavigation() {
  if (robotState.status === 'starting' || robotState.status === 'running') {
    return { ok: false, reason: 'Robot is already navigating.' };
  }

  clearTimers();
  robotState.activeTaskId = createTaskId();
  pushState('received', 'Command received by demo server.');

  startTimer = setTimeout(() => {
    pushState('starting', 'Navigation is starting.');
  }, 800);

  runTimer = setTimeout(() => {
    pushState('running', 'Navigation is running.');
  }, 1800);

  return { ok: true, taskId: robotState.activeTaskId };
}

function stopNavigation() {
  clearTimers();
  if (robotState.status === 'idle' || robotState.status === 'stopped') {
    return { ok: false, reason: 'Robot is not running.' };
  }

  pushState('stopped', 'Navigation has stopped.', { movement: 'stopped' });
  const stoppedTaskId = robotState.activeTaskId;
  robotState.activeTaskId = null;
  return { ok: true, taskId: stoppedTaskId };
}

async function startLaunch(body = {}) {
  const payload = buildLaunchCommandPayload('start_launch', body);
  await publishMqttCommand(payload);
  robotState.activeTaskId = payload.requestId;
  pushState('launch_command_sent', 'Launch start command sent via MQTT.', {
    currentTask: `${payload.package}/${payload.launchFile}`,
    currentTaskType: 'launch',
    launchStatus: 'starting',
    lastError: '',
  });
  return {
    ok: true,
    taskId: payload.requestId,
    mode: 'mqtt',
    commandTopic: MQTT_COMMAND_TOPIC,
    ...(payload.videoSessionId ? { videoSessionId: payload.videoSessionId } : {}),
  };
}

async function stopLaunch(body = {}) {
  const payload = buildLaunchCommandPayload('stop_launch', body);
  await publishMqttCommand(payload);
  robotState.activeTaskId = payload.requestId;
  pushState('launch_command_sent', 'Launch stop command sent via MQTT.', {
    currentTask: `${payload.package}/${payload.launchFile}`,
    currentTaskType: 'launch',
    launchStatus: 'stopping',
  });
  return {
    ok: true,
    taskId: payload.requestId,
    mode: 'mqtt',
    commandTopic: MQTT_COMMAND_TOPIC,
    ...(payload.videoSessionId ? { videoSessionId: payload.videoSessionId } : {}),
  };
}

async function startNode(body = {}) {
  const payload = buildNodeCommandPayload('start_node', body);
  await publishMqttCommand(payload);
  robotState.activeTaskId = payload.requestId;
  pushState('node_command_sent', 'Node start command sent via MQTT.', {
    currentTask: `${payload.package}/${payload.nodeExecutable}`,
    currentTaskType: 'node',
    launchStatus: 'starting',
    lastError: '',
  });
  return { ok: true, taskId: payload.requestId, mode: 'mqtt', commandTopic: MQTT_COMMAND_TOPIC };
}

async function stopNode(body = {}) {
  const payload = buildNodeCommandPayload('stop_node', body);
  await publishMqttCommand(payload);
  robotState.activeTaskId = payload.requestId;
  pushState('node_command_sent', 'Node stop command sent via MQTT.', {
    currentTask: `${payload.package}/${payload.nodeExecutable}`,
    currentTaskType: 'node',
    launchStatus: 'stopping',
  });
  return { ok: true, taskId: payload.requestId, mode: 'mqtt', commandTopic: MQTT_COMMAND_TOPIC };
}

async function callService(body = {}) {
  const payload = buildServiceCommandPayload(body);
  if (payload.servicePath === MAP_SERVICE_PATH) {
    markMapCommandAttempt();
  }
  await publishMqttCommand(payload);
  robotState.activeTaskId = payload.requestId;
  pushState('service_command_sent', 'Service call command sent via MQTT.', {
    currentTask: payload.servicePath,
    currentTaskType: 'service',
    launchStatus: 'starting',
    lastError: '',
  });
  return { ok: true, taskId: payload.requestId, mode: 'mqtt', commandTopic: MQTT_COMMAND_TOPIC };
}

function sendMapFile(req, res) {
  proxyMapFile(res).catch(error => {
    sendJson(res, 502, {
      ok: false,
      message: 'Map service unavailable',
      mapReady: false,
      mapFormat: MAP_FORMAT,
      lastError: error.message,
    });
  });
}

function sendLiveMapFile(req, res) {
  proxyLiveMapFile(res).catch(error => {
    sendJson(res, 502, {
      ok: false,
      message: 'Live map service unavailable',
      liveMapReady: false,
      mapFormat: MAP_FORMAT,
      lastError: error.message,
    });
  });
}

function moveRobot(direction, message) {
  robotState.activeTaskId = createTaskId();
  pushState('manual_control', message, { movement: direction });
  return { ok: true, taskId: robotState.activeTaskId };
}

function stopManualControl(message = 'Robot has stopped.') {
  robotState.activeTaskId = createTaskId();
  pushState('manual_control', message, { movement: 'stopped' });
  return { ok: true, taskId: robotState.activeTaskId };
}

function getManualMotion(action) {
  const actionMap = {
    move_forward: {
      direction: 'forward',
      linear: MANUAL_LINEAR_SPEED,
      angular: 0,
      message: 'Robot is moving forward.',
    },
    move_backward: {
      direction: 'backward',
      linear: -MANUAL_LINEAR_SPEED,
      angular: 0,
      message: 'Robot is moving backward.',
    },
    turn_left: {
      direction: 'left',
      linear: 0,
      angular: MANUAL_ANGULAR_SPEED,
      message: 'Robot is turning left.',
    },
    turn_right: {
      direction: 'right',
      linear: 0,
      angular: -MANUAL_ANGULAR_SPEED,
      message: 'Robot is turning right.',
    },
  };

  return actionMap[action] || null;
}

async function startManualControl(action) {
  const motion = getManualMotion(action);
  if (!motion) {
    return { ok: false, statusCode: 400, message: 'Unsupported manual action' };
  }

  if (CONTROL_MODE === 'proxy') {
    const taskId = createTaskId();
    const agentResult = await callJetsonAgent('/api/manual-control/start', {
      deviceId: robotState.deviceId,
      action,
      linear: motion.linear,
      angular: motion.angular,
    });

    robotState.activeTaskId = taskId;
    pushState('manual_control', agentResult.message || motion.message, {
      movement: motion.direction,
      linear: motion.linear,
      angular: motion.angular,
      agentMode: 'jetson',
    });

    return { ok: true, taskId, mode: 'proxy' };
  }

  const result = moveRobot(motion.direction, motion.message);
  return { ...result, mode: 'mock' };
}

async function endManualControl() {
  if (CONTROL_MODE === 'proxy') {
    const taskId = createTaskId();
    const agentResult = await callJetsonAgent('/api/manual-control/stop', {
      deviceId: robotState.deviceId,
      action: 'stop',
      linear: 0,
      angular: 0,
    });

    robotState.activeTaskId = taskId;
    pushState('manual_control', agentResult.message || 'Robot has stopped.', {
      movement: 'stopped',
      linear: 0,
      angular: 0,
      agentMode: 'jetson',
    });

    return { ok: true, taskId, mode: 'proxy' };
  }

  const result = stopManualControl();
  return { ...result, mode: 'mock' };
}

function getRequestPathname(req) {
  return new URL(req.url, `http://${req.headers.host || 'localhost'}`).pathname;
}

function serveFile(req, res) {
  const pathname = getRequestPathname(req);
  const requestPath = pathname === '/' ? '/index.html' : pathname;
  const filePath = path.normalize(path.join(WEB_ROOT, requestPath));

  if (!filePath.startsWith(WEB_ROOT)) {
    sendJson(res, 403, { ok: false, message: 'Forbidden' });
    return;
  }

  fs.readFile(filePath, (error, data) => {
    if (error) {
      sendJson(res, 404, { ok: false, message: 'File not found' });
      return;
    }

    const extension = path.extname(filePath);
    const contentTypes = {
      '.html': 'text/html; charset=utf-8',
      '.js': 'application/javascript; charset=utf-8',
      '.css': 'text/css; charset=utf-8',
    };

    res.writeHead(200, { 'Content-Type': contentTypes[extension] || 'text/plain; charset=utf-8' });
    res.end(data);
  });
}

function readBody(req, timeoutMs) {
  // 2026-10-07: 原来只监听 data/end，没有超时。若请求体永远不结束（对端异常、
  // 连接半开等），这里会永久挂住，调用方的 HTTP 响应也就永远发不出去
  // （现场：POST /api/feature/mapping/recover 经 :3000 挂 30s 无响应）。
  return new Promise((resolve, reject) => {
    let body = '';
    let settled = false;
    const finish = (fn, arg) => {
      if (settled) return;
      settled = true;
      if (timer) clearTimeout(timer);
      fn(arg);
    };
    const timer = Number.isFinite(Number(timeoutMs)) && Number(timeoutMs) > 0
      ? setTimeout(() => finish(reject, new Error('Request body timed out')), Number(timeoutMs))
      : null;
    req.on('data', chunk => {
      body += chunk;
      if (body.length > 1e6) {
        finish(reject, new Error('Body too large'));
        req.destroy();
      }
    });
    req.on('end', () => finish(resolve, body));
    req.on('error', error => finish(reject, error));
  });
}

const server = http.createServer(async (req, res) => {
  const pathname = getRequestPathname(req);

  if (req.method === 'GET' && pathname === '/api/status') {
    const mapSnapshot = await getMapStatusSnapshot(req, {
      timeoutMs: STATUS_MAP_PROBE_TIMEOUT_MS,
    });
    const mappingSession = getMappingSessionSummary();
    sendJson(res, 200, {
      status: robotState.status,
      taskId: robotState.activeTaskId,
      activeTaskId: robotState.activeTaskId,
      movement: robotState.movement,
      controlMode: robotState.controlMode,
      mqttEnabled: robotState.mqttEnabled,
      mqttConnected: robotState.mqttConnected,
      mqttBrokerUrl: robotState.mqttBrokerUrl,
      mqttCommandTopic: robotState.mqttCommandTopic,
      launchStatus: robotState.launchStatus,
      currentTask: robotState.currentTask,
      currentTaskType: robotState.currentTaskType,
      runningLaunches: robotState.runningLaunches,
      runningNodes: robotState.runningNodes,
      jetsonOnline: robotState.jetsonOnline,
      robotLastError: robotState.lastError,
      featureStatus: robotState.featureStatus,
      deviceRestart: getDeviceRestartStatus(),
      bootId: CURRENT_BOOT_ID,
      mappingSession,
      history: robotState.history,
      ...mapSnapshot,
    });
    return;
  }


  // ---- 2026-09-28: mapping recovery on the STATUS plane.
  // The mini program resolves its status/command base to :3000 only
  // (normalizeCommandStatusBaseUrl), while the mapping recovery endpoint is implemented by
  // the robot agent (:8001).  Without this bridge the app's auto-recovery POST landed on a
  // 404 here, the car never archived its terminal mapping failure state, and the
  // "mapping start failed" modal re-opened on every 1 Hz /api/status poll after dismissal.
  // The agent archives only a TERMINAL, non-active session, so this is safe to expose.
  if (req.method === 'POST' && pathname === '/api/feature/mapping/recover') {
    // 2026-10-07: 这条转发之前没有超时保护。实测：直连 agent :8001 是 15ms/200，
    // 但经这里会永久挂住（30s+ 无响应），小程序只好等到自己的 15s 超时，
    // 表现为"正在自动恢复建图服务"一直转、弹窗消不掉。
    // 现在：读体 5s、转发 8s，超时/异常都返回结构化结果（HTTP 200 + ok:false），
    // 让前端立刻收敛，绝不挂住。
    let recoverBody = {};
    try {
      const rawRecoverBody = (await readBody(req, RECOVER_BODY_TIMEOUT_MS)) || '';
      // 2026-10-07: readBody 返回的是【字符串】。之前直接把它当作 jsonBody 传入，
      // 而 requestUrl 会对 jsonBody 再做一次 JSON.stringify，于是发给 agent 的
      // 是一个"被引号包起来的字符串"而不是对象 → FastAPI 返回 422 →
      // 加上原先没有超时，请求就永久挂住（现场 30s 无响应）。
      recoverBody = rawRecoverBody ? JSON.parse(rawRecoverBody) : {};
      if (!recoverBody || typeof recoverBody !== 'object') recoverBody = {};
    } catch (error) {
      recoverBody = {};
    }
    try {
      const recovered = await postJetsonAgent(
        '/api/feature/mapping/recover',
        recoverBody,
        RECOVER_PROXY_TIMEOUT_MS,
      );
      sendJson(res, 200, recovered);
    } catch (error) {
      sendJson(res, 200, {
        ok: false,
        accepted: false,
        recovering: false,
        recovered: false,
        supported: true,
        reasonCode: 'PROXY_TIMEOUT',
        reasonText: `Mapping recovery proxy did not answer in ${RECOVER_PROXY_TIMEOUT_MS}ms`,
        lastError: String((error && error.message) || error || ''),
      });
    }
    return;
  }

  // ---- 2026-09-28: car-side provisioning (network agent).  Contract from the 0927 build:
  // all five endpoints live on this service (:3000).  The helper is a root-owned script
  // invoked with the same sudo -n pattern as the device-restart launcher.
  if (pathname.startsWith('/api/network/')) {
    const networkAgent = (args, timeoutMs) => new Promise(resolve => {
      execFile('sudo', ['-n', NETWORK_AGENT_LAUNCHER, ...args],
        { timeout: timeoutMs || 25000, maxBuffer: 256 * 1024 },
        (error, stdout) => {
          const text = String(stdout || '').trim();
          try {
            resolve(JSON.parse(text));
          } catch (parseError) {
            resolve({ ok: false, errorCode: 'UNSUPPORTED', message: 'network agent unavailable' });
          }
        });
    });

    if (req.method === 'GET' && pathname === '/api/network/status') {
      const result = await networkAgent(['--status'], 15000);
      if (!result || result.ok !== true) {
        sendJson(res, 503, { ok: false, errorCode: 'NM_UNAVAILABLE' });
        return;
      }
      sendJson(res, 200, { ok: true, network: result.network });
      return;
    }

    if (req.method === 'GET' && pathname === '/api/network/scan') {
      const result = await networkAgent(['--scan'], 30000);
      if (!result || result.ok !== true) {
        sendJson(res, 503, { ok: false, errorCode: 'NM_UNAVAILABLE' });
        return;
      }
      sendJson(res, 200, { ok: true, networks: result.networks || [] });
      return;
    }

    if (req.method === 'POST' && pathname === '/api/network/connect') {
      const rawBody = await readBody(req);
      let body = {};
      try { body = rawBody ? JSON.parse(rawBody) : {}; } catch (error) { body = {}; }
      const requestId = String(body.requestId || '').trim();
      const ssid = String(body.ssid || '').trim();
      const password = String(body.password || '');
      if (!/^[A-Za-z0-9_-]{8,128}$/.test(requestId)) {
        sendJson(res, 400, { ok: false, errorCode: 'BAD_REQUEST' });
        return;
      }
      const ssidBytes = Buffer.byteLength(ssid, 'utf8');
      if (ssidBytes < 1 || ssidBytes > 32) {
        sendJson(res, 400, { ok: false, errorCode: 'INVALID_SSID' });
        return;
      }
      if (password && (password.length < 8 || password.length > 63)) {
        sendJson(res, 400, { ok: false, errorCode: 'INVALID_PASSWORD' });
        return;
      }
      const result = await networkAgent(
        ['--connect', '--request-id', requestId, '--ssid', ssid, '--password', password], 20000);
      if (result && result.ok === true) {
        sendJson(res, 202, result);
        return;
      }
      const code = (result && result.errorCode) || 'UNSUPPORTED';
      sendJson(res, code === 'BUSY' ? 409 : code === 'UNSUPPORTED' ? 404 : 400,
        { ok: false, errorCode: code, requestId: (result && result.requestId) || '' });
      return;
    }

    if (req.method === 'GET' && pathname === '/api/network/connect/status') {
      const requestId = String(
        new URL(req.url, 'http://' + (req.headers.host || 'localhost')).searchParams.get('requestId') || ''
      ).trim();
      const result = await networkAgent(['--connect-status', '--request-id', requestId], 15000);
      if (!result) {
        sendJson(res, 503, { ok: false, errorCode: 'NM_UNAVAILABLE' });
        return;
      }
      sendJson(res, result.ok === true ? 200 : 200, result);
      return;
    }

    if (req.method === 'POST' && pathname === '/api/network/hotspot') {
      const rawBody = await readBody(req);
      let body = {};
      try { body = rawBody ? JSON.parse(rawBody) : {}; } catch (error) { body = {}; }
      const enable = body.enable === true || body.enable === 1 || body.enable === '1';
      const durationSec = Number.isFinite(Number(body.durationSec))
        ? Math.max(0, Math.floor(Number(body.durationSec))) : 0;
      const args = ['--hotspot', '--enable', enable ? '1' : '0'];
      if (enable && durationSec > 0) { args.push('--duration-sec', String(durationSec)); }
      const result = await networkAgent(args, 45000);
      if (result && result.ok === true) {
        sendJson(res, 200, result);
        return;
      }
      const code = (result && result.errorCode) || 'HOTSPOT_FAILED';
      sendJson(res, code === 'UNSUPPORTED' ? 404 : 503, { ok: false, errorCode: code });
      return;
    }
  }

  if (req.method === 'GET' && pathname === '/api/device/restart/status') {
    sendJson(res, 200, {
      ok: true,
      deviceId: robotState.deviceId,
      deviceRestart: getDeviceRestartStatus(),
    });
    return;
  }

  if (req.method === 'POST' && pathname === '/api/device/restart') {
    try {
      const rawBody = await readBody(req);
      const body = rawBody ? JSON.parse(rawBody) : {};
      const idempotencyKey = normalizeDeviceRestartIdempotencyKey(
        body && body.idempotencyKey
      );
      if (!idempotencyKey) {
        sendJson(res, 400, {
          ok: false,
          code: 'INVALID_IDEMPOTENCY_KEY',
          deviceRestart: getDeviceRestartStatus(),
        });
        return;
      }
      const result = await requestDeviceRestart(idempotencyKey);
      const deviceRestart = result.deviceRestart || getDeviceRestartStatus();
      if (result.ok && result.restartId) {
        broadcast({
          type: 'device_restart',
          deviceId: robotState.deviceId,
          restartId: result.restartId,
          deviceRestart,
          message: 'device_restart_changed',
        });
        sendJson(res, deviceRestart.state === 'succeeded' ? 200 : 202, {
          ok: true,
          accepted: result.accepted === true,
          reused: result.reused === true,
          restartId: result.restartId,
          state: deviceRestart.state,
          deviceRestart,
        });
        return;
      }
      sendJson(
        res,
        result.code === 'RESTART_ALREADY_ACTIVE' ? 409 : 503,
        {
          ok: false,
          code: result.code || 'DEVICE_RESTART_UNAVAILABLE',
          restartId: result.restartId || '',
          state: deviceRestart.state,
          deviceRestart,
        }
      );
    } catch (error) {
      sendJson(res, 400, {
        ok: false,
        code: 'INVALID_RESTART_REQUEST',
        deviceRestart: getDeviceRestartStatus(),
      });
    }
    return;
  }

  if (req.method === 'GET' && pathname === '/api/mapping/session/status') {
    const taskId = String(new URL(req.url, 'http://' + (req.headers.host || 'localhost')).searchParams.get('taskId') || '').trim();
    if (!taskId) {
      sendJson(res, 400, { ok: false, message: 'taskId is required' });
      return;
    }

    // Session state is authoritative for mapping completion.  Reconcile it
    // before calculating task status so a bridge "launch exited" message
    // cannot turn a safely canceled mapping into a false success.
    let session = reconcileCartographerSessionStatus();
    let task = taskStatusRecords.get(taskId) || [...robotState.history].reverse().find(item => item.taskId === taskId);
    if (!task) {
      sendJson(res, 404, { ok: false, taskId, message: 'Task status is not available' });
      return;
    }
    task = reconcileInactiveCartographerTask(taskId, task);
    session = reconcileCartographerSessionStatus();
    task = taskStatusRecords.get(taskId) || task;
    const status = normalizeTaskStatus(task);
    const sessionMatchesTask = Boolean(session && session.taskId === taskId);

    if (!sessionMatchesTask) {
      if (isTerminalTaskStatus(status)) {
        // Also recognize task records written by the pre-fix server. That
        // historical error was produced only after the bridge saw neither a
        // live mapping session nor its launch process.
        const processExited = task.mappingProcessExited === true ||
          String(task.lastError || '').includes(
            'Cartographer task is not backed by a live session or launch process.');
        const reportedStatus = processExited ? 'canceled' : status;
        const fallbackSession = task.mappingSession || {};
        const fallbackPhase = String(fallbackSession.phase || '').trim() || 'closed';
        const fallbackLastError = String(
          task.lastError || fallbackSession.lastError || ''
        ).trim();
        sendJson(res, 200, {
          ok: true, taskId, status: reportedStatus, terminal: true,
          phase: fallbackPhase,
          manualTakeoverRequired: false, sessionActive: false,
          processExited: processExited || fallbackSession.processExited === true,
          functionalServicesRestored: fallbackSession.functionalServicesRestored === true,
          cancelRequested: processExited || fallbackSession.cancelRequested === true,
          terminationStatus: fallbackSession.terminationStatus || '',
          terminationReason: fallbackSession.terminationReason || '',
          returnHomeFailureReason: fallbackSession.returnHomeFailureReason || '',
          returnHomeFailureConfirmed: fallbackSession.returnHomeFailureConfirmed === true,
          returnHomeFailureKind: fallbackSession.returnHomeFailureKind || '',
          finishRequestedAt: fallbackSession.finishRequestedAt || '',
          finishRequestedAtEpochMs: Number(fallbackSession.finishRequestedAtEpochMs || 0),
          returnHomeStartedAt: fallbackSession.returnHomeStartedAt || '',
          returnHomeStartedAtEpochMs: Number(fallbackSession.returnHomeStartedAtEpochMs || 0),
          mappingDone: processExited || fallbackSession.mappingDone === true,
          navigationState: fallbackSession.navigationState || '',
          finalMapSaved: fallbackSession.finalMapSaved === true,
          finalMapSavedAt: fallbackSession.finalMapSavedAt || '',
          finalMapSaveError: fallbackSession.finalMapSaveError || '',
          returnHomeRetry: Number(fallbackSession.returnHomeRetry || 0),
          returnHomeMaxRetries: Number(fallbackSession.returnHomeMaxRetries || 0),
          positionErrorMeters: Object.prototype.hasOwnProperty.call(
            fallbackSession, 'positionErrorMeters'
          ) ? fallbackSession.positionErrorMeters : null,
          pose: fallbackSession.pose || null,
          // A retained terminal success/cancel record must remain a terminal
          // result, not become a synthetic failure after the live-session
          // retention window expires.
          lastError: fallbackLastError || (
            reportedStatus === 'failed'
              ? 'Cartographer session status is not available.'
              : ''
          ),
          updatedAt: fallbackSession.updatedAt || task.time || task.timestamp || '',
          updatedAtEpochMs: Number(
            fallbackSession.updatedAtEpochMs ||
            Date.parse(task.time || task.timestamp || '') ||
            0
          ),
        });
        return;
      }
      sendJson(res, 503, { ok: false, taskId, message: 'Cartographer session status is not available' });
      return;
    }

    sendJson(res, 200, {
      ok: true,
      taskId,
      status,
      terminal: session.terminal === true || isTerminalTaskStatus(status),
      phase: session.phase || 'unknown',
      manualTakeoverRequired: session.manualTakeoverRequired === true,
      sessionActive: typeof session.sessionActive === 'boolean'
        ? session.sessionActive
        : session.terminal !== true,
      processExited: session.processExited === true,
      functionalServicesRestored: session.functionalServicesRestored === true,
      cancelRequested: session.cancelRequested === true,
      terminationStatus: session.terminationStatus || '',
      terminationReason: session.terminationReason || '',
      returnHomeFailureReason: session.returnHomeFailureReason || '',
      returnHomeFailureConfirmed: session.returnHomeFailureConfirmed === true,
      returnHomeFailureKind: session.returnHomeFailureKind || '',
      finishRequestedAt: session.finishRequestedAt || '',
      finishRequestedAtEpochMs: Number(session.finishRequestedAtEpochMs || 0),
      returnHomeStartedAt: session.returnHomeStartedAt || '',
      returnHomeStartedAtEpochMs: Number(session.returnHomeStartedAtEpochMs || 0),
      mappingDone: session.mappingDone === true,
      finalMapSaved: session.finalMapSaved === true,
      finalMapSavedAt: session.finalMapSavedAt || '',
      finalMapSaveError: session.finalMapSaveError || '',
      navigationState: session.navigationState || '',
      returnHomeRetry: Number(session.returnHomeRetry || 0),
      returnHomeMaxRetries: Number(session.returnHomeMaxRetries || 0),
      positionErrorMeters: session.positionErrorMeters,
      pose: session.pose || null,
      lastError: session.lastError || task.lastError || '',
      updatedAt: session.updatedAt || task.time || task.timestamp || '',
      updatedAtEpochMs: Number(session.updatedAtEpochMs || 0),
    });
    return;
  }

  if (req.method === 'GET' && pathname === '/api/task/status') {
    const taskId = String(new URL(req.url, 'http://' + (req.headers.host || 'localhost')).searchParams.get('taskId') || '').trim();
    if (!taskId) {
      sendJson(res, 400, { ok: false, message: 'taskId is required' });
      return;
    }
    reconcileCartographerSessionStatus();
    let task = taskStatusRecords.get(taskId) || [...robotState.history].reverse().find(item => item.taskId === taskId);
    if (!task) {
      sendJson(res, 404, { ok: false, taskId, message: 'Task status is not available' });
      return;
    }
    task = reconcileInactiveCartographerTask(taskId, task);
    task = taskStatusRecords.get(taskId) || task;
    const status = normalizeTaskStatus(task);
    sendJson(res, 200, {
      ok: true,
      taskId,
      status,
      rawStatus: task.status || task.launchStatus || '',
      terminal: ['succeeded', 'failed', 'canceled'].includes(status),
      updatedAt: task.time || task.timestamp || '',
      updatedAtEpochMs: Number(
        (task.mappingSession && task.mappingSession.updatedAtEpochMs) ||
        Date.parse(task.time || task.timestamp || '') ||
        0
      ),
      message: task.message || '',
      lastError: task.lastError || '',
      currentTask: task.currentTask || '-',
      currentTaskType: task.currentTaskType || '-',
      servicePath: task.servicePath || '',
      serviceType: task.serviceType || '',
      serviceSuccess: task.serviceSuccess === true,
      serviceResult: task.serviceResult || '',
      runningLaunches: task.runningLaunches || [],
      runningNodes: task.runningNodes || [],
    });
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/status') {
    const mapSnapshot = await getMapStatusSnapshot(req);
    sendJson(res, 200, mapSnapshot);
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/metadata') {
    try {
      const mapSnapshot = await fetchMapServiceMetadata(req);
      sendJson(res, 200, mapSnapshot);
    } catch (error) {
      sendJson(res, 502, {
        ok: false,
        message: error.message,
        mapServiceReady: false,
      });
    }
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/live/status') {
    try {
      const mapSnapshot = await fetchLiveMapServiceStatus(req);
      sendJson(res, 200, mapSnapshot);
    } catch (error) {
      sendJson(res, 502, {
        ok: false,
        message: error.message,
        liveMapReady: false,
        mapServiceReady: false,
      });
    }
    return;
  }

  if (req.method === 'GET' && pathname === '/api/robot/pose') {
    try {
      const poseSnapshot = await fetchRobotPose();
      sendJson(res, poseSnapshot.ok ? 200 : 502, poseSnapshot);
    } catch (error) {
      sendJson(res, 502, { ok: false, message: error.message });
    }
    return;
  }

  if (req.method === 'GET' && pathname === '/api/navigation/status') {
    try {
      const requestUrl = new URL(req.url, `http://${req.headers.host || 'localhost'}`);
      const navigation = await fetchNavigationStatus(requestUrl.searchParams.get('goalId') || '');
      sendJson(res, navigationStatusHttpStatus(navigation), navigation);
    } catch (error) {
      if (error.statusCode === 404 || error.statusCode === 410) {
        const payload = error.payload && error.payload.detail
          ? error.payload.detail
          : (error.payload || { ok: false, message: error.message });
        sendJson(res, error.statusCode, payload);
        return;
      }
      sendJson(res, 502, { ok: false, message: error.message });
    }
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/latest.pgm') {
    sendMapFile(req, res);
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/live.pgm') {
    sendLiveMapFile(req, res);
    return;
  }

  if (req.method === 'GET' && pathname === '/stream_session') {
    const health = await probeLocalVideoFrame();
    const framePath = LOCAL_VIDEO_FRAME_PATH.startsWith('/')
      ? LOCAL_VIDEO_FRAME_PATH
      : `/${LOCAL_VIDEO_FRAME_PATH}`;
    sendJson(res, 200, {
      playType: LOCAL_VIDEO_PLAY_TYPE,
      streamActive: health.streamActive,
      pollIntervalMs: LOCAL_VIDEO_POLL_INTERVAL,
      jpgUrl: `${buildBaseUrl(req)}${framePath}`,
      hlsUrl: `${buildBaseUrl(req)}${LOCAL_VIDEO_HLS_PATH.startsWith('/') ? LOCAL_VIDEO_HLS_PATH : `/${LOCAL_VIDEO_HLS_PATH}`}`,
      playUrl: `${buildBaseUrl(req)}${LOCAL_VIDEO_HLS_PATH.startsWith('/') ? LOCAL_VIDEO_HLS_PATH : `/${LOCAL_VIDEO_HLS_PATH}`}`,
      localOnly: true,
      upstream: LOCAL_HLS_URL,
      upstreamStatusCode: health.statusCode,
      upstreamContentType: health.contentType,
      reason: health.reason,
      lastError: health.lastError,
      retryAfterMs: health.streamActive ? 0 : LOCAL_VIDEO_POLL_INTERVAL,
    });
    return;
  }

  if (req.method === 'GET' && pathname.startsWith('/live/')) {
    proxyLocalHls(req, res);
    return;
  }

  if (req.method === 'POST' && pathname === '/api/navigation/goal') {
    if (isDeviceRestartActive()) {
      sendJson(res, 409, deviceRestartBlockedResponse());
      return;
    }
    try {
      const rawBody = await readBody(req);
      const body = rawBody ? JSON.parse(rawBody) : {};
      const result = await sendNavigationGoal(body);
      sendJson(res, result.ok ? 200 : 502, result);
    } catch (error) {
      sendJson(res, 400, { ok: false, message: error.message });
    }
    return;
  }

  if (req.method === 'POST' && pathname === '/api/map/save') {
    if (isDeviceRestartActive()) {
      sendJson(res, 409, deviceRestartBlockedResponse());
      return;
    }
    try {
      const rawBody = await readBody(req);
      const body = rawBody ? JSON.parse(rawBody) : {};
      const mapSnapshot = await triggerMapSave(req, body);
      const isPending = mapSnapshot.saveInProgress || mapSnapshot.lastCommandStatus === 'pending';
      sendJson(res, mapSnapshot.ok ? 200 : 502, {
        ...mapSnapshot,
        message: isPending
          ? 'Save map already in progress'
          : mapSnapshot.ok
          ? 'Save map request completed'
          : mapSnapshot.lastError || 'Save map request failed',
      });
    } catch (error) {
      sendJson(res, 502, {
        ok: false,
        message: error.message,
      });
    }
    return;
  }

  if (req.method === 'POST' && pathname === '/api/command') {
    if (isDeviceRestartActive()) {
      sendJson(res, 409, deviceRestartBlockedResponse());
      return;
    }
    try {
      const rawBody = await readBody(req);
      const body = rawBody ? JSON.parse(rawBody) : {};
      const action = body.action;

      if (action === 'start_navigation') {
        const result = startNavigation();
        sendJson(res, result.ok ? 200 : 409, result);
        return;
      }

      if (action === 'stop_navigation') {
        const result = stopNavigation();
        sendJson(res, result.ok ? 200 : 409, result);
        return;
      }

      if (action === 'start_launch') {
        const result = await startLaunch(body);
        sendJson(res, 200, result);
        return;
      }

      if (action === 'stop_launch') {
        const result = await stopLaunch(body);
        sendJson(res, 200, result);
        return;
      }

      if (action === 'start_node') {
        const result = await startNode(body);
        sendJson(res, 200, result);
        return;
      }

      if (action === 'stop_node') {
        const result = await stopNode(body);
        sendJson(res, 200, result);
        return;
      }

      if (action === 'call_service') {
        if (String(body.servicePath || '').trim() === MAP_SERVICE_PATH) {
          const mapSnapshot = await triggerMapSave(req, body);
          const isPending = mapSnapshot.saveInProgress || mapSnapshot.lastCommandStatus === 'pending';
          sendJson(res, mapSnapshot.ok ? 200 : 502, {
            ok: mapSnapshot.ok,
            message: isPending
              ? 'Save map already in progress'
              : mapSnapshot.ok
              ? 'Save map request completed'
              : mapSnapshot.lastError || 'Save map request failed',
            deviceId: robotState.deviceId,
            action: 'call_service',
            servicePath: MAP_SERVICE_PATH,
            serviceType: String(body.serviceType || '').trim(),
            mapFetch: {
              mapPath: mapSnapshot.mapPath,
              statusUrl: mapSnapshot.statusUrl,
              fileUrl: mapSnapshot.fileUrl,
            },
            mapStatus: mapSnapshot,
          });
          return;
        }

        const result = await callService(body);
        sendJson(res, 200, result);
        return;
      }

      if (['move_forward', 'move_backward', 'turn_left', 'turn_right'].includes(action)) {
        const result = await startManualControl(action);
        sendJson(res, result.ok ? 200 : result.statusCode || 409, result);
        return;
      }

      if (action === 'stop_manual_control') {
        const result = await endManualControl();
        sendJson(res, result.ok ? 200 : result.statusCode || 409, result);
        return;
      }

      sendJson(res, 400, { ok: false, message: 'Unsupported action' });
    } catch (error) {
      sendJson(res, 400, { ok: false, message: error.message });
    }
    return;
  }

  if (req.method === 'GET') {
    serveFile(req, res);
    return;
  }

  sendJson(res, 405, { ok: false, message: 'Method not allowed' });
});

const wss = new WebSocketServer({ server, path: '/ws' });

wss.on('connection', ws => {
  ws.send(JSON.stringify({
    ...buildStatePayload('WebSocket connected.'),
    history: robotState.history,
  }));
});

connectMqtt();
setInterval(reconcileCartographerSessionStatus, 1000).unref();

server.listen(PORT, HOST, () => {
  console.log(`Demo server running at http://${HOST}:${PORT}`);
  console.log(`WebSocket endpoint: ws://${HOST}:${PORT}/ws`);
  console.log(`Manual control mode: ${CONTROL_MODE}`);
  console.log(`MQTT enabled: ${MQTT_ENABLED}`);
  if (MQTT_ENABLED) {
    console.log(`MQTT broker: ${MQTT_BROKER_URL}`);
    console.log(`MQTT command topic: ${MQTT_COMMAND_TOPIC}`);
  }
  if (JETSON_AGENT_URL) {
    console.log(`Jetson agent: ${JETSON_AGENT_URL}`);
  }
  console.log(`Map service: ${MAP_SERVICE_URL}`);
});
