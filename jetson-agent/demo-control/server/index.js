const http = require('http');
const https = require('https');
const fs = require('fs');
const path = require('path');
const mqtt = require('mqtt');
const { WebSocketServer } = require('ws');

const PORT = Number(process.env.PORT || '3000');
const HOST = process.env.HOST || '0.0.0.0';
const WEB_ROOT = path.join(__dirname, '..', 'web');
const JETSON_AGENT_URL = process.env.JETSON_AGENT_URL || '';
const MANUAL_LINEAR_SPEED = Number(process.env.MANUAL_LINEAR_SPEED || '0.1');
const MANUAL_ANGULAR_SPEED = Number(process.env.MANUAL_ANGULAR_SPEED || '0.5');
const CONTROL_MODE = JETSON_AGENT_URL ? 'proxy' : 'mock';
const MQTT_BROKER_URL = process.env.MQTT_BROKER_URL || 'mqtt://127.0.0.1:1883';
const MQTT_CLIENT_ID = process.env.MQTT_CLIENT_ID || `demo-control-${process.pid}`;
const MQTT_DEVICE_ID = process.env.MQTT_DEVICE_ID || 'robot-001';
const MQTT_ENABLED = process.env.MQTT_ENABLED !== 'false';
const MQTT_COMMAND_TOPIC = process.env.MQTT_COMMAND_TOPIC || `robot/${MQTT_DEVICE_ID}/launch/command`;
const MQTT_STATUS_TOPIC = process.env.MQTT_STATUS_TOPIC || `robot/${MQTT_DEVICE_ID}/launch/status`;
const MQTT_HEARTBEAT_TOPIC = process.env.MQTT_HEARTBEAT_TOPIC || `robot/${MQTT_DEVICE_ID}/heartbeat`;
const MQTT_ERROR_TOPIC = process.env.MQTT_ERROR_TOPIC || `robot/${MQTT_DEVICE_ID}/error`;
const DEFAULT_LAUNCH_PACKAGE = process.env.MQTT_LAUNCH_PACKAGE || 'wheeltec_nav2';
const DEFAULT_LAUNCH_FILE = process.env.MQTT_LAUNCH_FILE || 'wheeltec_nav2.launch.py';
const DEFAULT_NODE_PACKAGE = process.env.MQTT_NODE_PACKAGE || 'auto_recharge_ros2';
const DEFAULT_NODE_EXECUTABLE = process.env.MQTT_NODE_EXECUTABLE || 'navigate_to_charger';
const MAP_FILE_PATH = process.env.MAP_FILE_PATH || '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm';
const MAP_SERVICE_PATH = process.env.MAP_SERVICE_PATH || '/stemm/save_map';
const MAP_SERVICE_URL = process.env.MAP_SERVICE_URL || 'http://127.0.0.1:8092';
const LOCAL_HLS_URL = process.env.LOCAL_HLS_URL || 'http://127.0.0.1:18080';
const LOCAL_VIDEO_PLAY_TYPE = process.env.LOCAL_VIDEO_PLAY_TYPE || 'mjpeg';
const LOCAL_VIDEO_POLL_INTERVAL = Number(process.env.LOCAL_VIDEO_POLL_INTERVAL || '150');
const MAP_FORMAT = 'pgm';

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
    buildTargetKey('stemm_nav2_manager', 'stemm_21_auto_mapping.launch.py'),
    buildTargetKey('turn_on_wheeltec_robot', 'turn_on_wheeltec_robot.launch.py'),
    buildTargetKey('bodyreader', 'bodyfollow_no_avoider.launch.py'),
    buildTargetKey('wheeltec_nav2', 'wheeltec_nav2.launch.py'),
    buildTargetKey('turn_on_wheeltec_robot', 'wheeltec_camera.launch.py'),
    buildTargetKey('cloud_video_pusher', 'rtmp_pusher.launch.py'),
    buildTargetKey('cloud_video_pusher', 'local_hls_server.launch.py'),
  ]
);

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
    buildTargetKey('/stemm/save_map', 'std_srvs/srv/Trigger'),
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
  history: [],
};

const mapState = {
  mapPath: MAP_FILE_PATH,
  lastAttemptAt: '',
  lastSuccessAt: '',
  lastError: '',
  lastCommandStatus: 'idle',
};

let taskCounter = 1;
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

  broadcast(buildStatePayload(message, extra));
}

function getPayloadField(payload, fieldName, fallbackValue) {
  return Object.prototype.hasOwnProperty.call(payload, fieldName) ? payload[fieldName] : fallbackValue;
}

function createTaskId() {
  return `task-${String(taskCounter++).padStart(3, '0')}`;
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

  return {
    requestId: createTaskId(),
    deviceId: robotState.deviceId,
    action,
    package: packageName,
    launchFile,
    args: body.args || {},
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
    const topics = [MQTT_STATUS_TOPIC, MQTT_HEARTBEAT_TOPIC, MQTT_ERROR_TOPIC];
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
          reject(new Error(result.message || result.reason || `Upstream error (${response.statusCode})`));
          return;
        }

        resolve(result);
      });
    });

    request.on('error', reject);
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

async function postJetsonAgent(endpoint, payload) {
  if (!JETSON_AGENT_URL) {
    return { ok: false, message: 'JETSON_AGENT_URL is not configured' };
  }
  return requestUrl(`${JETSON_AGENT_URL}${endpoint}`, {
    method: 'POST',
    jsonBody: payload,
    allowErrorResult: true,
  });
}

async function sendNavigationGoal(body = {}) {
  const x = Number(body.x);
  const y = Number(body.y);
  const yaw = body.yaw === undefined || body.yaw === null ? 0 : Number(body.yaw);
  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(yaw)) {
    return { ok: false, message: 'x, y and yaw must be valid numbers' };
  }
  return postJetsonAgent('/api/navigation/goal', {
    deviceId: body.deviceId || robotState.deviceId,
    x,
    y,
    yaw,
  });
}

async function fetchRobotPose() {
  return getJetsonAgent('/api/robot/pose');
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

async function fetchMapServiceStatus(req) {
  const snapshot = await requestUrl(`${MAP_SERVICE_URL}/api/map/status`);
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

async function getMapStatusSnapshot(req) {
  try {
    return await fetchMapServiceStatus(req);
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
  return { ok: true, taskId: payload.requestId, mode: 'mqtt', commandTopic: MQTT_COMMAND_TOPIC };
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
  return { ok: true, taskId: payload.requestId, mode: 'mqtt', commandTopic: MQTT_COMMAND_TOPIC };
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

function readBody(req) {
  return new Promise((resolve, reject) => {
    let body = '';
    req.on('data', chunk => {
      body += chunk;
      if (body.length > 1e6) {
        reject(new Error('Body too large'));
        req.destroy();
      }
    });
    req.on('end', () => resolve(body));
    req.on('error', reject);
  });
}

const server = http.createServer(async (req, res) => {
  const pathname = getRequestPathname(req);

  if (req.method === 'GET' && pathname === '/api/status') {
    const mapSnapshot = await getMapStatusSnapshot(req);
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
      jetsonOnline: robotState.jetsonOnline,
      robotLastError: robotState.lastError,
      history: robotState.history,
      ...mapSnapshot,
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

  if (req.method === 'GET' && pathname === '/api/map/latest.pgm') {
    sendMapFile(req, res);
    return;
  }

  if (req.method === 'GET' && pathname === '/api/map/live.pgm') {
    sendLiveMapFile(req, res);
    return;
  }

  if (req.method === 'GET' && pathname === '/stream_session') {
    sendJson(res, 200, {
      playType: LOCAL_VIDEO_PLAY_TYPE,
      streamActive: true,
      pollIntervalMs: LOCAL_VIDEO_POLL_INTERVAL,
    });
    return;
  }

  if (req.method === 'GET' && pathname.startsWith('/live/')) {
    proxyLocalHls(req, res);
    return;
  }

  if (req.method === 'POST' && pathname === '/api/navigation/goal') {
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
