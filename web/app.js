const statusElement = document.getElementById('status');
const taskIdElement = document.getElementById('taskId');
const deviceIdElement = document.getElementById('deviceId');
const movementElement = document.getElementById('movement');
const controlModeElement = document.getElementById('controlMode');
const mqttConnectedElement = document.getElementById('mqttConnected');
const jetsonOnlineElement = document.getElementById('jetsonOnline');
const launchStatusElement = document.getElementById('launchStatus');
const currentTaskElement = document.getElementById('currentTask');
const currentTaskTypeElement = document.getElementById('currentTaskType');
const lastErrorElement = document.getElementById('lastError');
const hintElement = document.getElementById('hint');
const logListElement = document.getElementById('logList');
const startButton = document.getElementById('startButton');
const stopButton = document.getElementById('stopButton');
const startLaunchButton = document.getElementById('startLaunchButton');
const stopLaunchButton = document.getElementById('stopLaunchButton');
const startNodeButton = document.getElementById('startNodeButton');
const stopNodeButton = document.getElementById('stopNodeButton');
const clearLogButton = document.getElementById('clearLogButton');
const forwardButton = document.getElementById('forwardButton');
const backwardButton = document.getElementById('backwardButton');
const leftButton = document.getElementById('leftButton');
const rightButton = document.getElementById('rightButton');
const stopManualButton = document.getElementById('stopManualButton');

const DEFAULT_LAUNCH_PAYLOAD = {
  deviceId: 'robot-001',
  action: 'start_launch',
  package: 'wheeltec_nav2',
  launchFile: 'wheeltec_nav2.launch.py',
};

const DEFAULT_STOP_LAUNCH_PAYLOAD = {
  deviceId: 'robot-001',
  action: 'stop_launch',
  package: 'wheeltec_nav2',
  launchFile: 'wheeltec_nav2.launch.py',
};

const DEFAULT_NODE_PAYLOAD = {
  deviceId: 'robot-001',
  action: 'start_node',
  package: 'auto_recharge_ros2',
  nodeExecutable: 'navigate_to_charger',
};

const DEFAULT_STOP_NODE_PAYLOAD = {
  deviceId: 'robot-001',
  action: 'stop_node',
  package: 'auto_recharge_ros2',
  nodeExecutable: 'navigate_to_charger',
};

let activeManualAction = null;
let activePointerId = null;

function appendLog(text) {
  const item = document.createElement('li');
  item.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
  logListElement.prepend(item);
}

function updateState(payload) {
  if (payload.deviceId) {
    deviceIdElement.textContent = payload.deviceId;
  }
  statusElement.textContent = payload.status || 'unknown';
  taskIdElement.textContent = payload.taskId || '-';
  movementElement.textContent = payload.movement || '-';
  controlModeElement.textContent = payload.controlMode || '-';
  mqttConnectedElement.textContent = payload.mqttConnected === true ? 'connected' : 'disconnected';
  jetsonOnlineElement.textContent = payload.jetsonOnline === true ? 'online' : 'offline';
  launchStatusElement.textContent = payload.launchStatus || '-';
  currentTaskElement.textContent = payload.currentTask || '-';
  currentTaskTypeElement.textContent = payload.currentTaskType || '-';
  lastErrorElement.textContent = payload.lastError || '-';
  hintElement.textContent = payload.message || '等待操作...';

  if (payload.message) {
    appendLog(`${payload.status}: ${payload.message}`);
  }
}

async function sendCommand(action) {
  return sendCommandWithBody({
    deviceId: 'robot-001',
    action,
  });
}

async function sendCommandWithBody(body) {
  hintElement.textContent = '正在发送命令...';
  try {
    const response = await fetch('/api/command', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(body),
    });

    const result = await response.json();
    if (!response.ok) {
      throw new Error(result.reason || result.message || 'Command failed');
    }

    appendLog(`command accepted: ${body.action} (${result.taskId || '-'})`);
    return result;
  } catch (error) {
    hintElement.textContent = `命令失败: ${error.message}`;
    appendLog(`error: ${error.message}`);
    throw error;
  }
}

function bindHoldControl(button, action, label) {
  const start = event => {
    event.preventDefault();
    if (activePointerId !== null && activePointerId !== event.pointerId) {
      return;
    }
    if (activeManualAction === action) {
      return;
    }

    activePointerId = event.pointerId;
    activeManualAction = action;
    if (typeof button.setPointerCapture === 'function') {
      button.setPointerCapture(event.pointerId);
    }
    hintElement.textContent = `正在发送${label}命令...`;
    button.classList.add('active');
    sendCommand(action).catch(() => {
      activeManualAction = null;
      activePointerId = null;
      button.classList.remove('active');
    });
  };

  const stop = event => {
    if (event) {
      event.preventDefault();
    }
    if (activePointerId !== null && event && event.pointerId !== undefined && event.pointerId !== activePointerId) {
      return;
    }
    if (activeManualAction !== action) {
      button.classList.remove('active');
      return;
    }

    activeManualAction = null;
    activePointerId = null;
    button.classList.remove('active');
    if (event && typeof button.releasePointerCapture === 'function' && event.pointerId !== undefined) {
      try {
        button.releasePointerCapture(event.pointerId);
      } catch (error) {
        // Ignore if the pointer is already released.
      }
    }
    hintElement.textContent = '正在发送停止命令...';
    sendCommand('stop_manual_control');
  };

  button.addEventListener('pointerdown', start);
  button.addEventListener('pointerup', stop);
  button.addEventListener('pointercancel', stop);
  button.addEventListener('lostpointercapture', stop);
}

async function loadInitialStatus() {
  const response = await fetch('/api/status');
  const result = await response.json();
  if (result.ok) {
    updateState(result);
  }
}

function connectWebSocket() {
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const ws = new WebSocket(`${protocol}//${location.host}/ws`);

  ws.addEventListener('open', () => {
    appendLog('WebSocket connected');
  });

  ws.addEventListener('message', event => {
    const payload = JSON.parse(event.data);
    updateState(payload);
  });

  ws.addEventListener('close', () => {
    appendLog('WebSocket disconnected, retrying in 1 second');
    setTimeout(connectWebSocket, 1000);
  });
}

startButton.addEventListener('click', () => sendCommand('start_navigation'));
stopButton.addEventListener('click', () => sendCommand('stop_navigation'));
startLaunchButton.addEventListener('click', () => sendCommandWithBody(DEFAULT_LAUNCH_PAYLOAD));
stopLaunchButton.addEventListener('click', () => sendCommandWithBody(DEFAULT_STOP_LAUNCH_PAYLOAD));
startNodeButton.addEventListener('click', () => sendCommandWithBody(DEFAULT_NODE_PAYLOAD));
stopNodeButton.addEventListener('click', () => sendCommandWithBody(DEFAULT_STOP_NODE_PAYLOAD));
bindHoldControl(forwardButton, 'move_forward', '前进');
bindHoldControl(backwardButton, 'move_backward', '后退');
bindHoldControl(leftButton, 'turn_left', '左转');
bindHoldControl(rightButton, 'turn_right', '右转');
stopManualButton.addEventListener('click', () => {
  activeManualAction = null;
  activePointerId = null;
  document.querySelectorAll('.movement-grid button').forEach(button => {
    button.classList.remove('active');
  });
  sendCommand('stop_manual_control');
});
clearLogButton.addEventListener('click', () => {
  logListElement.innerHTML = '';
});

loadInitialStatus().catch(error => {
  appendLog(`status load failed: ${error.message}`);
});
connectWebSocket();
