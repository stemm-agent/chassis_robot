# demo-control

最小化机器人控制 Demo，用于验证：

- 网页按钮发送命令
- 后端接收命令
- WebSocket 回传状态
- 页面实时更新
- 手动控制按住移动、松开停止
- 在不改动当前手动控制链路的前提下，增添 MQTT launch/task 能力

## 当前通信拆分

- `HTTP + WebSocket`：页面状态同步、手动控制、现有 Jetson `robot_agent.py` 链路
- `MQTT`：launch 启停命令、launch 状态、Jetson 心跳、错误上报

这样做的目标是：保留当前已经验证通过的手动控制体验，只把任务型控制增量迁移到 MQTT。

## 本地运行

```powershell
cd C:\Users\Lenovo\Desktop\demo-control
npm install
npm start
```

浏览器打开：

```text
http://localhost:3000
```

如果 MQTT broker 不可用，页面仍然可以继续使用当前手动控制 Demo；只是在状态区显示 `disconnected` / `offline`。

## 环境变量

可以复制 `.env.example` 为 `.env` 后再按需修改。

主要变量：

- `JETSON_AGENT_URL`：已有 Jetson HTTP 手动控制代理地址，例如 `http://192.168.124.162:8001`
- `MANUAL_LINEAR_SPEED=0.1`
- `MANUAL_ANGULAR_SPEED=0.5`
- `MQTT_ENABLED=true`
- `MQTT_BROKER_URL=mqtt://192.168.124.162:1883`
- `MQTT_DEVICE_ID=robot-001`
- `MQTT_COMMAND_TOPIC=robot/robot-001/launch/command`
- `MQTT_STATUS_TOPIC=robot/robot-001/launch/status`
- `MQTT_HEARTBEAT_TOPIC=robot/robot-001/heartbeat`
- `MQTT_ERROR_TOPIC=robot/robot-001/error`
- `MQTT_LAUNCH_PACKAGE=wheeltec_mic`
- `MQTT_LAUNCH_FILE=wheeltec_mic.launch.py`
- `MQTT_LAUNCH_ALLOWED_FILES=wheeltec_mic.launch.py`

## 手动控制模式

### mock 模式

不设置 `JETSON_AGENT_URL` 时：

- 前进/后退/左转/右转只在本地模拟状态
- 页面通过 WebSocket 实时更新 `movement`

### proxy 模式

如果你已经验证过 Jetson 的 `/cmd_vel` 手动控制链路，可以继续沿用：

Jetson 端启动 HTTP agent：

```bash
pip install -r jetson-agent/requirements.txt
python jetson-agent/robot_agent.py
```

Windows PowerShell 设置本地转发：

```powershell
$env:JETSON_AGENT_URL = "http://192.168.124.162:8001"
$env:MANUAL_LINEAR_SPEED = "0.1"
$env:MANUAL_ANGULAR_SPEED = "0.5"
npm start
```

启动后控制台会显示：

- `Manual control mode: proxy`

此时页面手动控制链路仍然是：

- 按住方向按钮 -> 本地 Node 后端 -> Jetson HTTP agent -> Jetson 持续发布 `/cmd_vel`
- 松开按钮 -> 本地 Node 后端 stop -> Jetson 发布 0 速度停止

## MQTT launch bridge

这是新增的 Jetson 侧样例，只负责 launch/task 控制，不负责手动移动。

安装依赖：

```bash
pip install -r jetson-agent/requirements.txt
```

启动 bridge：

```bash
python jetson-agent/launch_mqtt_bridge.py
```

默认环境变量：

- `MQTT_BROKER_HOST=0.0.0.0`
- `MQTT_BROKER_PORT=1883`
- `MQTT_DEVICE_ID=robot-001`
- `MQTT_LAUNCH_PACKAGE=wheeltec_mic`
- `MQTT_LAUNCH_FILE=wheeltec_mic.launch.py`
- `MQTT_LAUNCH_ALLOWED_FILES=wheeltec_mic.launch.py`

### 设计约束

- 只接受结构化 `start_launch` / `stop_launch` 命令
- 只允许白名单中的 launch 文件
- 不执行任意 shell 字符串
- `stop_launch` 只停止当前 bridge 启动并托管的进程，属于业务停止，不是任意 kill

## 页面新增能力

页面在保留原按钮的基础上，只做了最小增量：

- 新增 `启动 Launch`
- 新增 `停止 Launch`
- 新增状态字段：
  - `mqttConnected`
  - `jetsonOnline`
  - `launchStatus`
  - `currentTask`
  - `lastError`

## 接口

- `GET /api/status`
- `POST /api/command`
- `GET /ws` WebSocket

### 支持动作

- `start_navigation`
- `stop_navigation`
- `move_forward`
- `move_backward`
- `turn_left`
- `turn_right`
- `stop_manual_control`
- `start_launch`
- `stop_launch`

### 命令示例

```json
{
  "deviceId": "robot-001",
  "action": "start_launch",
  "package": "wheeltec_mic",
  "launchFile": "wheeltec_mic.launch.py",
  "args": {}
}
```

## 已完成校验

已做的轻量校验：

```powershell
node --check server/index.js
node --check web/app.js
python -m py_compile jetson-agent/launch_mqtt_bridge.py jetson-agent/robot_agent.py
```

如果要继续联调，下一步建议先确认：

1. 本地 `npm start` 后页面可以正常打开
2. 不接 MQTT broker 时，手动控制页面仍可正常使用
3. Jetson 启动 MQTT bridge 后，页面 Launch 按钮能收到状态回传
