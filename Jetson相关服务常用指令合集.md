## 当前五个服务说明

### 1. demo-control

- 作用：这是对外提供接口的主控服务，默认监听 `3000` 端口，前端、Windows 脚本、局域网内其它设备，都是先访问它。
- systemd 做了什么：
  - `demo-control.service` 在系统启动后拉起 `/opt/demo-control/deploy/jetson/start_demo_control.sh`。
  - service 里通常会设置 `APP_HOST=0.0.0.0`、`APP_PORT=3000`、`JETSON_AGENT_URL=http://127.0.0.1:8001` 这类环境变量。
  - 它本身不直接跑 ROS2，而是把 HTTP 服务长期托管给 systemd，保证异常退出后自动重启。
- 启动脚本做了什么：
  - 脚本文件：`/opt/demo-control/deploy/jetson/start_demo_control.sh`
  - 进入 `/opt/demo-control/server` 项目目录。
  - 读取或补默认环境变量，例如 `HOST`、`PORT`、`JETSON_AGENT_URL`。
  - 最后执行 `npm start`，真正启动 Node.js 服务。
  - 这个脚本的作用主要是“整理环境 + 启动 Node 服务”，让 systemd 不需要直接写一长串命令。
- 主程序做了什么：
  - 主程序文件：`/opt/demo-control/server/index.js`
  - 启动一个对外 HTTP 服务，默认监听 `3000`。
  - 提供 `http://<JetsonIP>:3000/api/status`，用于查看当前桥接状态、运行中的 launch/node 等信息。
  - 提供 `http://<JetsonIP>:3000/api/command`，用于接收外部控制命令。
  - 当收到手动控制相关请求时，把请求转发给 `robot-agent`。
  - 当收到 `start_launch`、`stop_launch`、`start_node`、`stop_node` 这类请求时，构造 MQTT 指令并发布给 `launch-mqtt-bridge`。
  - 同时订阅 bridge 回传的 MQTT 状态/心跳/错误消息，再把这些状态整理成上层更容易看的接口返回。
- 关键文件：
  - 程序入口：`/opt/demo-control/server/index.js`
  - 启动脚本：`/opt/demo-control/deploy/jetson/start_demo_control.sh`
  - systemd：`/etc/systemd/system/demo-control.service`
- 可以理解为：它是整个 Jetson 侧的“统一对外大门”，负责收请求、分发请求、汇总状态。

### 2. robot-agent

- 作用：这是本机机器人运动代理服务，默认监听 `8001` 端口，专门负责手动控制和机器人本机状态接口。
- systemd 做了什么：
  - `robot-agent.service` 拉起 `/opt/demo-control/deploy/jetson/start_robot_agent.sh`。
  - 这个服务常驻监听本机 `8001` 端口，供 `demo-control` 调用。
  - 它的定位是“只给 Jetson 本机其它服务用”，因此常见配置是走 `127.0.0.1`，不需要对外公开太多细节。
- 启动脚本做了什么：
  - 脚本文件：`/opt/demo-control/deploy/jetson/start_robot_agent.sh`
  - 进入 `/opt/demo-control/jetson-agent` 目录。
  - 先临时 `set +u`，再 `source /opt/ros/humble/setup.bash`，必要时继续 `source ~/wheeltec_ros2/install/setup.bash`。
  - 这样做是为了避免 ROS 环境脚本里的 `AMENT_TRACE_SETUP_FILES` 之类变量在 `set -u` 下报错。
  - ROS 环境准备好后，执行 `uvicorn robot_agent:app --host 0.0.0.0 --port 8001` 一类命令，把 FastAPI 服务跑起来。
- 主程序做了什么：
  - 主程序文件：`/opt/demo-control/jetson-agent/robot_agent.py`
  - 提供 `http://127.0.0.1:8001/api/status` 用于状态检查。
  - 提供手动控制相关接口，例如开始手动控制、停止手动控制。
  - 在 ROS2 环境里初始化控制节点、发布器或相关资源。
  - 接到上层请求后，真正去发机器人底盘运动命令，比如 `/cmd_vel` 这一类。
  - 它不负责回充 launch/node 的生命周期管理，那部分是 `launch-mqtt-bridge` 的职责。
- 关键文件：
  - 程序入口：`/opt/demo-control/jetson-agent/robot_agent.py`
  - 启动脚本：`/opt/demo-control/deploy/jetson/start_robot_agent.sh`
  - systemd：`/etc/systemd/system/robot-agent.service`
- 可以理解为：它是 `demo-control` 背后的“本机运动执行代理”，专门处理手动控制这一路。

### 3. launch-mqtt-bridge

- 作用：这是 ROS2 启动桥接服务，负责接收 MQTT 指令，再真正执行 `ros2 launch` 或 `ros2 run`。
- systemd 做了什么：
  - `launch-mqtt-bridge.service` 拉起 `/opt/demo-control/deploy/jetson/start_launch_mqtt_bridge.sh`。
  - 它保证桥接程序在后台常驻，随时准备接收 MQTT 指令。
  - 如果桥接程序异常退出，systemd 可以负责重新拉起。
- 启动脚本做了什么：
  - 脚本文件：`/opt/demo-control/deploy/jetson/start_launch_mqtt_bridge.sh`
  - 进入 `/opt/demo-control/jetson-agent` 目录。
  - 加载 ROS2 基础环境和 `~/wheeltec_ros2/install/setup.bash` 工作区环境，确保 `ros2 launch`、`ros2 run`、目标包都能被找到。
  - 导出 MQTT 相关环境变量，比如 `MQTT_BROKER_HOST=127.0.0.1`、`MQTT_BROKER_PORT=1883`、`MQTT_DEVICE_ID=robot-001`。
  - 最后执行 Python 桥接程序 `launch_mqtt_bridge.py`。
- 主程序做了什么：
  - 主程序文件：`/opt/demo-control/jetson-agent/launch_mqtt_bridge.py`
  - 连接 MQTT broker，订阅控制主题。
  - 收到命令后解析 `action`、`package`、`launchFile`、`nodeExecutable` 等字段。
  - 对 `start_launch`：使用子进程启动 `ros2 launch <package> <launchFile>`。
  - 对 `start_node`：使用子进程启动 `ros2 run <package> <nodeExecutable>`。
  - 对 `stop_launch`、`stop_node`：根据内部记录找到对应进程并停止。
  - 维护 `launch_processes`、`node_processes`、`one_shot_processes` 之类的进程表，用来跟踪每个 launch/node 当前是否还活着。
  - 监控子进程退出事件，并把状态、错误、心跳重新发布回 MQTT，让 `demo-control` 能同步展示出来。
  - 当前回充相关功能基本都落在这里执行，包括：
    - `wheeltec_nav2/wheeltec_nav2.launch.py`
    - `auto_recharge_ros2/navigate_to_charger`
    - `auto_recharge_ros2/cancel_charge`
- 关键文件：
  - 程序入口：`/opt/demo-control/jetson-agent/launch_mqtt_bridge.py`
  - 启动脚本：`/opt/demo-control/deploy/jetson/start_launch_mqtt_bridge.sh`
  - systemd：`/etc/systemd/system/launch-mqtt-bridge.service`
- 可以理解为：它是“接口请求”到“ROS2 真正执行”之间的核心翻译层，也是 launch/node 生命周期总管。

### 4. wheeltec-mic

- 作用：这是语音相关功能的专用 systemd 服务，用来把两条语音 launch 固定包装成一个常驻服务。
- systemd 做了什么：
  - `wheeltec-mic.service` 拉起 `/opt/demo-control/deploy/jetson/start_wheeltec_mic.sh`。
  - 它不是对外提供 HTTP 接口，而是单纯托管语音相关 ROS2 launch。
  - 配置了 `Restart=always` 这类策略后，只要整个语音服务退出，systemd 就会尝试重新拉起。
- 启动脚本做了什么：
  - 脚本文件：`/opt/demo-control/deploy/jetson/start_wheeltec_mic.sh`
  - 进入 `/opt/demo-control` 工作目录。
  - 加载 ROS2 基础环境和 `~/wheeltec_ros2/install/setup.bash`。
  - 先后台启动 `ros2 launch wheeltec_mic_ros2 mic_init.launch.py`。
  - 等待 `MIC_INIT_DELAY` 秒，默认是 `3` 秒。
  - 再后台启动 `ros2 launch wheeltec_mic_ros2 base.launch.py`。
  - 脚本内部有 `trap cleanup`，收到退出信号时会尝试把两个子进程一起结束。
  - 使用 `wait -n` 监控两个 launch，只要其中一个先退出，就结束整个脚本，让 systemd 统一处理后续重启。
- 主程序实际做了什么：
  - 这个服务没有像前面三个那样的单一 Python/Node 主程序，它的“主体逻辑”就在 `start_wheeltec_mic.sh` 这个 bash 脚本里。
  - 真正被拉起的是两个 ROS2 launch：
    - `wheeltec_mic_ros2/mic_init.launch.py`
    - `wheeltec_mic_ros2/base.launch.py`
  - 也就是说，这个服务本质上是“用一个脚本托管两个 ROS2 launch 进程”。
- 关键文件：
  - 启动脚本：`/opt/demo-control/deploy/jetson/start_wheeltec_mic.sh`
  - systemd：`/etc/systemd/system/wheeltec-mic.service`
- 可以理解为：它是独立于 `demo-control / robot-agent / launch-mqtt-bridge` 之外的“语音功能常驻服务”。

### 5. device-reporter

- 作用：这是动态 IP 上报服务，专门负责在小车网络状态变化时，主动把“当前是否可直连、当前局域网 IP 是多少”上报给中控/后端。
- systemd 做了什么：
  - `device-reporter.service` 拉起 `/opt/demo-control/deploy/jetson/start_device_reporter.sh`。
  - 这个服务会在开机后常驻运行，但它不会把 AP / 热点模式下的地址当成可直连地址直接上报。
  - 它独立运行，哪怕后端暂时不可用，也不会影响 `demo-control`、`robot-agent`、`launch-mqtt-bridge`、`wheeltec-mic` 这几个已有服务。
- 启动脚本做了什么：
  - 脚本文件：`/opt/demo-control/deploy/jetson/start_device_reporter.sh`
  - 进入 `/opt/demo-control/jetson-agent` 目录。
  - 读取环境变量，例如 `DEVICE_REPORT_URL`、`DEVICE_REPORT_CHECK_INTERVAL`、`DEVICE_REPORT_HEARTBEAT_INTERVAL`、`DEVICE_REPORT_INCLUDE_WAITING_STATE`。
  - 如果存在 `/opt/demo-control/.venv`，会先激活虚拟环境。
  - 最后执行 `python3 device_reporter.py`，启动动态 IP 上报程序。
- 主程序做了什么：
  - 主程序文件：`/opt/demo-control/jetson-agent/device_reporter.py`
  - 持续检测无线网卡当前是 AP 模式，还是正常 Wi-Fi 客户端模式。
  - 如果还是 AP 模式，会进入等待态，避免把热点地址误上报成“前端可直连地址”。
  - 如果已经切到正常 Wi-Fi 客户端模式，并拿到可用的局域网 IPv4，就把 `directConnectIp` 上报给后端。
  - 当网络模式变化、IP 变化、到达心跳周期时，会再次上报。
  - 如果后端接口暂时不可达，它只会继续重试并写日志，不会把其它控制链路带崩。
- 关键文件：
  - 程序入口：`/opt/demo-control/jetson-agent/device_reporter.py`
  - 启动脚本：`/opt/demo-control/deploy/jetson/start_device_reporter.sh`
  - systemd：`/etc/systemd/system/device-reporter.service`
- 可以理解为：它是 Jetson 侧专门负责“告诉外部：我现在能不能直连、该连哪个 IP”的后台汇报员。

### 五个服务之间的关系

- `demo-control`：对外 HTTP 入口，别人先访问它。
- `robot-agent`：处理手动控制、本机运动代理。
- `launch-mqtt-bridge`：处理 launch / node 的实际执行。
- `wheeltec-mic`：独立管理语音相关 ROS2 launch。
- `device-reporter`：独立管理当前网络模式和局域网 IP 的主动上报。

- 简单链路可以这样记：
  - 手动控制：`外部请求 -> demo-control -> robot-agent -> ROS2 /cmd_vel`
  - 回充控制：`外部请求 -> demo-control -> MQTT -> launch-mqtt-bridge -> ros2 launch/run`
  - 语音功能：`systemd -> wheeltec-mic -> mic_init.launch.py + base.launch.py`
  - 动态 IP 上报：`systemd -> device-reporter -> 检测网络模式/IP -> POST 到中控/后端`

# Jetson 相关服务常用指令合集

## 1. systemd 服务状态

```bash
systemctl status demo-control --no-pager
systemctl status robot-agent --no-pager
systemctl status launch-mqtt-bridge --no-pager
systemctl status wheeltec-mic --no-pager
systemctl status device-reporter --no-pager
```

## 2. 启动 / 重启 / 停止服务

```bash
sudo systemctl start demo-control
sudo systemctl start robot-agent
sudo systemctl start launch-mqtt-bridge
sudo systemctl start wheeltec-mic
sudo systemctl start device-reporter
```

```bash
sudo systemctl restart demo-control
sudo systemctl restart robot-agent
sudo systemctl restart launch-mqtt-bridge
sudo systemctl restart wheeltec-mic
sudo systemctl restart device-reporter
```

```bash
sudo systemctl stop demo-control
sudo systemctl stop robot-agent
sudo systemctl stop launch-mqtt-bridge
sudo systemctl stop wheeltec-mic
sudo systemctl stop device-reporter
```

## 3. 开机自启

```bash
sudo systemctl enable demo-control
sudo systemctl enable robot-agent
sudo systemctl enable launch-mqtt-bridge
sudo systemctl enable wheeltec-mic
sudo systemctl enable device-reporter
```

```bash
sudo systemctl disable demo-control
sudo systemctl disable robot-agent
sudo systemctl disable launch-mqtt-bridge
sudo systemctl disable wheeltec-mic
sudo systemctl disable device-reporter
```

## 4. 查看日志

```bash
journalctl -u demo-control -n 200 --no-pager
journalctl -u robot-agent -n 200 --no-pager
journalctl -u launch-mqtt-bridge -n 200 --no-pager
journalctl -u wheeltec-mic -n 200 --no-pager
journalctl -u device-reporter -n 200 --no-pager
```

```bash
journalctl -u demo-control -f
journalctl -u robot-agent -f
journalctl -u launch-mqtt-bridge -f
journalctl -u wheeltec-mic -f
journalctl -u device-reporter -f
```

## 5. 查看服务配置

```bash
systemctl cat demo-control
systemctl cat robot-agent
systemctl cat launch-mqtt-bridge
systemctl cat wheeltec-mic
systemctl cat device-reporter
```

```bash
systemctl show demo-control -p User -p WorkingDirectory -p Environment -p ExecStart
systemctl show robot-agent -p User -p WorkingDirectory -p Environment -p ExecStart
systemctl show launch-mqtt-bridge -p User -p WorkingDirectory -p Environment -p ExecStart
systemctl show wheeltec-mic -p User -p WorkingDirectory -p Environment -p ExecStart
systemctl show device-reporter -p User -p WorkingDirectory -p Environment -p ExecStart
```

## 6. 进入项目目录

```bash
cd /opt/demo-control
cd /opt/demo-control/server
cd /opt/demo-control/jetson-agent
cd /opt/demo-control/deploy/jetson
```

## 7. 手动启动脚本

```bash
cd /opt/demo-control/deploy/jetson
bash start_demo_control.sh
bash start_robot_agent.sh
bash start_launch_mqtt_bridge.sh
bash start_wheeltec_mic.sh
bash start_device_reporter.sh
```

## 8. 检查端口监听

```bash
ss -lntp | grep 3000
ss -lntp | grep 8001
ss -lntp | grep 1883
```

## 9. 查看进程

```bash
ps -ef | grep demo-control
ps -ef | grep robot_agent.py
ps -ef | grep launch_mqtt_bridge.py
ps -ef | grep wheeltec_mic_ros2
ps -ef | grep device_reporter.py
ps -ef | grep ros2
```

## 10. 检查 Node / Python / ROS 环境

```bash
node -v
npm -v
python3 --version
which ros2
printenv | grep -E 'ROS|RMW|AMENT|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|PYTHONPATH'
```

## 11. 手动加载 ROS 环境

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
```

## 12. 查看 ROS 图

```bash
ros2 node list
ros2 topic list
ros2 action list
ros2 service list
```

## 13. 检查 auto_recharge_ros2 可执行项

```bash
ros2 pkg executables auto_recharge_ros2
```

## 14. 手动测试语音相关 launch

```bash
ros2 launch wheeltec_mic_ros2 mic_init.launch.py
```

```bash
ros2 launch wheeltec_mic_ros2 base.launch.py
```

## 15. 手动测试回充相关命令

```bash
ros2 run auto_recharge_ros2 navigate_to_charger
```

```bash
ros2 run auto_recharge_ros2 cancel_charge
```

## 16. MQTT bridge 本地接口状态排查

```bash
curl http://127.0.0.1:3000/api/status
curl http://127.0.0.1:8001/api/status
```

## 17. 从 Jetson 本机发开始回充

```bash
curl -X POST http://127.0.0.1:3000/api/command \
  -H 'Content-Type: application/json' \
  -d '{
    "deviceId": "robot-001",
    "action": "start_launch",
    "package": "wheeltec_nav2",
    "launchFile": "wheeltec_nav2.launch.py"
  }'
```

```bash
curl -X POST http://127.0.0.1:3000/api/command \
  -H 'Content-Type: application/json' \
  -d '{
    "deviceId": "robot-001",
    "action": "start_node",
    "package": "auto_recharge_ros2",
    "nodeExecutable": "navigate_to_charger"
  }'
```

## 18. 从 Jetson 本机发取消回充

```bash
curl -X POST http://127.0.0.1:3000/api/command \
  -H 'Content-Type: application/json' \
  -d '{
    "deviceId": "robot-001",
    "action": "start_node",
    "package": "auto_recharge_ros2",
    "nodeExecutable": "cancel_charge"
  }'
```

## 19. wheeltec-mic 单服务部署

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_wheeltec_mic.sh
sudo cp /opt/demo-control/deploy/jetson/wheeltec-mic.service /etc/systemd/system/wheeltec-mic.service
sudo systemctl daemon-reload
sudo systemctl enable wheeltec-mic.service
sudo systemctl start wheeltec-mic.service
```

```bash
sudo systemctl status wheeltec-mic --no-pager
journalctl -u wheeltec-mic -n 100 --no-pager
```

## 20. device-reporter 动态 IP 上报服务部署

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_device_reporter.sh
sudo cp /opt/demo-control/deploy/jetson/device-reporter.service /etc/systemd/system/device-reporter.service
sudo systemctl daemon-reload
sudo systemctl enable device-reporter.service
sudo systemctl start device-reporter.service
```

```bash
sudo systemctl status device-reporter --no-pager
journalctl -u device-reporter -n 100 --no-pager
journalctl -u device-reporter -f
```

```bash
systemctl cat device-reporter
systemctl show device-reporter -p Environment -p ExecStart
```

## 21. Windows 侧常用接口测试

```powershell
Invoke-RestMethod -Uri "http://192.168.124.162:3000/api/status" -Method Get | ConvertTo-Json -Depth 8
```

```powershell
& "C:\Users\Lenovo\Desktop\开始回充.ps1"
& "C:\Users\Lenovo\Desktop\取消回充.ps1"
```

## 22. 常用文件位置

```bash
/opt/demo-control/server/index.js
/opt/demo-control/jetson-agent/robot_agent.py
/opt/demo-control/jetson-agent/launch_mqtt_bridge.py
/opt/demo-control/deploy/jetson/start_demo_control.sh
/opt/demo-control/deploy/jetson/start_robot_agent.sh
/opt/demo-control/deploy/jetson/start_launch_mqtt_bridge.sh
/opt/demo-control/deploy/jetson/start_wheeltec_mic.sh
/opt/demo-control/deploy/jetson/start_device_reporter.sh
/etc/systemd/system/demo-control.service
/etc/systemd/system/robot-agent.service
/etc/systemd/system/launch-mqtt-bridge.service
/etc/systemd/system/wheeltec-mic.service
/etc/systemd/system/device-reporter.service
```

## 23. 修改 service 后重新加载

```bash
sudo systemctl daemon-reload
sudo systemctl restart demo-control
sudo systemctl restart robot-agent
sudo systemctl restart launch-mqtt-bridge
sudo systemctl restart wheeltec-mic
sudo systemctl restart device-reporter
```
