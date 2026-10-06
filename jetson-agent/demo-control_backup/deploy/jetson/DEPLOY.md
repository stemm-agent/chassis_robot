# Jetson 部署说明（backup2 / 老接口）

## 1. 安装运行环境

```bash
sudo apt-get update
sudo apt-get install -y nodejs npm python3 python3-venv
```

## 2. 拷贝项目到小车

```bash
sudo mkdir -p /opt/demo-control
sudo cp -r /path/to/demo-control/* /opt/demo-control/
sudo chown -R wheeltec:wheeltec /opt/demo-control
```

## 3. 安装 Node.js 和 Python 依赖

```bash
cd /opt/demo-control
rm -rf node_modules package-lock.json
npm install
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r jetson-agent/requirements.txt
```

这样会重新安装一套兼容 Jetson 当前 Node.js 版本的依赖，避免 `mqtt` 新版本里的 `?.` 语法报错。

`--system-site-packages` 是为了让虚拟环境也能访问 ROS2 自带的 `rclpy` 等 Python 包。

`launch-mqtt-bridge` 还会额外加载 `~/wheeltec_ros2/install/setup.bash`，这样才能找到 `wheeltec_nav2`、`turn_on_wheeltec_robot`、`auto_recharge_ros2` 这类业务包。

## 4. 手工启动 8001 robot_agent

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_robot_agent.sh
APP_ROOT=/opt/demo-control ./deploy/jetson/start_robot_agent.sh
```

如果之前出现过 `AMENT_TRACE_SETUP_FILES: unbound variable`，现在直接重新执行上面的命令即可，不需要再手工 `set +u`。

## 5. 手工启动 3000 demo-control

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_demo_control.sh
APP_ROOT=/opt/demo-control APP_HOST=0.0.0.0 APP_PORT=3000 JETSON_AGENT_URL=http://127.0.0.1:8001 ./deploy/jetson/start_demo_control.sh
```

## 6. 验证服务

```bash
curl http://127.0.0.1:8001/api/status
curl http://127.0.0.1:3000/api/status
ss -lntp | grep 8001
ss -lntp | grep 3000
```

## 7. 配置为 systemd 常驻服务

```bash
sudo cp /opt/demo-control/deploy/jetson/robot-agent.service /etc/systemd/system/robot-agent.service
sudo cp /opt/demo-control/deploy/jetson/launch-mqtt-bridge.service /etc/systemd/system/launch-mqtt-bridge.service
sudo cp /opt/demo-control/deploy/jetson/demo-control.service /etc/systemd/system/demo-control.service
sudo systemctl daemon-reload
sudo systemctl enable robot-agent launch-mqtt-bridge demo-control
sudo systemctl start robot-agent launch-mqtt-bridge demo-control

sudo systemctl status robot-agent
sudo systemctl status launch-mqtt-bridge
sudo systemctl status demo-control
```

## 8. 局域网验证

```bash
curl http://127.0.0.1:3000/api/status
curl http://192.168.124.162:3000/api/status
```

小程序最终基地址：

```text
http://192.168.124.162:3000
```

## 9. 配置语音相关 launch 为单个 systemd 服务

对应原始命令：

```bash
ros2 launch wheeltec_mic_ros2 mic_init.launch.py
ros2 launch wheeltec_mic_ros2 base.launch.py
```

先确保脚本有执行权限：

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_wheeltec_mic.sh
```

复制 service 文件并设置开机自启：

```bash
sudo cp /opt/demo-control/deploy/jetson/wheeltec-mic.service /etc/systemd/system/wheeltec-mic.service
sudo systemctl daemon-reload
sudo systemctl enable wheeltec-mic.service
sudo systemctl start wheeltec-mic.service
```

这个单服务会按顺序启动：

1. `ros2 launch wheeltec_mic_ros2 mic_init.launch.py`
2. 等待 `MIC_INIT_DELAY` 秒（默认 `3` 秒）
3. `ros2 launch wheeltec_mic_ros2 base.launch.py`

查看状态和日志：

```bash
sudo systemctl status wheeltec-mic --no-pager
journalctl -u wheeltec-mic -n 100 --no-pager
```

## 10. 配置动态 IP 上报服务（Jetson 主动上报）

这个服务会在开机后常驻运行，但不会把 AP 模式下的热点地址当成可直连地址上报。

它的行为是：

- 开机后持续检测无线网卡状态
- 如果当前还是 AP / 热点模式，只上报等待状态（或按配置跳过等待态上报）
- 只有切到正常 Wi-Fi 客户端模式并拿到可用局域网 IPv4 后，才上报 `directConnectIp`
- 后续如果 IP 变化、模式变化，或者到了心跳周期，也会再次上报

先确保脚本有执行权限：

```bash
cd /opt/demo-control
chmod +x deploy/jetson/start_device_reporter.sh
```

复制 service 文件：

```bash
sudo cp /opt/demo-control/deploy/jetson/device-reporter.service /etc/systemd/system/device-reporter.service
```

按实际后端地址修改 service 里的上报地址，例如：

```bash
sudo nano /etc/systemd/system/device-reporter.service
```

重点看这几个环境变量：

- `DEVICE_REPORT_URL`：后端接收上报的接口地址
- `MQTT_DEVICE_ID`：设备 ID，没有单独配置时会复用它
- `DEVICE_REPORT_CHECK_INTERVAL`：状态轮询间隔，默认 `5` 秒
- `DEVICE_REPORT_HEARTBEAT_INTERVAL`：心跳上报间隔，默认 `60` 秒
- `DEVICE_REPORT_INCLUDE_WAITING_STATE`：是否把 AP / 等待 IP 状态也发给后端，默认 `true`

重新加载并启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable device-reporter.service
sudo systemctl start device-reporter.service
```

查看状态和日志：

```bash
sudo systemctl status device-reporter --no-pager
journalctl -u device-reporter -n 100 --no-pager
journalctl -u device-reporter -f
```

手工前台测试：

```bash
cd /opt/demo-control
APP_ROOT=/opt/demo-control DEVICE_REPORT_URL=https://stemm.hnh666.com/api/device/report ./deploy/jetson/start_device_reporter.sh
```

测试建议：

1. 小车刚开机、仍处于 AP 模式时，查看日志里应出现 `waiting_ap` 或类似等待信息。
2. 让小车切到正常 Wi-Fi 客户端模式后，再查看日志，应出现 `ready`，并带上新的局域网 IP。
3. 手动切换到另一个 Wi-Fi，或者让 DHCP 重新分配地址，确认日志会再次上报新的 IP。
4. 临时关闭后端接口时，确认 reporter 只会打印失败日志，不会影响 `demo-control`、`robot-agent`、`launch-mqtt-bridge`、`wheeltec-mic`。
