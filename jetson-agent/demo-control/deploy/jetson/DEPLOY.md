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
