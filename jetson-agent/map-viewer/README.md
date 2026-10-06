# Standalone Jetson Map Viewer

这是一套独立于 `backup2` 的地图刷新和展示服务，不修改现有 `demo-control` / `jetson-agent`。

## 功能说明

- 每 5 秒向 `http://127.0.0.1:3000/api/command` 发送一次保存地图请求。
- 请求体固定使用：

```json
{
  "deviceId": "robot-001",
  "action": "call_service",
  "servicePath": "/stemm/save_map",
  "serviceType": "std_srvs/srv/Trigger",
  "serviceArgs": {}
}
```

- 每次请求后读取 Jetson 本地地图文件：
  `/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm`
- 自动把 PGM 转成 PNG，通过浏览器页面展示。
- 前端页面只保留一个地图框，用于显示最新地图。

## 目录说明

```text
map-viewer/
  map_viewer_server.py
  start_map_viewer.sh
  map-viewer.service
  README.md
  web/
    index.html
    style.css
    app.js
```

## 运行方式一：直接运行

```bash
cd /opt/map-viewer
chmod +x start_map_viewer.sh
python3 map_viewer_server.py
```

默认监听：

- 页面：`http://<JetsonIP>:8091/`
- 地图接口：`http://<JetsonIP>:8091/api/map/latest`
- 状态接口：`http://<JetsonIP>:8091/api/status`

## 运行方式二：systemd 常驻运行

### 1. 拷贝目录到 Jetson

建议把整个 `map-viewer` 目录放到：

```bash
/opt/map-viewer
```

### 2. 给启动脚本执行权限

```bash
cd /opt/map-viewer
chmod +x start_map_viewer.sh
```

### 3. 安装 systemd 服务

```bash
sudo cp /opt/map-viewer/map-viewer.service /etc/systemd/system/map-viewer.service
sudo systemctl daemon-reload
sudo systemctl enable map-viewer.service
sudo systemctl start map-viewer.service
```

### 4. 查看服务状态

```bash
sudo systemctl status map-viewer --no-pager
journalctl -u map-viewer -n 100 --no-pager
journalctl -u map-viewer -f
```

## 常用验证步骤

### 1. 先确认原有 demo-control 正在运行

```bash
curl http://127.0.0.1:3000/api/status
```

### 2. 再确认地图服务状态

```bash
curl http://127.0.0.1:8091/api/status
```

正常时会看到类似字段：

- `mapReady`
- `mapUpdatedAt`
- `lastAttemptAt`
- `lastSuccessAt`
- `lastError`

### 3. 浏览器打开地图页

```text
http://<JetsonIP>:8091/
```

### 4. 单独检查地图接口

```bash
curl -I http://127.0.0.1:8091/api/map/latest
```

如果已经拿到地图，会返回：

- `HTTP/1.0 200 OK`
- `Content-Type: image/png`
- `X-Map-Updated-At: ...`

## 可调参数

支持以下环境变量：

- `MAP_VIEWER_HOST`
- `MAP_VIEWER_PORT`
- `MAP_VIEWER_COMMAND_URL`
- `MAP_VIEWER_MAP_FILE`
- `MAP_VIEWER_INTERVAL`
- `MAP_VIEWER_DEVICE_ID`

默认值已经写在 `start_map_viewer.sh` 和 `map-viewer.service` 里。

## 注意事项

- 这套服务不会修改 `backup2` 的任何代码。
- 它依赖 `backup2` 已有的 `demo-control` 的 `POST /api/command` 能正常调用 `save_map`。
- 如果地图文件路径变了，只改 `MAP_VIEWER_MAP_FILE` 即可。
- 当前实现使用 Python 标准库，不额外依赖 Pillow 等三方库。
