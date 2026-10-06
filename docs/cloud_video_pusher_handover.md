# 机器人视频推流方案交接文档

更新时间：2026-07-01

## 1. 当前目标

本方案用于让机器人端把 ROS2 摄像头画面提供给小程序/前端查看，分为两条链路：

1. 云端链路：机器人把 `/camera/color/image_raw` 推到阿里云 ECS 上的自建媒体服务，前端/小程序拉 HLS 播放地址。
2. 本地链路：机器人在局域网内提供 MJPEG/JPEG 轮询 HTTP 服务，前端/小程序从机器人 IP 拉 snapshot（JPEG）或 MJPEG 流，不经过云端。

当前阿里云云直播不可用，因此云端方案按“公仔机器人类似逻辑”走自建媒体服务/SRS：机器人只负责推 RTMP，服务器负责把 RTMP 转成 HLS。

## 2. 工作空间与包位置

工作空间：

```bash
/mnt/wheeltec_ros2
```

新增/修改的核心包：

```text
src/cloud_video_pusher/
├── cloud_video_pusher/
│   ├── rtmp_image_pusher.py
│   └── local_hls_image_server.py
├── launch/
│   ├── rtmp_pusher.launch.py
│   └── local_hls_server.launch.py
├── package.xml
├── setup.py
└── README.md
```

已使用的 Python 环境：

```bash
/mnt/wheeltec_ros2/.venv/bin/python
```

注意：用户倾向于手动编译，不要随意自动执行 `colcon build`，除非用户明确要求。

## 3. 总体架构

### 3.1 云端推流架构

```text
turn_on_wheeltec_robot/wheeltec_camera.launch.py
    -> 发布 /camera/color/image_raw
    -> cloud_video_pusher/rtmp_image_pusher
    -> ffmpeg rawvideo/libx264
    -> RTMP: rtmp://8.148.249.36/live/robot001
    -> ECS/SRS 转 HLS
    -> 小程序 video 组件播放 http://8.148.249.36:8088/live/robot001.m3u8
```

当前云端默认参数：

| 参数 | 默认值 |
| --- | --- |
| `image_topic` | `/camera/color/image_raw` |
| `rtmp_base_url` | `rtmp://8.148.249.36/live` |
| `hls_base_url` | `http://8.148.249.36:8088/live` |
| `stream_name` | `robot001` |
| `hub_id` | `910007` |
| `camera_device_id` | `robot-camera-1` |
| `width` | `640` |
| `height` | `480` |
| `fps` | `15.0` |
| `bitrate` | `800k` |
| `auto_start` | `true` |

最终默认地址：

```text
RTMP 推流地址：rtmp://8.148.249.36/live/robot001
HLS 播放地址：http://8.148.249.36:8088/live/robot001.m3u8
```

### 3.2 本地局域网 MJPEG/JPEG 架构

```text
用户自行启动摄像头
    -> 发布 /camera/color/image_raw
    -> cloud_video_pusher/local_hls_image_server
    -> ROS Image → cv_bridge → OpenCV → cv2.imencode(.jpg)
    -> Python ThreadingHTTPServer
    -> 小程序用 image 组件轮询 http://机器人局域网IP:18080/live/robot001.jpg
    -> 也可直接用 MJPEG 流：http://机器人局域网IP:18080/live/robot001.mjpg
```

当前本地默认参数：

| 参数 | 默认值 |
| --- | --- |
| `image_topic` | `/camera/color/image_raw` |
| `bind_address` | `0.0.0.0` |
| `public_host` | 空，节点自动探测本机局域网 IP |
| `http_port` | `18080` |
| `stream_app` | `live` |
| `stream_name` | `robot001` |
| `hub_id` | `910007` |
| `camera_device_id` | `robot-camera-1` |
| `width` | `640` |
| `height` | `480` |
| `fps` | `8.0` |
| `jpeg_quality` | `75` |
| `poll_interval_ms` | `150` |
| `auto_start` | `true` |

本地 URL：

```text
JPEG snapshot:  http://机器人局域网IP:18080/live/robot001.jpg
MJPEG 流:      http://机器人局域网IP:18080/live/robot001.mjpg
短别名:        http://机器人局域网IP:18080/frame.jpg
              http://机器人局域网IP:18080/snapshot.jpg
              http://机器人局域网IP:18080/stream.mjpg
回话信息:      http://机器人局域网IP:18080/stream_session
```

如果自动识别的 IP 不对，可以启动时指定：

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py public_host:=192.168.2.123
```

## 4. 当前 launch 状态

### 4.1 云端 launch：集成版

文件：

```text
src/cloud_video_pusher/launch/rtmp_pusher.launch.py
```

当前它已经集成以下三项，用户只需要执行一个命令：

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py
```

内部启动顺序：

```text
1. Include turn_on_wheeltec_robot/launch/wheeltec_camera.launch.py
2. 2 秒后启动 web_video_server/web_video_server
3. 4 秒后启动 cloud_video_pusher/rtmp_image_pusher
```

说明：

- `wheeltec_camera.launch.py` 用于启动摄像头并发布 `/camera/color/image_raw`。
- `web_video_server` 保留用于本机调试/MJPEG 访问，默认端口一般为 `8090`。
- `rtmp_image_pusher` 负责真正推云端 RTMP。
- 这里使用 `TimerAction` 是为了尽量保证摄像头先起来，再启动 web 调试服务和推流节点。

### 4.2 本地 launch：单节点版

文件：

```text
src/cloud_video_pusher/launch/local_hls_server.launch.py
```

按用户最新要求，已经改回“单 launch/单节点”模式，只启动：

```text
cloud_video_pusher/local_hls_image_server
```

当前它不会自动启动：

```text
turn_on_wheeltec_robot/wheeltec_camera.launch.py
web_video_server/web_video_server
```

如果要完整本地调试，需要手动分开启动：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
ros2 run web_video_server web_video_server
ros2 launch cloud_video_pusher local_hls_server.launch.py
```

如果只需要本地 HLS 且摄像头已经由其他 launch 启动，则只运行：

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py
```

## 5. 节点说明

### 5.1 rtmp_image_pusher

文件：

```text
src/cloud_video_pusher/cloud_video_pusher/rtmp_image_pusher.py
```

入口注册：

```text
rtmp_image_pusher = cloud_video_pusher.rtmp_image_pusher:main
```

功能：

- 订阅 ROS 图像话题，默认 `/camera/color/image_raw`。
- 使用 `cv_bridge` 转为 OpenCV BGR 图像。
- 按 `width/height/fps` 调整图像。
- 通过 stdin 把 rawvideo 喂给 `ffmpeg`。
- `ffmpeg` 使用 `libx264` 编码并以 RTMP/FLV 格式推到服务器。
- 发布状态话题和会话 JSON，供上层或调试使用。
- 提供 `SetBool` 服务控制开始/停止推流。

话题/服务：

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| publisher | `/rtmp_image_pusher/stream_status` | 字符串状态，例如 `streaming`、`idle`、`error:no_rtmp_url` |
| publisher | `/rtmp_image_pusher/stream_session` | 给后端/小程序参考的 JSON 会话信息 |
| service | `/rtmp_image_pusher/set_streaming` | `std_srvs/srv/SetBool`，true 开始，false 停止 |

示例控制：

```bash
ros2 service call /rtmp_image_pusher/set_streaming std_srvs/srv/SetBool "{data: true}"
ros2 service call /rtmp_image_pusher/set_streaming std_srvs/srv/SetBool "{data: false}"
```

会话 payload 示例：

```json
{
  "hubId": "910007",
  "cameraDeviceId": "robot-camera-1",
  "streamSessionId": "robot001",
  "rtmpUrl": "rtmp://8.148.249.36/live/robot001",
  "hlsUrl": "http://8.148.249.36:8088/live/robot001.m3u8",
  "playUrl": "http://8.148.249.36:8088/live/robot001.m3u8",
  "playType": "hls",
  "streamActive": true,
  "sessionTtlSeconds": 60,
  "uploadIntervalMs": 15000
}
```

### 5.2 local_hls_image_server

文件：

```text
src/cloud_video_pusher/cloud_video_pusher/local_hls_image_server.py
```

入口注册：

```text
local_hls_image_server = cloud_video_pusher.local_hls_image_server:main
```

功能（已从 HLS 改为 MJPEG/JPEG）：

- 订阅 ROS 图像话题，默认 `/camera/color/image_raw`。
- 使用 `cv_bridge` 转 OpenCV BGR 图像，按 `width/height` resize，`cv2.imencode('.jpg')` 编码。
- 使用 Python `ThreadingHTTPServer` 直接返回 JPEG snapshot 或 MJPEG multipart 流。
- 不再依赖 ffmpeg 和磁盘 HLS 分片文件。
- 自动添加 CORS 和 no-cache 头，方便小程序/浏览器测试。
- 自动探测本机可访问 IP；也可用 `public_host` 显式指定。
- 提供状态话题、会话话题和 `SetBool` 控制服务。

话题/服务：

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| publisher | `/local_hls_image_server/stream_status` | 状态字符串 |
| publisher | `/local_hls_image_server/stream_session` | 本地 MJPEG 会话 JSON |
| service | `/local_hls_image_server/set_streaming` | `std_srvs/srv/SetBool`，true 开始，false 停止 |

本地 payload 示例：

```json
{
  "hubId": "910007",
  "cameraDeviceId": "robot-camera-1",
  "streamSessionId": "robot001",
  "rtmpUrl": "",
  "hlsUrl": "",
  "mjpegUrl": "http://192.168.2.123:18080/live/robot001.mjpg",
  "snapshotUrl": "http://192.168.2.123:18080/live/robot001.jpg",
  "playUrl": "http://192.168.2.123:18080/live/robot001.jpg",
  "playType": "mjpeg",
  "pollIntervalMs": 150,
  "streamActive": true,
  "sessionTtlSeconds": 60,
  "uploadIntervalMs": 15000,
  "localOnly": true
}
```

## 6. 小程序/前端交接重点

小程序不要直接播放 RTMP。

**本地局域网模式（推荐低延迟方案）：**

使用 `image` 组件 + 时间戳轮询 snapshot URL：

```js
// 小程序侧示例
src = snapshotUrl + '?t=' + Date.now()
// 每 pollIntervalMs 毫秒刷新一次
```

或使用 `video` 组件播放 MJPEG 流：

```text
src = mjpegUrl
```

**云端模式：**

统一使用普通 `video` 组件播放 HLS：

```text
src = hlsUrl || playUrl
```

推荐字段：

| 字段 | 说明 |
| --- | --- |
| `hubId` | 设备/机器人归属 ID，当前默认 `910007` |
| `cameraDeviceId` | 摄像头 ID，当前默认 `robot-camera-1` |
| `streamSessionId` | 流会话/流名，当前默认 `robot001` |
| `rtmpUrl` | 机器人推流地址；本地模式为空 |
| `hlsUrl` | 云端 HLS 播放地址；本地模式为空 |
| `mjpegUrl` | 本地 MJPEG 流地址（仅本地模式） |
| `snapshotUrl` | 本地 JPEG snapshot 地址（仅本地模式） |
| `playUrl` | 主要播放地址，本地为 snapshotUrl，云端为 hlsUrl |
| `playType` | 本地 `mjpeg`，云端 `hls` |
| `pollIntervalMs` | snapshot 轮询间隔（仅本地模式） |
| `streamActive` | 当前是否在推流 |
| `localOnly` | 仅本地模式有，值为 `true` |

本地模式要求：

- 小程序手机必须和机器人在同一局域网。
- 播放地址中的 IP 必须是手机能访问到的机器人 IP。
- 如果自动 IP 不对，机器人端用 `public_host:=机器人IP` 指定。
- 小程序同事推荐使用 `snapshotUrl` + 时间戳轮询（更低延迟、更兼容）。

## 7. 编译与运行

编译：

```bash
cd /mnt/wheeltec_ros2
colcon build --packages-select cloud_video_pusher
source install/setup.bash
```

云端一键运行：

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py
```

云端运行后关注：

```bash
ros2 topic echo /rtmp_image_pusher/stream_status
ros2 topic echo /rtmp_image_pusher/stream_session
```

本地手动三段运行：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
ros2 run web_video_server web_video_server
ros2 launch cloud_video_pusher local_hls_server.launch.py
```

本地运行后关注：

```bash
ros2 topic echo /local_hls_image_server/stream_status
ros2 topic echo /local_hls_image_server/stream_session
```

本地播放测试：

```text
浏览器直接打开:  http://机器人局域网IP:18080/live/robot001.jpg
MJPEG 流测试:   http://机器人局域网IP:18080/live/robot001.mjpg
会话信息:       http://机器人局域网IP:18080/stream_session
```

web_video_server 调试入口：

```text
http://机器人局域网IP:8090/
```

## 8. 服务器侧信息

已有 ECS：

```text
8.148.249.36
```

当前机器人端默认按以下地址使用：

```text
RTMP: rtmp://8.148.249.36/live/robot001
HLS : http://8.148.249.36:8088/live/robot001.m3u8
```

需要服务器侧满足：

- RTMP 端口可达，通常是 `1935`。
- HLS HTTP 端口可达，本方案默认使用 `8088`。
- ECS 安全组和系统防火墙都要放行相关端口。
- SRS/nginx/反代等服务必须实际运行。

之前排查信息：

- 阿里云云直播无法使用，所以转为自建媒体服务。
- 服务器 `8.148.249.36:1935` 曾出现超时，需要继续确认 SRS/端口/安全组。
- 服务器 `8080` 曾被其他 Python websockets 服务占用，因此 HLS 规划为 `8088`。

## 9. 依赖要求

机器人端需要：

- ROS2 Humble
- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `std_srvs`
- `cv_bridge`
- `python3-opencv`
- 系统命令 `ffmpeg`

检查 ffmpeg：

```bash
which ffmpeg
ffmpeg -version
```

如果缺失：

```bash
sudo apt update
sudo apt install -y ffmpeg
```

## 10. 已验证内容

已做过的验证：

```bash
/mnt/wheeltec_ros2/.venv/bin/python -m py_compile \
  src/cloud_video_pusher/launch/rtmp_pusher.launch.py \
  src/cloud_video_pusher/launch/local_hls_server.launch.py
```

本地 `local_hls_server.launch.py` 改回单节点后，也单独通过：

```bash
/mnt/wheeltec_ros2/.venv/bin/python -m py_compile \
  src/cloud_video_pusher/launch/local_hls_server.launch.py
```

未做/待验证：

- 用户手动 `colcon build --packages-select cloud_video_pusher` 后的完整构建。
- 机器人实机摄像头是否稳定发布 `/camera/color/image_raw`。
- 云端 `8.148.249.36:1935` 是否已可连通。
- 云端 HLS 地址 `http://8.148.249.36:8088/live/robot001.m3u8` 是否实际可播放。
- 小程序真机是否能播放本地/云端 HLS。

## 11. 常见问题排查

### 11.1 没有画面

检查图像话题：

```bash
ros2 topic list | grep image
ros2 topic hz /camera/color/image_raw
```

如果没有 `/camera/color/image_raw`，先启动摄像头 launch。

### 11.2 RTMP 推不上云

检查：

```bash
ros2 topic echo /rtmp_image_pusher/stream_status
```

常见状态：

| 状态 | 含义 |
| --- | --- |
| `error:no_rtmp_url` | 没生成/没传入 RTMP 地址 |
| `error:no_ffmpeg` | 机器人没有 ffmpeg |
| `error:start_failed` | ffmpeg 进程启动失败 |
| `streaming` | 已启动推流进程，不代表服务器一定收到 |

继续检查服务器端口：

```bash
nc -vz 8.148.249.36 1935
nc -vz 8.148.249.36 8088
```

### 11.3 本地 MJPEG/JPEG 无法访问

检查本机端口：

```bash
ss -lntp | grep 18080
```

浏览器直接访问 snapshot 测试：

```bash
curl -o /tmp/test.jpg http://机器人局域网IP:18080/live/robot001.jpg
file /tmp/test.jpg   # 应该输出 JPEG image data
```

检查会话信息：

```bash
curl http://机器人局域网IP:18080/stream_session | python3 -m json.tool
```

检查播放地址中的 IP 是否是小程序手机能访问到的机器人 IP。如果不是，用：

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py public_host:=机器人局域网IP
```

### 11.4 web_video_server 和 local_hls_image_server 的区别

`web_video_server`：

- 已有包，没有修改。
- 主要用于本机/浏览器调试 ROS 图像流。
- 默认端口一般为 `8090`。

`local_hls_image_server`：

- 本次新增，名称含 `hls` 是历史遗留。
- 现已改为 MJPEG/JPEG polling 服务，不再使用 ffmpeg/HLS。
- 专门给小程序提供低延迟 snapshot 轮询和 MJPEG 流。
- 默认端口 `18080`。

## 12. 当前 Git/代码注意事项

本次方案主要新增或修改：

```text
src/cloud_video_pusher/package.xml
src/cloud_video_pusher/setup.py
src/cloud_video_pusher/README.md
src/cloud_video_pusher/cloud_video_pusher/rtmp_image_pusher.py
src/cloud_video_pusher/cloud_video_pusher/local_hls_image_server.py
src/cloud_video_pusher/launch/rtmp_pusher.launch.py
src/cloud_video_pusher/launch/local_hls_server.launch.py
```

重要现状：

- 云端 `rtmp_pusher.launch.py` 是集成版：摄像头 + web_video_server + RTMP pusher。
- 本地 `local_hls_server.launch.py` 是单节点版：只启动 local MJPEG/JPEG 服务，不自动启动摄像头和 web_video_server。
- 没有修改现有 `web_video_server` 源码。
- 没有修改现有 `turn_on_wheeltec_robot/wheeltec_camera.launch.py`。
- 当前文档为交接下一智能体使用，后续如要改架构，先确认用户希望云端/本地 launch 是否继续保持这种差异。

## 13. 推荐下一步

1. 用户手动编译：

```bash
cd /mnt/wheeltec_ros2
colcon build --packages-select cloud_video_pusher
source install/setup.bash
```

2. 先测本地摄像头话题：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py
ros2 topic hz /camera/color/image_raw
```

3. 测本地 MJPEG/JPEG：

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py public_host:=机器人局域网IP
```

4. 确认服务器 `1935/8088` 可用后，再测云端：

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py
```

5. 小程序只接入 `hlsUrl || playUrl`，不要直接播放 RTMP。
