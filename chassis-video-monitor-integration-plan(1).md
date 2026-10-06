# 小车实时监控接入信息与实现方向

## 1. 当前目标

现在准备接入“小车实时监控”功能。目标是参考当前公仔监控器的视频链路，让小程序仍然通过云后端拿播放地址，而不是直接把小程序暴露到 ROS2 / Jetson 的底层接口。

当前建议先走：

```text
机器人本机 ROS2 摄像头
  -> 本机 web_video_server 或同类视频服务
  -> 通过 Tailscale / WireGuard / frp 打通到阿里云 ECS
  -> ECS 上的媒体服务转成小程序可播放地址
  -> 云后端返回 playUrl / hlsUrl
  -> 小程序 video 组件播放
```

## 2. 可以借鉴的公仔监控链路

小程序当前公仔监控器不是直接访问摄像头板，而是走云后端接口：

| 步骤         | 小程序动作                               | 当前接口                                              |
| ------------ | ---------------------------------------- | ----------------------------------------------------- |
| 开始实时监控 | 请求后端启动或获取流                     | `POST /app/camera/stream/start`                       |
| 播放视频     | 使用后端返回的 `hlsUrl` / `playUrl`      | 小程序 `<video>` 组件                                 |
| 保活         | 页面播放期间每 15 秒保活                 | `POST /app/camera/stream/keepalive`                   |
| 停止播放     | 离开页面或切换设备时停止                 | `POST /app/camera/stream/stop`                        |
| 录像         | 基于当前 `streamSessionId` 开始/停止录像 | `/app/camera/record/start`、`/app/camera/record/stop` |
| 拍照         | 基于当前 `streamSessionId` 截图          | `POST /app/camera/photo/capture`                      |

前端当前期望后端返回类似：

```json
{
  "hubId": 910007,
  "cameraDeviceId": "camera_xxx",
  "streamSessionId": "session_xxx",
  "streamActive": true,
  "sessionTtlSeconds": 60,
  "playUrl": "https://...",
  "hlsUrl": "https://...",
  "playType": "hls"
}
```

小车实时监控也建议复用这个模型，只是设备来源从公仔 camera 板换成小车摄像头。

## 3. 你需要提供给小车同事的信息

| 信息                 | 当前建议/已知值                                                            | 是否已确认      | 备注                                                        |
| -------------------- | -------------------------------------------------------------------------- | --------------- | ----------------------------------------------------------- |
| ECS 公网域名         | `stemm.hnh666.com`                                                         | 基本已在用      | 小程序云后端当前就是 `https://stemm.hnh666.com`             |
| ECS 公网 IP          | `8.148.249.36`                                                             | 待最终确认      | 之前宝塔/后端部署用过该 IP，建议再确认是否仍是当前 ECS      |
| ECS 系统版本         | 待确认                                                                     | 未确认          | 小车同事希望最好是 Ubuntu 22.04                             |
| 组网方式             | 建议先用 Tailscale                                                         | 待确认          | 联调最快，避免直接暴露机器人端口到公网                      |
| 机器人本机视频流地址 | 例：`http://机器人本机IP:8090/stream_viewer?topic=/camera/color/image_raw` | 需要小车端确认  | 必须先在机器人本机浏览器能打开                              |
| ROS2 图像话题名      | 建议先用 `/camera/color/image_raw`                                         | 需要小车端确认  | 以实际相机 topic 为准                                       |
| 外部播放格式         | HLS 优先                                                                   | 待小车/后端确认 | 小程序 `<video>` 对 HLS 更友好，避免 `live-player` 权限问题 |
| 小车设备 ID          | 当前小程序写死 `robot-001`                                                 | 临时可用        | 后续应由后端绑定关系返回                                    |

## 4. 建议选型

### 4.1 推荐先用 Tailscale

原因：

- 联调最快，不需要小车暴露公网端口。
- ECS 和机器人加入同一个 Tailscale 网络后，ECS 可以通过机器人 Tailscale IP 访问 `8090`。
- 机器人在不同局域网时，只要能联网，ECS 仍可访问。
- 安全性比直接公网开放 `8090` 好。

建议链路：

```text
机器人
  ROS2 camera topic
  -> web_video_server: http://127.0.0.1:8090/stream_viewer?topic=/camera/color/image_raw
  -> Tailscale IP: http://100.x.x.x:8090/stream_viewer?topic=/camera/color/image_raw

ECS
  -> 通过 Tailscale IP 拉机器人视频流
  -> 转 HLS / FLV / MP4 fragment
  -> 对小程序暴露 HTTPS 播放地址
```

### 4.2 不建议直接公网暴露机器人 8090

不建议这样：

```text
http://机器人公网IP:8090/stream_viewer?topic=...
```

原因：

- 机器人通常没有稳定公网 IP。
- 安全风险高。
- 微信小程序正式环境对域名、HTTPS、证书有要求。
- 后续录像、拍照、鉴权、权限控制不好做。

## 5. 小车端需要做的事情

### 5.1 先确认本机视频流可用

在机器人本机确认 ROS2 摄像头 topic 存在：

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

确认目标 topic，例如：

```text
/camera/color/image_raw
```

启动本机视频服务后，在机器人本机浏览器打开：

```text
http://127.0.0.1:8090/stream_viewer?topic=/camera/color/image_raw
```

或：

```text
http://机器人局域网IP:8090/stream_viewer?topic=/camera/color/image_raw
```

要求：本机页面能看到实时画面，才进入下一步。

### 5.2 安装并配置 Tailscale

机器人和 ECS 都加入同一个 Tailscale 网络。

小车同事需要提供：

```text
机器人 Tailscale IP：100.x.x.x
ECS Tailscale IP：100.x.x.x
```

ECS 上需要能访问：

```text
http://机器人TailscaleIP:8090/stream_viewer?topic=/camera/color/image_raw
```

### 5.3 ECS 媒体服务处理

ECS 侧需要把机器人本机视频流转成小程序可播放的 HTTPS 地址。

建议输出：

```text
https://stemm.hnh666.com/live/chassis_robot_001.m3u8
```

或后端返回：

```json
{
  "playType": "hls",
  "hlsUrl": "https://stemm.hnh666.com/live/chassis_robot_001.m3u8",
  "playUrl": "https://stemm.hnh666.com/live/chassis_robot_001.m3u8"
}
```

注意：小程序正式环境建议不要返回 `http://`、IP 地址、`localhost`、`127.0.0.1` 或 Tailscale 私网地址给前端。前端用户手机访问不到这些地址，或者会被微信域名规则限制。

## 6. 后端建议接口模型

为了复用公仔监控链路，建议新增或扩展小车监控接口，不建议让小程序直接请求机器人 `8090`。

### 6.1 开始小车监控

```http
POST /app/chassis/camera/stream/start
```

请求体：

```json
{
  "hubId": 910007,
  "deviceId": "robot-001",
  "topic": "/camera/color/image_raw",
  "fps": 13,
  "bitrate": "800k"
}
```

返回：

```json
{
  "streamSessionId": "chassis_stream_xxx",
  "deviceId": "robot-001",
  "streamActive": true,
  "sessionTtlSeconds": 60,
  "playType": "hls",
  "playUrl": "https://stemm.hnh666.com/live/chassis_robot_001.m3u8",
  "hlsUrl": "https://stemm.hnh666.com/live/chassis_robot_001.m3u8"
}
```

### 6.2 保活

```http
POST /app/chassis/camera/stream/keepalive
```

请求体：

```json
{
  "hubId": 910007,
  "deviceId": "robot-001",
  "streamSessionId": "chassis_stream_xxx"
}
```

### 6.3 停止

```http
POST /app/chassis/camera/stream/stop
```

请求体：

```json
{
  "hubId": 910007,
  "deviceId": "robot-001",
  "streamSessionId": "chassis_stream_xxx"
}
```

## 7. 前端小程序需要怎么接

当前小程序已有公仔监控逻辑，可以复用同样的页面生命周期：

```text
进入小车监控
  -> 调 start
  -> 拿 playUrl / hlsUrl
  -> video 播放
  -> 每 15 秒 keepalive

离开页面 / 切换摄像头 / 页面隐藏
  -> stop
  -> 清理 video 状态和定时器
```

小程序侧需要新增或扩展：

- `chassisCameraApi.startStream`
- `chassisCameraApi.keepaliveStream`
- `chassisCameraApi.stopStream`
- 在线操控页或监控页里选择“小车摄像头”时调用

前端不应该直接写：

```text
http://机器人IP:8090/stream_viewer?topic=...
```

前端应该只拿后端返回的：

```text
https://stemm.hnh666.com/live/xxx.m3u8
```

## 8. 需要小车同事明确回复的问题

| 问题                                     | 期望回复                                 |
| ---------------------------------------- | ---------------------------------------- |
| 机器人本机视频流是否已通                 | 是/否，给出本机可打开 URL                |
| 实际 ROS2 图像 topic                     | 例如 `/camera/color/image_raw`           |
| 8090 服务是否开机自启                    | 是/否                                    |
| 是否同意先用 Tailscale                   | 是/否                                    |
| 机器人 Tailscale IP                      | `100.x.x.x`                              |
| ECS 是否能通过 Tailscale 访问机器人 8090 | 是/否，给出 curl 或浏览器测试结果        |
| ECS 侧准备用什么转 HLS                   | ffmpeg / nginx-rtmp / SRS / 现有媒体服务 |
| 最终给后端的播放 URL 格式                | HLS `.m3u8` 优先                         |
| 是否需要小程序支持录像/拍照              | 如果要，建议复用公仔的 record/photo 模型 |

## 9. 现在你可以先回复小车同事的信息

可以先按下面口径回复：

```text
我们当前小程序云后端域名是 https://stemm.hnh666.com。
ECS 公网 IP 之前使用的是 8.148.249.36，我这边再最终确认一下。
ECS 系统版本我需要查一下，目标按 Ubuntu 22.04 兼容处理。
组网方案建议先用 Tailscale，机器人和 ECS 加入同一个 tailnet，ECS 通过机器人 Tailscale IP 拉 8090 视频流。
小程序不直接访问机器人 8090，小程序只调用云后端 start/keepalive/stop，播放后端返回的 HLS URL。
机器人本机需要先确认 http://本机IP:8090/stream_viewer?topic=/camera/color/image_raw 能打开。
如果实际 topic 不是 /camera/color/image_raw，请你们给最终 topic。
```

## 10. 最佳落地顺序

```text
1. 小车端确认 ROS2 camera topic
2. 小车端确认 8090 本机视频页面可打开
3. 机器人和 ECS 安装并登录 Tailscale
4. ECS 通过机器人 Tailscale IP 拉通 8090
5. ECS 媒体服务把 8090 流转成 HLS
6. 后端新增小车 stream start/keepalive/stop 接口
7. 小程序复用公仔监控播放生命周期接入小车 playUrl
8. 再扩展录像、拍照、截图、异常识别
```
