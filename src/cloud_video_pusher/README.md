# cloud_video_pusher

ROS2 demo package for pushing a local image topic to a self-hosted media service without modifying the existing camera or `web_video_server` nodes.

Default flow:

```text
/camera/color/image_raw -> rtmp_image_pusher -> ffmpeg -> RTMP media service -> HLS/m3u8 playUrl for mini program
```

The robot only uses the RTMP push URL. The mini program should not play RTMP directly and should not depend on FLV as the main playback path. The backend should return an HLS address for the normal `video` component.

Address roles:

| Role | Example | Used by |
| --- | --- | --- |
| Push URL | `rtmp://example.com/live/robot001` | Robot pusher node |
| Playback URL | `https://example.com/live/robot001.m3u8` | Mini program `video` component |

Recommended start response:

```json
{
  "streamSessionId": "xxx",
  "hlsUrl": "https://example.com/live/robot001.m3u8",
  "playUrl": "https://example.com/live/robot001.m3u8",
  "playType": "hls",
  "streamActive": true
}
```

The frontend should use `hlsUrl || playUrl`.

## Local-only HLS flow

Use this when the robot and mini program are in the same LAN and the video must not go to cloud. This starts a small HTTP server on the robot and writes live HLS files locally.

```text
/camera/color/image_raw -> local_hls_image_server -> ffmpeg HLS files -> robot HTTP server -> mini program video
```

Start it on the robot:

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py
```

Default playback URL format:

```text
http://ROBOT_LAN_IP:18080/live/robot001.m3u8
```

If the auto-detected IP is not the Wi-Fi/LAN IP that the mini program can reach, pass it explicitly:

```bash
ros2 launch cloud_video_pusher local_hls_server.launch.py public_host:=192.168.2.123
```

Watch the local session payload:

```bash
ros2 topic echo /local_hls_image_server/stream_session
```

The mini program should still use the normal `video` component and `hlsUrl || playUrl`; this mode sets `rtmpUrl` to an empty string and `localOnly` to `true`.

## Self-hosted SRS flow

Use this when Alibaba Cloud Live is unavailable. The ECS only runs a media service; the robot still pushes RTMP and the mini program still plays HLS.

```text
Robot -> rtmp://ECS/live/robot001 -> SRS -> https://video.example.com/live/robot001.m3u8 -> mini program video
```

Open these ECS security group ports first:

| Port | Purpose |
| --- | --- |
| `1935` | RTMP push from robot |
| `8080` | SRS HTTP/HLS test port |
| `80` | HTTP reverse proxy / certificate issue |
| `443` | HTTPS HLS playback for mini program |

Start SRS on ECS:

```bash
sudo apt update
sudo apt install -y docker.io
sudo systemctl enable --now docker
sudo docker run -d --name srs --restart=always \
  -p 1935:1935 \
  -p 8080:8080 \
  ossrs/srs:5
```

For first test, the robot can use the ECS public IP directly:

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py \
  media_host:=YOUR_ECS_PUBLIC_IP \
  stream_name:=robot001
```

This generates:

```text
rtmpUrl = rtmp://YOUR_ECS_PUBLIC_IP/live/robot001
hlsUrl  = http://YOUR_ECS_PUBLIC_IP:8080/live/robot001.m3u8
```

For production, use a domain and HTTPS reverse proxy. Then pass explicit bases:

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py \
  rtmp_base_url:=rtmp://video.example.com/live \
  hls_base_url:=https://video.example.com/live \
  stream_name:=robot001
```

This generates:

```text
rtmpUrl = rtmp://video.example.com/live/robot001
hlsUrl  = https://video.example.com/live/robot001.m3u8
```

## Parameters

| Name | Default | Description |
| --- | --- | --- |
| `image_topic` | `/camera/color/image_raw` | ROS image topic to push |
| `rtmp_url` | empty | RTMP push URL, for example `rtmp://host/live/robot001` |
| `media_host` | empty | ECS host/IP; auto-generates RTMP and test HLS URLs |
| `rtmp_base_url` | empty | Production RTMP base, for example `rtmp://video.example.com/live` |
| `hls_base_url` | empty | Production HLS base, for example `https://video.example.com/live` |
| `stream_name` | `robot001` | Stream/session key |
| `width` | `640` | Output width |
| `height` | `480` | Output height |
| `fps` | `15.0` | Output frame rate |
| `bitrate` | `800k` | H.264 bitrate |
| `auto_start` | `true` | Start ffmpeg on node startup |

## Build

```bash
cd /mnt/wheeltec_ros2
colcon build --packages-select cloud_video_pusher
source install/setup.bash
```

## Run

Replace the RTMP URL with the robot push URL returned by the backend or media service. Do not pass the HLS playback URL here.

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py \
  rtmp_url:='rtmp://example.com/live/robot001' \
  image_topic:=/camera/color/image_raw
```

If you want the node to wait for a manual start:

```bash
ros2 launch cloud_video_pusher rtmp_pusher.launch.py \
  auto_start:=false \
  rtmp_url:='rtmp://example.com/live/robot001'
```

Then start or stop it with:

```bash
ros2 service call /rtmp_image_pusher/set_streaming std_srvs/srv/SetBool "{data: true}"
ros2 service call /rtmp_image_pusher/set_streaming std_srvs/srv/SetBool "{data: false}"
```

Watch status:

```bash
ros2 topic echo /rtmp_image_pusher/stream_status
```

Watch the toy-compatible session payload:

```bash
ros2 topic echo /rtmp_image_pusher/stream_session
```

Example payload:

```json
{
  "hubId": "910007",
  "cameraDeviceId": "robot-camera-1",
  "streamSessionId": "robot001",
  "rtmpUrl": "rtmp://video.example.com/live/robot001",
  "hlsUrl": "https://video.example.com/live/robot001.m3u8",
  "playUrl": "https://video.example.com/live/robot001.m3u8",
  "playType": "hls",
  "streamActive": true,
  "sessionTtlSeconds": 60,
  "uploadIntervalMs": 15000
}
```

## Notes

- This demo requires `ffmpeg` on the robot.
- If `rtmp_url` is empty, the node stays alive but reports `error:no_rtmp_url`.
- The first version only validates robot-side RTMP push. Backend `start/keepalive/stop` integration can be added after the RTMP-to-HLS path is verified.
- The `ffmpeg` output uses RTMP/FLV packaging internally because RTMP requires it; this is not the mini-program FLV playback plan.