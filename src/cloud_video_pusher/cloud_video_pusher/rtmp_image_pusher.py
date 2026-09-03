import json
import shutil
import subprocess
import threading
import time
from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool


class RtmpImagePusher(Node):
    """Push a ROS Image topic to an RTMP endpoint using ffmpeg."""

    def __init__(self) -> None:
        super().__init__('rtmp_image_pusher')

        self.image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
        self.rtmp_url = self.declare_parameter('rtmp_url', '').value
        self.media_host = self.declare_parameter('media_host', '').value
        self.rtmp_base_url = self.declare_parameter('rtmp_base_url', '').value
        self.hls_base_url = self.declare_parameter('hls_base_url', '').value
        self.stream_app = self.declare_parameter('stream_app', 'live').value
        self.stream_name = self.declare_parameter('stream_name', 'robot001').value
        self.hub_id = self.declare_parameter('hub_id', '910007').value
        self.camera_device_id = self.declare_parameter('camera_device_id', 'robot-camera-1').value
        self.session_ttl_seconds = int(self.declare_parameter('session_ttl_seconds', 60).value)
        self.upload_interval_ms = int(self.declare_parameter('upload_interval_ms', 15000).value)
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = float(self.declare_parameter('fps', 15.0).value)
        self.bitrate = self.declare_parameter('bitrate', '800k').value
        self.gop = int(self.declare_parameter('gop', 30).value)
        self.ffmpeg_path = self.declare_parameter('ffmpeg_path', 'ffmpeg').value
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)
        self.restart_on_failure = bool(self.declare_parameter('restart_on_failure', True).value)
        self.restart_interval_sec = float(self.declare_parameter('restart_interval_sec', 3.0).value)

        self.bridge = CvBridge()
        self._latest_frame = None
        self._latest_frame_stamp = 0.0
        self._frame_lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._want_streaming = False
        self._last_restart_attempt = 0.0
        self._last_status = ''
        self.stream_session_id = self.stream_name
        self.hls_url = ''
        self.play_url = ''

        self.status_pub = self.create_publisher(String, 'stream_status', 10)
        self.session_pub = self.create_publisher(String, 'stream_session', 10)
        self.control_srv = self.create_service(SetBool, 'set_streaming', self._handle_set_streaming)
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        period = 1.0 / max(self.fps, 1.0)
        self.push_timer = self.create_timer(period, self._push_latest_frame)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog)

        self.get_logger().info(
            f'RTMP pusher ready: topic={self.image_topic}, size={self.width}x{self.height}, '
            f'fps={self.fps:g}, bitrate={self.bitrate}'
        )
        self._prepare_session_urls()
        self._publish_session(False)

        if self.auto_start:
            self._want_streaming = True
            self._start_ffmpeg()
        else:
            self._publish_status('idle')

    def destroy_node(self) -> bool:
        self._stop_ffmpeg(publish_status=rclpy.ok())
        return super().destroy_node()

    def _handle_set_streaming(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        if request.data:
            self._want_streaming = True
            ok = self._start_ffmpeg()
            response.success = ok
            response.message = 'streaming requested' if ok else 'failed to start ffmpeg'
        else:
            self._want_streaming = False
            self._stop_ffmpeg()
            response.success = True
            response.message = 'streaming stopped'
        return response

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - keep ROS callbacks alive on bad frames.
            self.get_logger().warn(f'Failed to convert image frame: {exc}')
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        with self._frame_lock:
            self._latest_frame = frame
            self._latest_frame_stamp = time.monotonic()

    def _build_ffmpeg_command(self) -> list[str]:
        return [
            self.ffmpeg_path,
            '-hide_banner',
            '-loglevel', 'warning',
            '-f', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.width}x{self.height}',
            '-r', f'{self.fps:g}',
            '-i', '-',
            '-an',
            '-c:v', 'libx264',
            '-preset', 'veryfast',
            '-tune', 'zerolatency',
            '-b:v', self.bitrate,
            '-maxrate', self.bitrate,
            '-bufsize', self._double_bitrate(self.bitrate),
            '-g', str(self.gop),
            '-pix_fmt', 'yuv420p',
            '-f', 'flv',
            self.rtmp_url,
        ]

    @staticmethod
    def _double_bitrate(value: str) -> str:
        if value.endswith(('k', 'K')) and value[:-1].isdigit():
            return f'{int(value[:-1]) * 2}k'
        if value.endswith(('m', 'M')) and value[:-1].isdigit():
            return f'{int(value[:-1]) * 2}m'
        return value

    def _join_url(self, base: str, suffix: str) -> str:
        return f'{base.rstrip("/")}/{suffix.lstrip("/")}'

    def _prepare_session_urls(self) -> None:
        stream_path = f'{self.stream_app.strip("/")}/{self.stream_name.strip("/")}'

        if not self.rtmp_url:
            if self.rtmp_base_url:
                self.rtmp_url = self._join_url(self.rtmp_base_url, self.stream_name)
            elif self.media_host:
                self.rtmp_url = f'rtmp://{self.media_host.rstrip("/")}/{stream_path}'

        if self.hls_base_url:
            self.hls_url = self._join_url(self.hls_base_url, f'{self.stream_name}.m3u8')
        elif self.media_host:
            # SRS default HTTP server exposes HLS on port 8080; production should use HTTPS reverse proxy.
            self.hls_url = f'http://{self.media_host.rstrip("/")}:8080/{stream_path}.m3u8'

        self.play_url = self.hls_url

    def _build_session_payload(self, active: bool) -> dict:
        return {
            'hubId': self.hub_id,
            'cameraDeviceId': self.camera_device_id,
            'streamSessionId': self.stream_session_id,
            'rtmpUrl': self.rtmp_url,
            'hlsUrl': self.hls_url,
            'playUrl': self.play_url,
            'playType': 'hls',
            'streamActive': active,
            'sessionTtlSeconds': self.session_ttl_seconds,
            'uploadIntervalMs': self.upload_interval_ms,
        }

    def _publish_session(self, active: bool) -> None:
        msg = String()
        msg.data = json.dumps(self._build_session_payload(active), ensure_ascii=False)
        self.session_pub.publish(msg)

    def _start_ffmpeg(self) -> bool:
        with self._process_lock:
            self._prepare_session_urls()
            if self._process and self._process.poll() is None:
                self._publish_status('streaming')
                self._publish_session(True)
                return True

            if not self.rtmp_url:
                self.get_logger().error('Parameter rtmp_url is empty; pass rtmp_url:=rtmp://... to start pushing.')
                self._publish_status('error:no_rtmp_url')
                return False

            if shutil.which(self.ffmpeg_path) is None:
                self.get_logger().error(f'ffmpeg not found: {self.ffmpeg_path}. Install ffmpeg or set ffmpeg_path.')
                self._publish_status('error:no_ffmpeg')
                return False

            self._last_restart_attempt = time.monotonic()
            cmd = self._build_ffmpeg_command()
            self.get_logger().info('Starting ffmpeg RTMP push process.')
            try:
                self._process = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    bufsize=0,
                )
            except Exception as exc:  # noqa: BLE001 - surface process launch failures to ROS logs.
                self.get_logger().error(f'Failed to start ffmpeg: {exc}')
                self._process = None
                self._publish_status('error:start_failed')
                return False

            self._publish_status('streaming')
            self._publish_session(True)
            return True

    def _stop_ffmpeg(self, publish_status: bool = True) -> None:
        with self._process_lock:
            proc = self._process
            self._process = None

        if not proc:
            if publish_status:
                self._publish_status('idle')
            return

        if publish_status:
            self.get_logger().info('Stopping ffmpeg RTMP push process.')
        try:
            if proc.stdin:
                proc.stdin.close()
            proc.terminate()
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3.0)
        except Exception as exc:  # noqa: BLE001 - cleanup should never crash shutdown.
            self.get_logger().warn(f'Error while stopping ffmpeg: {exc}')
        if publish_status:
            self._publish_status('idle')
            self._publish_session(False)

    def _push_latest_frame(self) -> None:
        with self._process_lock:
            proc = self._process

        if not self._want_streaming or not proc or proc.poll() is not None or not proc.stdin:
            return

        with self._frame_lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        try:
            proc.stdin.write(frame.tobytes())
        except (BrokenPipeError, OSError) as exc:
            self.get_logger().warn(f'ffmpeg pipe closed: {exc}')
            self._mark_process_dead('error:pipe_closed')

    def _watchdog(self) -> None:
        with self._process_lock:
            proc = self._process

        if not self._want_streaming:
            return

        if proc and proc.poll() is None:
            with self._frame_lock:
                age = time.monotonic() - self._latest_frame_stamp if self._latest_frame is not None else None
            if age is None:
                self._publish_status('waiting_for_frame')
            elif age > 5.0:
                self._publish_status('warning:no_recent_frame')
            else:
                self._publish_status('streaming')
            return

        if proc and proc.poll() is not None:
            self._log_ffmpeg_stderr(proc)
            self._mark_process_dead('error:ffmpeg_exited')

        if self.restart_on_failure and time.monotonic() - self._last_restart_attempt >= self.restart_interval_sec:
            self.get_logger().warn('Restarting ffmpeg after failure.')
            self._start_ffmpeg()

    def _mark_process_dead(self, status: str) -> None:
        with self._process_lock:
            proc = self._process
            self._process = None
        if proc:
            self._log_ffmpeg_stderr(proc)
            try:
                if proc.stdin:
                    proc.stdin.close()
            except Exception:
                pass
        self._publish_status(status)

    def _log_ffmpeg_stderr(self, proc: subprocess.Popen) -> None:
        if not proc.stderr:
            return
        try:
            stderr = proc.stderr.read()
        except Exception:
            return
        if stderr:
            text = stderr.decode(errors='replace').strip()
            if text:
                self.get_logger().warn(f'ffmpeg stderr: {text[-1000:]}')

    def _publish_status(self, status: str) -> None:
        if status != self._last_status:
            self.get_logger().info(f'stream_status={status}')
            self._last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtmpImagePusher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()