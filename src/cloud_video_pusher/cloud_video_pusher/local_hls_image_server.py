import json
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import urlparse

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool


class LocalMjpegHandler(BaseHTTPRequestHandler):
    """Serve the latest ROS image as JPEG snapshots or an MJPEG stream."""

    server: 'LocalMjpegHttpServer'

    def end_headers(self) -> None:
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        super().end_headers()

    def do_OPTIONS(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler.
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler.
        node = self.server.node
        path = urlparse(self.path).path.rstrip('/') or '/'

        if path in node.snapshot_paths:
            self._send_snapshot(node)
            return
        if path in node.stream_paths:
            self._send_mjpeg_stream(node)
            return
        if path == '/stream_session':
            self._send_json(node._build_session_payload(node.stream_active))
            return
        if path == '/':
            self._send_json({
                'snapshotUrl': node.snapshot_url,
                'mjpegUrl': node.mjpeg_url,
                'streamSessionUrl': node.stream_session_url,
            })
            return

        self.send_error(HTTPStatus.NOT_FOUND, 'unknown local MJPEG endpoint')

    def _send_snapshot(self, node: 'LocalHlsImageServer') -> None:
        jpeg, stamp = node.get_latest_jpeg()
        if jpeg is None:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, 'waiting for camera frame')
            return

        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('X-Timestamp', f'{stamp:.06f}')
        self.end_headers()
        self.wfile.write(jpeg)

    def _send_mjpeg_stream(self, node: 'LocalHlsImageServer') -> None:
        if not node.stream_active:
            self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, 'local MJPEG stream is stopped')
            return

        boundary = 'frame'
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={boundary}')
        self.send_header('Connection', 'close')
        self.end_headers()

        last_stamp = 0.0
        while node.stream_active and rclpy.ok():
            jpeg, stamp = node.get_latest_jpeg()
            if jpeg is None or stamp <= last_stamp:
                time.sleep(0.02)
                continue

            try:
                self.wfile.write(f'--{boundary}\r\n'.encode())
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n'.encode())
                self.wfile.write(f'X-Timestamp: {stamp:.06f}\r\n\r\n'.encode())
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

            last_stamp = stamp

    def _send_json(self, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args) -> None:  # noqa: A002 - stdlib method signature.
        return


class LocalMjpegHttpServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], node: 'LocalHlsImageServer') -> None:
        super().__init__(server_address, LocalMjpegHandler)
        self.node = node


class LocalHlsImageServer(Node):
    """Serve a ROS Image topic locally as MJPEG and JPEG polling endpoints."""

    def __init__(self) -> None:
        super().__init__('local_hls_image_server')

        self.image_topic = self.declare_parameter('image_topic', '/camera/color/image_raw').value
        self.bind_address = self.declare_parameter('bind_address', '0.0.0.0').value
        self.public_host = self.declare_parameter('public_host', '').value
        self.http_port = int(self.declare_parameter('http_port', 18080).value)
        self.stream_app = self.declare_parameter('stream_app', 'live').value.strip('/') or 'live'
        self.stream_name = self.declare_parameter('stream_name', 'robot001').value.strip('/') or 'robot001'
        self.hub_id = self.declare_parameter('hub_id', '910007').value
        self.camera_device_id = self.declare_parameter('camera_device_id', 'robot-camera-1').value
        self.width = int(self.declare_parameter('width', 640).value)
        self.height = int(self.declare_parameter('height', 480).value)
        self.fps = float(self.declare_parameter('fps', 8.0).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 75).value)
        self.poll_interval_ms = int(self.declare_parameter('poll_interval_ms', 150).value)
        self.auto_start = bool(self.declare_parameter('auto_start', True).value)

        self.bridge = CvBridge()
        self._jpeg_lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._latest_jpeg_stamp = 0.0
        self._last_encode_time = 0.0
        self._httpd: Optional[LocalMjpegHttpServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._last_status = ''
        self.stream_active = self.auto_start

        self.snapshot_path = f'/{self.stream_app}/{self.stream_name}.jpg'
        self.mjpeg_path = f'/{self.stream_app}/{self.stream_name}.mjpg'
        self.snapshot_paths = {self.snapshot_path, '/frame.jpg', '/snapshot.jpg'}
        self.stream_paths = {self.mjpeg_path, '/stream.mjpg'}
        self.snapshot_url = self._build_url(self.snapshot_path)
        self.mjpeg_url = self._build_url(self.mjpeg_path)
        self.stream_session_url = self._build_url('/stream_session')
        self.play_url = self.snapshot_url

        self.status_pub = self.create_publisher(String, 'stream_status', 10)
        self.session_pub = self.create_publisher(String, 'stream_session', 10)
        self.control_srv = self.create_service(SetBool, 'set_streaming', self._handle_set_streaming)
        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, qos_profile_sensor_data)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog)

        self._start_http_server()
        self._publish_session(self.stream_active)
        self._publish_status('streaming' if self.stream_active else 'idle')
        self.get_logger().info(
            f'Local MJPEG server ready: topic={self.image_topic}, snapshotUrl={self.snapshot_url}, '
            f'mjpegUrl={self.mjpeg_url}, size={self.width}x{self.height}, fps={self.fps:g}, '
            f'jpeg_quality={self.jpeg_quality}'
        )

    def destroy_node(self) -> bool:
        self._stop_http_server()
        return super().destroy_node()

    def _handle_set_streaming(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self.stream_active = bool(request.data)
        if self.stream_active:
            self._publish_status('streaming')
            response.message = 'local MJPEG streaming started'
        else:
            self._publish_status('idle')
            response.message = 'local MJPEG streaming stopped'
        self._publish_session(self.stream_active)
        response.success = True
        return response

    def _image_callback(self, msg: Image) -> None:
        if not self.stream_active:
            return

        now = time.monotonic()
        min_interval = 1.0 / max(self.fps, 1.0)
        if now - self._last_encode_time < min_interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - keep ROS callbacks alive on bad frames.
            self.get_logger().warn(f'Failed to convert image frame: {exc}')
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)

        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            self.get_logger().warn('Failed to encode JPEG frame')
            return

        with self._jpeg_lock:
            self._latest_jpeg = encoded.tobytes()
            self._latest_jpeg_stamp = time.time()
            self._last_encode_time = now

    def get_latest_jpeg(self) -> tuple[Optional[bytes], float]:
        with self._jpeg_lock:
            return self._latest_jpeg, self._latest_jpeg_stamp

    def _start_http_server(self) -> None:
        self._httpd = LocalMjpegHttpServer((self.bind_address, self.http_port), self)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, name='local_mjpeg_http', daemon=True)
        self._http_thread.start()
        self.get_logger().info(f'HTTP server listening on {self.bind_address}:{self.http_port}')

    def _stop_http_server(self) -> None:
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        if self._http_thread:
            self._http_thread.join(timeout=2.0)
            self._http_thread = None

    def _build_url(self, path: str) -> str:
        host = self.public_host.strip() or self._detect_local_ip()
        return f'http://{host}:{self.http_port}{path}'

    def _detect_local_ip(self) -> str:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect(('8.8.8.8', 80))
                return sock.getsockname()[0]
        except OSError:
            try:
                return socket.gethostbyname(socket.gethostname())
            except OSError:
                return '127.0.0.1'

    def _build_session_payload(self, active: bool) -> dict:
        return {
            'hubId': self.hub_id,
            'cameraDeviceId': self.camera_device_id,
            'streamSessionId': self.stream_name,
            'rtmpUrl': '',
            'hlsUrl': '',
            'mjpegUrl': self.mjpeg_url,
            'snapshotUrl': self.snapshot_url,
            'playUrl': self.snapshot_url,
            'playType': 'mjpeg',
            'pollIntervalMs': self.poll_interval_ms,
            'streamActive': active,
            'sessionTtlSeconds': 60,
            'uploadIntervalMs': 15000,
            'localOnly': True,
        }

    def _publish_session(self, active: bool) -> None:
        msg = String()
        msg.data = json.dumps(self._build_session_payload(active), ensure_ascii=False)
        self.session_pub.publish(msg)

    def _watchdog(self) -> None:
        if not self.stream_active:
            self._publish_status('idle')
            return

        with self._jpeg_lock:
            has_frame = self._latest_jpeg is not None
            age = time.time() - self._latest_jpeg_stamp if has_frame else None

        if not has_frame:
            self._publish_status('waiting_for_frame')
        elif age is not None and age > 5.0:
            self._publish_status('warning:no_recent_frame')
        else:
            self._publish_status('streaming')
        self._publish_session(self.stream_active)

    def _publish_status(self, status: str) -> None:
        if status != self._last_status:
            self.get_logger().info(f'stream_status={status}')
            self._last_status = status
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalHlsImageServer()
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
