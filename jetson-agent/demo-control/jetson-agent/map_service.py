import os
import threading
from datetime import datetime
from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi import HTTPException
from fastapi.responses import FileResponse
from fastapi.responses import Response
from pydantic import BaseModel

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from std_srvs.srv import Trigger
except ImportError:  # pragma: no cover - allows local editing without ROS2 installed
    rclpy = None
    Node = object
    OccupancyGrid = None
    Trigger = None


DEVICE_ID = os.getenv('ROBOT_DEVICE_ID', 'robot-001')
MAP_FILE_PATH = Path(
    os.getenv(
        'MAP_FILE_PATH',
        '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm',
    )
)
MAP_SERVICE_NAME = os.getenv('MAP_SERVICE_PATH', '/stemm/save_map')
LIVE_MAP_TOPIC = os.getenv('LIVE_MAP_TOPIC', '/map')
MAP_FORMAT = 'pgm'
MAP_WAIT_TIMEOUT = float(os.getenv('MAP_WAIT_TIMEOUT', '30'))
MAP_FILE_WAIT_TIMEOUT = float(os.getenv('MAP_FILE_WAIT_TIMEOUT', '5'))
MAP_METADATA_PATH = Path(os.getenv('MAP_METADATA_PATH', str(MAP_FILE_PATH.with_suffix('.yaml'))))

app = FastAPI(title='map-service', version='1.0.0')


class SaveMapPayload(BaseModel):
    deviceId: str = DEVICE_ID


def _read_pgm_size(path: Path) -> tuple[Optional[int], Optional[int]]:
    if not path.is_file():
        return None, None

    tokens = []
    with path.open('rb') as file_obj:
        for raw_line in file_obj:
            line = raw_line.split(b'#', 1)[0].strip()
            if not line:
                continue
            tokens.extend(line.split())
            if len(tokens) >= 3:
                break

    if len(tokens) < 3 or tokens[0] not in (b'P2', b'P5'):
        return None, None

    return int(tokens[1]), int(tokens[2])


def _parse_yaml_scalar(text: str, key: str) -> Optional[float]:
    for line in text.splitlines():
        stripped = line.split('#', 1)[0].strip()
        if not stripped.startswith(f'{key}:'):
            continue
        value = stripped.split(':', 1)[1].strip().strip('"\'')
        try:
            return float(value)
        except ValueError:
            return None
    return None


def _parse_yaml_origin(text: str) -> tuple[Optional[float], Optional[float], float]:
    for line in text.splitlines():
        stripped = line.split('#', 1)[0].strip()
        if not stripped.startswith('origin:'):
            continue
        raw_value = stripped.split(':', 1)[1].strip().strip('[]')
        parts = [item.strip() for item in raw_value.split(',') if item.strip()]
        if len(parts) < 2:
            return None, None, 0.0
        try:
            origin_x = float(parts[0])
            origin_y = float(parts[1])
            origin_yaw = float(parts[2]) if len(parts) > 2 else 0.0
            return origin_x, origin_y, origin_yaw
        except ValueError:
            return None, None, 0.0
    return None, None, 0.0


def get_map_metadata() -> dict:
    map_ready = MAP_FILE_PATH.is_file()
    map_width, map_height = _read_pgm_size(MAP_FILE_PATH)
    resolution = None
    origin_x = None
    origin_y = None
    origin_yaw = 0.0

    if MAP_METADATA_PATH.is_file():
        yaml_text = MAP_METADATA_PATH.read_text(encoding='utf-8', errors='ignore')
        resolution = _parse_yaml_scalar(yaml_text, 'resolution')
        origin_x, origin_y, origin_yaw = _parse_yaml_origin(yaml_text)

    map_updated_at = ''
    if map_ready:
        map_updated_at = datetime.utcfromtimestamp(MAP_FILE_PATH.stat().st_mtime).isoformat() + 'Z'

    return {
        'mapReady': map_ready,
        'mapFormat': MAP_FORMAT,
        'mapPath': str(MAP_FILE_PATH),
        'mapMetadataPath': str(MAP_METADATA_PATH),
        'mapUpdatedAt': map_updated_at,
        'mapWidth': map_width,
        'mapHeight': map_height,
        'resolution': resolution,
        'originX': origin_x,
        'originY': origin_y,
        'originYaw': origin_yaw,
    }


class MapSaver:
    def __init__(self) -> None:
        if rclpy is None or Trigger is None or OccupancyGrid is None:
            raise RuntimeError('ROS2 Python packages are not available. Please source ROS2 first.')

        rclpy.init(args=None)
        self.node = Node('robot_map_service')
        self.client = self.node.create_client(Trigger, MAP_SERVICE_NAME)
        self.live_map_sub = self.node.create_subscription(
            OccupancyGrid,
            LIVE_MAP_TOPIC,
            self._on_live_map,
            1,
        )
        self.lock = threading.Lock()
        self.spin_stop = threading.Event()
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.live_map_lock = threading.Lock()
        self.live_map_pgm: Optional[bytes] = None
        self.live_map_updated_at = ''
        self.live_map_width: Optional[int] = None
        self.live_map_height: Optional[int] = None
        self.live_map_resolution: Optional[float] = None
        self.live_map_origin_x: Optional[float] = None
        self.live_map_origin_y: Optional[float] = None
        self.live_map_origin_yaw = 0.0
        self.last_attempt_at = ''
        self.last_success_at = ''
        self.last_error = ''
        self.last_command_status = 'idle'
        self.save_in_progress = False
        self.ros_ready = self.client.wait_for_service(timeout_sec=5.0)
        self.spin_thread.start()

    def _now(self) -> str:
        return datetime.utcnow().isoformat() + 'Z'

    def _spin_loop(self) -> None:
        while not self.spin_stop.is_set():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _on_live_map(self, msg: OccupancyGrid) -> None:
        pgm_bytes = self._occupancy_grid_to_pgm(msg)
        origin = msg.info.origin
        with self.live_map_lock:
            self.live_map_pgm = pgm_bytes
            self.live_map_updated_at = self._now()
            self.live_map_width = int(msg.info.width)
            self.live_map_height = int(msg.info.height)
            self.live_map_resolution = float(msg.info.resolution)
            self.live_map_origin_x = float(origin.position.x)
            self.live_map_origin_y = float(origin.position.y)
            self.live_map_origin_yaw = 0.0

    def _occupancy_grid_to_pgm(self, msg: OccupancyGrid) -> bytes:
        width = int(msg.info.width)
        height = int(msg.info.height)
        header = f'P5\n# source: {LIVE_MAP_TOPIC}\n{width} {height}\n255\n'.encode('ascii')
        pixels = bytearray(width * height)

        for row in range(height):
            source_row = height - 1 - row
            for col in range(width):
                source_index = source_row * width + col
                target_index = row * width + col
                value = int(msg.data[source_index]) if source_index < len(msg.data) else -1
                if value < 0:
                    pixels[target_index] = 205
                elif value >= 65:
                    pixels[target_index] = 0
                else:
                    pixels[target_index] = 254

        return header + bytes(pixels)

    def live_snapshot(self) -> dict:
        with self.live_map_lock:
            return {
                'ok': True,
                'deviceId': DEVICE_ID,
                'liveMapReady': self.live_map_pgm is not None,
                'liveMapTopic': LIVE_MAP_TOPIC,
                'mapFormat': MAP_FORMAT,
                'mapUpdatedAt': self.live_map_updated_at,
                'mapWidth': self.live_map_width,
                'mapHeight': self.live_map_height,
                'resolution': self.live_map_resolution,
                'originX': self.live_map_origin_x,
                'originY': self.live_map_origin_y,
                'originYaw': self.live_map_origin_yaw,
                'rosReady': True,
            }

    def live_map_response(self) -> Response:
        with self.live_map_lock:
            if self.live_map_pgm is None:
                raise HTTPException(status_code=404, detail='Live map not ready')
            pgm_bytes = self.live_map_pgm
            updated_at = self.live_map_updated_at

        response = Response(content=pgm_bytes, media_type='image/x-portable-graymap')
        response.headers['Cache-Control'] = 'no-store'
        response.headers['Content-Disposition'] = 'inline; filename="live_map.pgm"'
        response.headers['X-Map-Format'] = MAP_FORMAT
        response.headers['X-Map-Source'] = LIVE_MAP_TOPIC
        response.headers['X-Map-Updated-At'] = updated_at
        return response

    def snapshot(self) -> dict:
        metadata = get_map_metadata()
        if metadata['mapReady']:
            map_updated_at = metadata['mapUpdatedAt']
            if not self.last_success_at or self.last_success_at < map_updated_at:
                self.last_success_at = map_updated_at

        with self.lock:
            return {
                'ok': True,
                'deviceId': DEVICE_ID,
                **metadata,
                'lastAttemptAt': self.last_attempt_at,
                'lastSuccessAt': self.last_success_at,
                'lastError': self.last_error,
                'lastCommandStatus': self.last_command_status,
                'saveInProgress': self.save_in_progress,
                'rosReady': self.ros_ready,
            }

    def save_map(self) -> dict:
        with self.lock:
            if self.save_in_progress:
                already_in_progress = True
            else:
                already_in_progress = False
                self.save_in_progress = True
                self.last_attempt_at = self._now()
                self.last_error = ''
                self.last_command_status = 'pending'

        if already_in_progress:
            snapshot = self.snapshot()
            snapshot['ok'] = True
            snapshot['message'] = 'Save map already in progress'
            snapshot['lastCommandStatus'] = 'pending'
            snapshot['saveInProgress'] = True
            return snapshot

        try:
            if not self.client.service_is_ready():
                self.ros_ready = self.client.wait_for_service(timeout_sec=5.0)
            else:
                self.ros_ready = True

            if not self.ros_ready:
                raise RuntimeError(f'Map service not available: {MAP_SERVICE_NAME}')

            request = Trigger.Request()
            future = self.client.call_async(request)
            deadline = datetime.utcnow().timestamp() + MAP_WAIT_TIMEOUT
            while not future.done() and datetime.utcnow().timestamp() < deadline:
                self.spin_stop.wait(0.05)
            if not future.done():
                raise RuntimeError('Save map service call timed out')

            response = future.result()
            if response is None:
                raise RuntimeError('Save map service returned no response')
            if not response.success:
                raise RuntimeError(response.message or 'Save map service returned success=false')

            if MAP_FILE_WAIT_TIMEOUT > 0:
                deadline = datetime.utcnow().timestamp() + MAP_FILE_WAIT_TIMEOUT
                while datetime.utcnow().timestamp() < deadline:
                    if MAP_FILE_PATH.is_file():
                        break
                    self.spin_stop.wait(0.05)

            with self.lock:
                self.last_command_status = 'ok'
                self.last_error = ''

            snapshot = self.snapshot()
            if not snapshot['mapReady']:
                snapshot['lastCommandStatus'] = 'ok'
            return snapshot
        except Exception as exc:
            with self.lock:
                self.last_command_status = 'failed'
                self.last_error = str(exc)
            raise
        finally:
            with self.lock:
                self.save_in_progress = False

    def shutdown(self) -> None:
        self.spin_stop.set()
        self.spin_thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.shutdown()


map_saver: Optional[MapSaver] = None
startup_error: Optional[str] = None


@app.on_event('startup')
def on_startup() -> None:
    global map_saver, startup_error
    try:
        map_saver = MapSaver()
    except Exception as exc:  # pragma: no cover - depends on target environment
        startup_error = str(exc)
        map_saver = None


@app.on_event('shutdown')
def on_shutdown() -> None:
    if map_saver is not None:
        map_saver.shutdown()


@app.get('/api/status')
def get_status() -> dict:
    if map_saver is None:
        return {
            'ok': False,
            'deviceId': DEVICE_ID,
            'message': startup_error or 'map-service unavailable',
        }
    snapshot = map_saver.snapshot()
    snapshot['message'] = 'map-service ready'
    return snapshot


@app.get('/api/map/status')
def get_map_status() -> dict:
    if map_saver is None:
        return {
            'ok': False,
            'deviceId': DEVICE_ID,
            **get_map_metadata(),
            'lastAttemptAt': '',
            'lastSuccessAt': '',
            'lastError': startup_error or 'map-service unavailable',
            'lastCommandStatus': 'failed',
            'saveInProgress': False,
            'rosReady': False,
        }
    return map_saver.snapshot()


@app.get('/api/map/metadata')
def get_metadata() -> dict:
    return {
        'ok': True,
        'deviceId': DEVICE_ID,
        **get_map_metadata(),
    }


@app.get('/api/map/live/status')
def get_live_map_status() -> dict:
    if map_saver is None:
        return {
            'ok': False,
            'deviceId': DEVICE_ID,
            'liveMapReady': False,
            'liveMapTopic': LIVE_MAP_TOPIC,
            'mapFormat': MAP_FORMAT,
            'mapUpdatedAt': '',
            'lastError': startup_error or 'map-service unavailable',
            'rosReady': False,
        }
    return map_saver.live_snapshot()


@app.post('/api/map/save')
def save_map(payload: SaveMapPayload) -> dict:
    if payload.deviceId != DEVICE_ID:
        return {
            **get_map_status(),
            'ok': False,
            'deviceId': DEVICE_ID,
            'message': f'Unexpected deviceId: {payload.deviceId}',
        }
    if map_saver is None:
        return {
            'deviceId': DEVICE_ID,
            'ok': False,
            'mapReady': MAP_FILE_PATH.is_file(),
            'mapFormat': MAP_FORMAT,
            'mapPath': str(MAP_FILE_PATH),
            'lastAttemptAt': '',
            'lastSuccessAt': '',
            'lastError': startup_error or 'map-service unavailable',
            'lastCommandStatus': 'failed',
            'saveInProgress': False,
            'rosReady': False,
            'message': startup_error or 'map-service unavailable',
        }

    try:
        snapshot = map_saver.save_map()
    except RuntimeError as exc:
        snapshot = map_saver.snapshot()
        return {
            **snapshot,
            'ok': False,
            'deviceId': DEVICE_ID,
            'message': str(exc),
        }
    except Exception as exc:
        snapshot = map_saver.snapshot()
        return {
            **snapshot,
            'ok': False,
            'deviceId': DEVICE_ID,
            'message': str(exc),
        }

    snapshot['message'] = 'Save map completed'
    return snapshot


@app.get('/api/map/latest.pgm')
def get_latest_map() -> FileResponse:
    if not MAP_FILE_PATH.is_file():
        raise HTTPException(status_code=404, detail='Map not ready')

    response = FileResponse(
        path=str(MAP_FILE_PATH),
        media_type='image/x-portable-graymap',
        filename='stemm_map.pgm',
    )
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Map-Format'] = MAP_FORMAT
    response.headers['X-Map-Updated-At'] = datetime.utcfromtimestamp(MAP_FILE_PATH.stat().st_mtime).isoformat() + 'Z'
    return response


@app.get('/api/map/live.pgm')
def get_live_map() -> Response:
    if map_saver is None:
        raise HTTPException(status_code=503, detail=startup_error or 'map-service unavailable')
    return map_saver.live_map_response()
