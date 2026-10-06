#!/usr/bin/env python3
import argparse
import json
import os
import threading
import time
import urllib.error
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional

BASE_DIR = Path(__file__).resolve().parent
WEB_DIR = BASE_DIR / 'web'

DEFAULT_COMMAND_URL = 'http://127.0.0.1:3000/api/command'
DEFAULT_MAP_FILE = '/home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/map/stemm_map.pgm'
DEFAULT_DEVICE_ID = 'robot-001'
DEFAULT_INTERVAL = 5.0
DEFAULT_HOST = '0.0.0.0'
DEFAULT_PORT = 8091
DEFAULT_SERVICE_PATH = '/stemm/save_map'
DEFAULT_SERVICE_TYPE = 'std_srvs/srv/Trigger'


def build_save_map_payload(device_id: str) -> dict:
    return {
        'deviceId': device_id,
        'action': 'call_service',
        'servicePath': DEFAULT_SERVICE_PATH,
        'serviceType': DEFAULT_SERVICE_TYPE,
        'serviceArgs': {},
    }


class SharedState:
    def __init__(self, map_path: Path, command_url: str, interval: float, device_id: str) -> None:
        self.map_path = map_path
        self.command_url = command_url
        self.interval = interval
        self.device_id = device_id
        self.latest_map_bytes: Optional[bytes] = None
        self.latest_map_updated_at: Optional[str] = None
        self.last_attempt_at: Optional[str] = None
        self.last_success_at: Optional[str] = None
        self.last_error: str = ''
        self.last_command_status: str = 'never-called'
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def update_map(self, image_bytes: bytes, updated_at: str) -> None:
        with self.lock:
            self.latest_map_bytes = image_bytes
            self.latest_map_updated_at = updated_at
            self.last_success_at = updated_at
            self.last_error = ''
            self.last_command_status = 'ok'

    def update_attempt(self, attempted_at: str) -> None:
        with self.lock:
            self.last_attempt_at = attempted_at

    def update_error(self, message: str, attempted_at: str) -> None:
        with self.lock:
            self.last_attempt_at = attempted_at
            self.last_error = message
            self.last_command_status = 'error'

    def snapshot(self) -> dict:
        with self.lock:
            return {
                'mapReady': self.latest_map_bytes is not None,
                'mapUpdatedAt': self.latest_map_updated_at,
                'lastAttemptAt': self.last_attempt_at,
                'lastSuccessAt': self.last_success_at,
                'lastError': self.last_error,
                'lastCommandStatus': self.last_command_status,
                'mapPath': str(self.map_path),
                'commandUrl': self.command_url,
                'intervalSeconds': self.interval,
                'deviceId': self.device_id,
            }

    def latest_map(self) -> tuple[Optional[bytes], Optional[str]]:
        with self.lock:
            return self.latest_map_bytes, self.latest_map_updated_at


def read_pgm_as_png_bytes(file_path: Path) -> bytes:
    raw = file_path.read_bytes()
    position = 0

    def next_token() -> bytes:
        nonlocal position
        while position < len(raw):
            byte = raw[position:position + 1]
            if byte == b'#':
                while position < len(raw) and raw[position:position + 1] not in (b'\n', b'\r'):
                    position += 1
            elif byte.isspace():
                position += 1
            else:
                break

        start = position
        while position < len(raw) and not raw[position:position + 1].isspace():
            position += 1
        return raw[start:position]

    magic = next_token()
    if magic not in (b'P5', b'P2'):
        raise ValueError(f'Unsupported PGM format: {magic!r}')

    width = int(next_token())
    height = int(next_token())
    max_value = int(next_token())
    if max_value <= 0:
        raise ValueError('Invalid PGM max value')

    while position < len(raw) and raw[position:position + 1].isspace():
        position += 1

    if magic == b'P5':
        pixel_data = raw[position:]
        bytes_per_sample = 1 if max_value < 256 else 2
        expected_size = width * height * bytes_per_sample
        if len(pixel_data) < expected_size:
            raise ValueError('PGM pixel data is incomplete')

        if bytes_per_sample == 1:
            pixels = pixel_data[:expected_size]
        else:
            scaled = bytearray(width * height)
            for index in range(width * height):
                sample_offset = index * 2
                sample = int.from_bytes(pixel_data[sample_offset:sample_offset + 2], 'big')
                scaled[index] = int(sample * 255 / max_value)
            pixels = bytes(scaled)
    else:
        tokens = raw[position:].split()
        needed = width * height
        if len(tokens) < needed:
            raise ValueError('PGM pixel data is incomplete')
        scaled = bytearray(needed)
        for index in range(needed):
            scaled[index] = int(int(tokens[index]) * 255 / max_value)
        pixels = bytes(scaled)

    return encode_png_grayscale(width, height, pixels)


def encode_png_grayscale(width: int, height: int, pixels: bytes) -> bytes:
    import struct
    import zlib
    import binascii

    if len(pixels) != width * height:
        raise ValueError('Pixel buffer size does not match width and height')

    rows = []
    for row in range(height):
        start = row * width
        rows.append(b'\x00' + pixels[start:start + width])
    image_data = zlib.compress(b''.join(rows))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack('>I', len(data))
            + tag
            + data
            + struct.pack('>I', binascii.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return b''.join([
        b'\x89PNG\r\n\x1a\n',
        chunk(b'IHDR', ihdr),
        chunk(b'IDAT', image_data),
        chunk(b'IEND', b''),
    ])


def call_save_map(command_url: str, device_id: str, timeout: float = 20.0) -> None:
    payload = json.dumps(build_save_map_payload(device_id)).encode('utf-8')
    request = urllib.request.Request(
        command_url,
        data=payload,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        if response.status >= 400:
            raise RuntimeError(f'Save-map request failed: HTTP {response.status}')
        body = response.read().decode('utf-8', errors='replace').strip()
        if not body:
            return
        parsed = json.loads(body)
        if parsed.get('ok') is False:
            raise RuntimeError(parsed.get('message') or parsed.get('reason') or 'Save-map request failed')


def refresh_once(state: SharedState) -> None:
    attempted_at = time.strftime('%Y-%m-%dT%H:%M:%S')
    state.update_attempt(attempted_at)
    call_save_map(state.command_url, state.device_id)

    if not state.map_path.exists():
        raise FileNotFoundError(f'Map file not found: {state.map_path}')

    png_bytes = read_pgm_as_png_bytes(state.map_path)
    state.update_map(png_bytes, attempted_at)


def refresh_loop(state: SharedState) -> None:
    while not state.stop_event.is_set():
        try:
            refresh_once(state)
        except Exception as exc:
            state.update_error(str(exc), time.strftime('%Y-%m-%dT%H:%M:%S'))
        if state.stop_event.wait(state.interval):
            break


class MapRequestHandler(BaseHTTPRequestHandler):
    server_version = 'MapViewer/1.0'

    def do_GET(self) -> None:
        if self.path == '/' or self.path == '/index.html':
            self._serve_static('index.html', 'text/html; charset=utf-8')
            return
        if self.path == '/style.css':
            self._serve_static('style.css', 'text/css; charset=utf-8')
            return
        if self.path == '/app.js':
            self._serve_static('app.js', 'application/javascript; charset=utf-8')
            return
        if self.path.startswith('/api/status'):
            self._send_json(HTTPStatus.OK, {'ok': True, **self.server.shared_state.snapshot()})
            return
        if self.path.startswith('/api/map/latest'):
            image_bytes, updated_at = self.server.shared_state.latest_map()
            if image_bytes is None:
                self.send_response(HTTPStatus.NOT_FOUND)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                return

            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(image_bytes)))
            if updated_at:
                self.send_header('X-Map-Updated-At', updated_at)
            self.end_headers()
            self.wfile.write(image_bytes)
            return

        self._send_json(HTTPStatus.NOT_FOUND, {'ok': False, 'message': 'Not found'})

    def log_message(self, fmt: str, *args) -> None:
        print('%s - - [%s] %s' % (self.address_string(), self.log_date_time_string(), fmt % args))

    def _serve_static(self, filename: str, content_type: str) -> None:
        file_path = WEB_DIR / filename
        if not file_path.exists():
            self._send_json(HTTPStatus.NOT_FOUND, {'ok': False, 'message': f'Missing static file: {filename}'})
            return

        content = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_json(self, status: HTTPStatus, payload: dict) -> None:
        content = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)


class MapServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], shared_state: SharedState) -> None:
        super().__init__(server_address, MapRequestHandler)
        self.shared_state = shared_state


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Standalone Jetson map viewer service')
    parser.add_argument('--host', default=os.getenv('MAP_VIEWER_HOST', DEFAULT_HOST))
    parser.add_argument('--port', type=int, default=int(os.getenv('MAP_VIEWER_PORT', DEFAULT_PORT)))
    parser.add_argument('--command-url', default=os.getenv('MAP_VIEWER_COMMAND_URL', DEFAULT_COMMAND_URL))
    parser.add_argument('--map-file', default=os.getenv('MAP_VIEWER_MAP_FILE', DEFAULT_MAP_FILE))
    parser.add_argument('--interval', type=float, default=float(os.getenv('MAP_VIEWER_INTERVAL', DEFAULT_INTERVAL)))
    parser.add_argument('--device-id', default=os.getenv('MAP_VIEWER_DEVICE_ID', DEFAULT_DEVICE_ID))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    state = SharedState(
        map_path=Path(args.map_file),
        command_url=args.command_url,
        interval=args.interval,
        device_id=args.device_id,
    )

    worker = threading.Thread(target=refresh_loop, args=(state,), daemon=True)
    worker.start()

    server = MapServer((args.host, args.port), state)
    print(f'Map viewer running at http://{args.host}:{args.port}')
    print(f'Polling {args.command_url} every {args.interval} seconds')
    print(f'Reading map file: {args.map_file}')

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.stop_event.set()
        server.server_close()
        worker.join(timeout=2.0)


if __name__ == '__main__':
    main()
