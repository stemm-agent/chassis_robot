#!/usr/bin/env python3
"""Staged, production-safe observation-only YOLO11 node for the x86 NUC.

It never publishes velocity, navigation, or actuator commands.  The ONNX model
is always opened with ONNX Runtime's CPUExecutionProvider.  Upper/lower clothing
colour features and appearance re-identification are deliberately deferred; an
explicit lock command is required before Bodyposture observations are emitted.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from bodyreader_msg.msg import Bodyposture
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int8, String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


Box = Tuple[float, float, float, float]


@dataclass
class PersonDetection:
    box: Box
    score: float


@dataclass
class PersonTrack:
    track_id: int
    box: np.ndarray
    score: float
    hits: int = 1
    missed: int = 0


def _box_area(box: Sequence[float]) -> float:
    return max(0.0, float(box[2]) - float(box[0])) * max(
        0.0, float(box[3]) - float(box[1])
    )


def _box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    x1 = max(float(a[0]), float(b[0]))
    y1 = max(float(a[1]), float(b[1]))
    x2 = min(float(a[2]), float(b[2]))
    y2 = min(float(a[3]), float(b[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = _box_area(a) + _box_area(b) - intersection
    return intersection / union if union > 1.0e-9 else 0.0


def _nms(
    boxes: np.ndarray, scores: np.ndarray, iou_threshold: float, limit: int
) -> List[int]:
    """Small NumPy NMS implementation, returning indexes by descending score."""
    if boxes.size == 0:
        return []
    order = np.argsort(scores)[::-1]
    keep: List[int] = []
    while order.size and len(keep) < limit:
        current = int(order[0])
        keep.append(current)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[current, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[current, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[current, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[current, 3], boxes[rest, 3])
        widths = np.maximum(0.0, xx2 - xx1)
        heights = np.maximum(0.0, yy2 - yy1)
        intersection = widths * heights
        current_area = max(0.0, boxes[current, 2] - boxes[current, 0]) * max(
            0.0, boxes[current, 3] - boxes[current, 1]
        )
        rest_area = np.maximum(0.0, boxes[rest, 2] - boxes[rest, 0]) * np.maximum(
            0.0, boxes[rest, 3] - boxes[rest, 1]
        )
        union = current_area + rest_area - intersection
        ious = np.divide(
            intersection,
            union,
            out=np.zeros_like(intersection, dtype=np.float32),
            where=union > 1.0e-9,
        )
        order = rest[ious <= iou_threshold]
    return keep


class TorsoCpuDemoNode(Node):
    """Rate-limited CPU detector, tracker, lock selector, and depth projector."""

    def __init__(self) -> None:
        super().__init__("torso_cpu_demo")

        # All integration paths are parameters so this file can be staged safely.
        default_model = "~/wheeltec_ros2/src/ultralytics_ros2/model/yolo11n.onnx"
        self.model_path = Path(
            os.path.expandvars(os.path.expanduser(self._param("model_path", default_model)))
        )
        self.image_topic = self._param("image_topic", "/camera/color/image_raw")
        self.depth_topic = self._param(
            "registered_depth_topic", "/camera/depth/image_raw"
        )
        self.camera_info_topic = self._param(
            "camera_info_topic", "/camera/color/camera_info"
        )
        self.mode_topic = self._param("mode_topic", "/mode")
        self.lock_command_topic = self._param(
            "lock_command_topic", "/torso_cpu_demo/lock_command"
        )
        self.detections_topic = self._param(
            "detections_topic", "/torso_cpu_demo/detections"
        )
        self.features_topic = self._param(
            "features_topic", "/torso_cpu_demo/features"
        )
        self.locked_topic = self._param(
            "locked_topic", "/torso_cpu_demo/locked_target"
        )
        self.state_topic = self._param("state_topic", "/torso_cpu_demo/state")
        self.bodyposture_topic = self._param(
            "bodyposture_topic", "/body_posture_yolo_validated"
        )

        self.required_mode = int(self._param("required_mode", 2))
        self.current_mode = int(self._param("initial_mode", 0))
        # The operational hard cap is 10 Hz even if a larger parameter is supplied.
        requested_hz = float(self._param("max_inference_hz", 10.0))
        self.max_inference_hz = max(0.1, min(10.0, requested_hz))
        self.confidence_threshold = float(self._param("confidence_threshold", 0.45))
        self.nms_iou_threshold = float(self._param("nms_iou_threshold", 0.45))
        self.max_detections = max(1, int(self._param("max_detections", 20)))
        self.min_box_area_px = max(1.0, float(self._param("min_box_area_px", 600.0)))
        self.track_iou_threshold = float(self._param("track_iou_threshold", 0.20))
        self.track_center_threshold = float(
            self._param("track_center_threshold", 0.12)
        )
        self.track_smoothing = min(
            1.0, max(0.05, float(self._param("track_smoothing", 0.55)))
        )
        self.track_max_missed = max(1, int(self._param("track_max_missed", 8)))
        self.lock_min_hits = max(1, int(self._param("lock_min_hits", 3)))
        self.auto_lock_center = bool(self._param("auto_lock_center", False))
        # --- relock policy: prefer the last known position, allow time-based relaxation ---
        self.relock_memory_s = max(0.0, float(self._param("relock_memory_s", 3.0)))
        self.relock_max_distance = min(
            1.0, max(0.05, float(self._param("relock_max_distance", 0.35)))
        )
        self.relock_relax_after_s = max(
            0.0, float(self._param("relock_relax_after_s", 5.0))
        )
        self.relock_relax_min_hits = max(
            1, int(self._param("relock_relax_min_hits", 2))
        )
        self.relock_relax_min_score = min(
            1.0, max(0.0, float(self._param("relock_relax_min_score", 0.6)))
        )
        self._last_locked_center = None
        self._last_locked_time = 0.0
        self._last_locked_track_id = -1
        self._unlocked_since = 0.0
        self.depth_max_age_sec = max(
            0.05, float(self._param("depth_max_age_sec", 0.35))
        )
        self.depth_max_skew_sec = max(
            0.0, float(self._param("depth_max_skew_sec", 0.20))
        )
        # --- T0 perception diagnostics: observation only, never affects control ---
        self.diag_enabled = bool(self._param("diag_enabled", True))
        self.diag_heartbeat_sec = max(0.2, float(self._param("diag_heartbeat_sec", 1.0)))
        self.diag_gap_threshold_s = max(0.05, float(self._param("diag_gap_threshold_s", 0.80)))
        self._diag_frames_total = 0
        self._diag_frames_valid = 0
        self._diag_frames_no_detection = 0
        self._diag_frames_no_track = 0
        self._diag_frames_no_lock = 0
        self._diag_frames_lock_unconfirmed = 0
        self._diag_reasons = {}
        self._diag_gap_list = []
        self._diag_gap_max_s = 0.0
        self._diag_gap_over_count = 0
        self._diag_invalid_since = 0.0
        self._diag_active_since = 0.0
        self._diag_last_publish = 0.0
        self._diag_window = []
        self._diag_reason_current = "unknown"
        self._diag_depth_age_ms = None
        self._diag_depth_skew_ms = None
        self._diag_roi_valid_px = None
        self._last_state_name = "init"
        self.depth_unit_scale = float(self._param("depth_unit_scale", 0.001))
        self.min_depth_m = max(0.01, float(self._param("min_depth_m", 0.20)))
        self.max_depth_m = max(
            self.min_depth_m, float(self._param("max_depth_m", 8.0))
        )
        self.depth_roi_width_ratio = min(
            1.0, max(0.05, float(self._param("depth_roi_width_ratio", 0.30)))
        )
        self.depth_roi_height_ratio = min(
            1.0, max(0.05, float(self._param("depth_roi_height_ratio", 0.25)))
        )
        self.input_width = 640
        self.input_height = 640

        self.bridge = CvBridge()
        self._inference_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._last_inference_start = 0.0
        self._latest_depth: Optional[Image] = None
        self._latest_depth_received = 0.0
        self._camera_info: Optional[CameraInfo] = None
        self._tracks: List[PersonTrack] = []
        self._visible_tracks: List[PersonTrack] = []
        self._next_track_id = 1
        self._locked_track_id = -1
        self._pending_lock_command: Optional[str] = None
        self._last_valid_body = False

        self.session: Optional[ort.InferenceSession] = None
        self._ov_request = None
        self._ov_output_port = None
        self.inference_backend = "onnxruntime"
        self.input_name = "images"
        self.output_name = "output0"
        self.ready = self._load_model()

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.detections_pub = self.create_publisher(
            Detection2DArray, self.detections_topic, command_qos
        )
        self.features_pub = self.create_publisher(String, self.features_topic, command_qos)
        self.locked_pub = self.create_publisher(
            Detection2DArray, self.locked_topic, command_qos
        )
        self.state_pub = self.create_publisher(String, self.state_topic, command_qos)
        self.bodyposture_pub = self.create_publisher(
            Bodyposture, self.bodyposture_topic, command_qos
        )
        self._subscriptions = [
            self.create_subscription(Image, self.image_topic, self._image_callback, sensor_qos),
            self.create_subscription(Image, self.depth_topic, self._depth_callback, sensor_qos),
            self.create_subscription(
                CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
            ),
            self.create_subscription(Int8, self.mode_topic, self._mode_callback, sensor_qos),
            self.create_subscription(
                String, self.lock_command_topic, self._lock_command_callback, command_qos
            ),
        ]

        self._publish_state("ready" if self.ready else "model_not_ready")
        self.get_logger().info(
            "CPU person detector ready=%s model=%s backend=%s required_mode=%d max_hz=%.1f"
            % (
                self.ready,
                self.model_path,
                self.inference_backend,
                self.required_mode,
                self.max_inference_hz,
            )
        )

    def _param(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _load_model(self) -> bool:
        if not self.model_path.is_file():
            self.get_logger().error("ONNX model not found: %s" % self.model_path)
            return False
        self.inference_backend = str(
            self._param("inference_backend", "openvino")
        ).strip().lower()
        if self.inference_backend == "openvino":
            if self._load_model_openvino():
                return True
            self.get_logger().warning(
                "OpenVINO backend unavailable; falling back to onnxruntime"
            )
            self.inference_backend = "onnxruntime"
        return self._load_model_onnxruntime()

    def _load_model_openvino(self) -> bool:
        try:
            import openvino as ov
        except Exception as exc:
            self.get_logger().warning("openvino import failed: %s" % exc)
            return False
        try:
            core = ov.Core()
            model = core.read_model(str(self.model_path))
            input_shape = [int(dim) for dim in model.input(0).shape]
            output_shape = [int(dim) for dim in model.output(0).shape]
            if len(input_shape) != 4 or input_shape[-2:] != [640, 640]:
                raise RuntimeError("expected images [1,3,640,640], got %s" % input_shape)
            if len(output_shape) != 3:
                raise RuntimeError("expected output0 rank 3, got %s" % output_shape)
            threads = max(
                1,
                int(
                    self._param(
                        "onnx_intra_op_threads",
                        max(1, min(6, (os.cpu_count() or 2) // 2)),
                    )
                ),
            )
            config = {
                "PERFORMANCE_HINT": str(
                    self._param("openvino_performance_hint", "LATENCY")
                ),
                "INFERENCE_NUM_THREADS": str(threads),
            }
            compiled = core.compile_model(
                model, str(self._param("openvino_device", "CPU")), config
            )
            self._ov_request = compiled.create_infer_request()
            try:
                self._ov_output_port = compiled.output(0)
            except Exception:
                self._ov_output_port = None
            self.session = None
            return True
        except Exception as exc:
            self._ov_request = None
            self._ov_output_port = None
            self.get_logger().error("Failed to load OpenVINO model: %s" % exc)
            return False

    def _infer(self, tensor: np.ndarray) -> np.ndarray:
        if self._ov_request is not None:
            result = self._ov_request.infer({0: tensor})
            array = None
            if self._ov_output_port is not None:
                try:
                    array = result[self._ov_output_port]
                except Exception:
                    array = None
            if array is None and hasattr(result, "values"):
                values = list(result.values())
                if values:
                    array = values[0]
            if array is None:
                array = result
            out = np.asarray(array)
            if out.ndim != 3 and out.dtype == object:
                raise RuntimeError(
                    "OpenVINO output extraction failed: %s %s"
                    % (type(result).__name__, np.shape(out))
                )
            return out
        assert self.session is not None
        return self.session.run([self.output_name], {self.input_name: tensor})[0]

    def _load_model_onnxruntime(self) -> bool:
        try:
            options = ort.SessionOptions()
            options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            default_threads = max(1, min(6, (os.cpu_count() or 2) // 2))
            options.intra_op_num_threads = max(
                1, int(self._param("onnx_intra_op_threads", default_threads))
            )
            options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(
                str(self.model_path),
                sess_options=options,
                providers=["CPUExecutionProvider"],
            )
            inputs = {item.name: item for item in self.session.get_inputs()}
            outputs = {item.name: item for item in self.session.get_outputs()}
            if self.input_name not in inputs or self.output_name not in outputs:
                raise RuntimeError(
                    "expected ONNX tensors images/output0, got inputs=%s outputs=%s"
                    % (sorted(inputs), sorted(outputs))
                )
            input_shape = list(inputs[self.input_name].shape)
            output_shape = list(outputs[self.output_name].shape)
            if len(input_shape) != 4 or input_shape[-2:] != [640, 640]:
                raise RuntimeError("expected images [1,3,640,640], got %s" % input_shape)
            if len(output_shape) != 3:
                raise RuntimeError("expected output0 rank 3, got %s" % output_shape)
            if "CPUExecutionProvider" not in self.session.get_providers():
                raise RuntimeError("CPUExecutionProvider is unavailable")
            return True
        except Exception as exc:  # The node remains alive and reports a clear state.
            self.session = None
            self.get_logger().error("Failed to load ONNX model on CPU: %s" % exc)
            return False

    def _depth_callback(self, msg: Image) -> None:
        with self._data_lock:
            self._latest_depth = msg
            self._latest_depth_received = time.monotonic()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        with self._data_lock:
            self._camera_info = msg

    def _mode_callback(self, msg: Int8) -> None:
        next_mode = int(msg.data)
        if next_mode == self.current_mode:
            return
        was_active = self.current_mode == self.required_mode
        self.current_mode = next_mode
        self._last_inference_start = 0.0
        if next_mode == self.required_mode:
            self._reset_tracks()
            self._diag_reset()
            self._publish_state("active")
        elif was_active:
            self._reset_tracks()
            self._publish_invalid_body()
            self._publish_state("paused")

    def _lock_command_callback(self, msg: String) -> None:
        command = msg.data.strip().lower()
        if command in {"clear", "unlock", "none"}:
            self._locked_track_id = -1
            self._pending_lock_command = None
            self._publish_invalid_body()
            self._publish_state("unlocked")
            return
        self._pending_lock_command = command

    def _reset_tracks(self) -> None:
        self._tracks.clear()
        self._visible_tracks.clear()
        self._locked_track_id = -1
        self._pending_lock_command = None

    def _image_callback(self, msg: Image) -> None:
        if not self.ready or self.current_mode != self.required_mode:
            return
        now = time.monotonic()
        if now - self._last_inference_start < 1.0 / self.max_inference_hz:
            return
        if not self._inference_lock.acquire(blocking=False):
            return
        self._last_inference_start = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            tensor, scale, pad_x, pad_y = self._letterbox(bgr)
            raw = self._infer(tensor)
            detections = self._parse_output(
                raw, scale, pad_x, pad_y, bgr.shape[1], bgr.shape[0]
            )
            visible = self._update_tracks(detections, bgr.shape[1], bgr.shape[0])
            self._apply_pending_lock(visible, bgr.shape[1], bgr.shape[0])
            stable = [track for track in visible if track.hits >= self.lock_min_hits]
            if self.auto_lock_center and self._locked_track_id < 0:
                if stable:
                    self._locked_track_id = self._select_relock(
                        stable, bgr.shape[1], bgr.shape[0]
                    )
                elif (
                    self.relock_relax_after_s > 0.0
                    and self._unlocked_since > 0.0
                    and (now - self._unlocked_since) >= self.relock_relax_after_s
                ):
                    relaxed = [
                        track
                        for track in visible
                        if track.hits >= self.relock_relax_min_hits
                        and float(track.score) >= self.relock_relax_min_score
                    ]
                    if relaxed:
                        self._locked_track_id = self._select_relock(
                            relaxed, bgr.shape[1], bgr.shape[0]
                        )
                        chosen = next(
                            (
                                track
                                for track in relaxed
                                if track.track_id == self._locked_track_id
                            ),
                            None,
                        )
                        self.get_logger().warning(
                            "Relock relaxed after %.1fs without lock: track=%d hits=%d score=%.2f"
                            % (
                                now - self._unlocked_since,
                                self._locked_track_id,
                                0 if chosen is None else chosen.hits,
                                0.0 if chosen is None else float(chosen.score),
                            )
                        )
            self._publish_outputs(
                msg, visible, bgr.shape[1], bgr.shape[0], len(detections)
            )
            self._maybe_publish_diag()
        except (CvBridgeError, ValueError, RuntimeError) as exc:
            self.get_logger().warning("CPU person inference skipped: %s" % exc)
            self._publish_state("inference_error", detail=str(exc))
        except Exception as exc:  # Keep the sensor callback alive on malformed frames.
            self.get_logger().error("Unexpected CPU inference error: %s" % exc)
            self._publish_state("inference_error", detail=type(exc).__name__)
        finally:
            self._inference_lock.release()

    def _letterbox(self, bgr: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError("expected a three-channel BGR image")
        height, width = bgr.shape[:2]
        if width < 2 or height < 2:
            raise ValueError("image is too small")
        scale = min(self.input_width / width, self.input_height / height)
        resized_width = max(1, int(round(width * scale)))
        resized_height = max(1, int(round(height * scale)))
        resized = cv2.resize(
            bgr, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
        )
        pad_x = (self.input_width - resized_width) // 2
        pad_y = (self.input_height - resized_height) // 2
        canvas = np.full(
            (self.input_height, self.input_width, 3), 114, dtype=np.uint8
        )
        canvas[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = resized
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32)
        tensor *= 1.0 / 255.0
        return tensor[np.newaxis, ...], scale, pad_x, pad_y

    def _parse_output(
        self,
        raw: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
        image_width: int,
        image_height: int,
    ) -> List[PersonDetection]:
        output = np.asarray(raw)
        if output.ndim != 3 or output.shape[0] != 1:
            raise ValueError("output0 must have shape [1,84,8400]")
        matrix = output[0]
        if matrix.shape == (84, 8400):
            matrix = matrix.T
        elif matrix.shape == (8400, 84):
            pass
        else:
            raise ValueError("unexpected output0 shape %s" % (output.shape,))

        # YOLO11 export: xywh + 80 class probabilities.  Person is class 0.
        scores = matrix[:, 4]
        selected = np.flatnonzero(scores >= self.confidence_threshold)
        if selected.size == 0:
            return []
        if selected.size > 1000:
            selected = selected[np.argsort(scores[selected])[-1000:]]
        xywh = matrix[selected, :4].astype(np.float32, copy=False)
        selected_scores = scores[selected].astype(np.float32, copy=False)
        boxes = np.empty_like(xywh)
        boxes[:, 0] = (xywh[:, 0] - 0.5 * xywh[:, 2] - pad_x) / scale
        boxes[:, 1] = (xywh[:, 1] - 0.5 * xywh[:, 3] - pad_y) / scale
        boxes[:, 2] = (xywh[:, 0] + 0.5 * xywh[:, 2] - pad_x) / scale
        boxes[:, 3] = (xywh[:, 1] + 0.5 * xywh[:, 3] - pad_y) / scale
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0.0, float(image_width - 1))
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0.0, float(image_height - 1))
        valid = (
            (boxes[:, 2] > boxes[:, 0])
            & (boxes[:, 3] > boxes[:, 1])
            & (
                (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
                >= self.min_box_area_px
            )
        )
        boxes = boxes[valid]
        selected_scores = selected_scores[valid]
        keep = _nms(
            boxes, selected_scores, self.nms_iou_threshold, self.max_detections
        )
        return [
            PersonDetection(tuple(float(v) for v in boxes[index]), float(selected_scores[index]))
            for index in keep
        ]

    def _update_tracks(
        self, detections: List[PersonDetection], width: int, height: int
    ) -> List[PersonTrack]:
        diagonal = max(1.0, math.hypot(width, height))
        candidates = []
        for track_index, track in enumerate(self._tracks):
            old_center = ((track.box[0] + track.box[2]) * 0.5, (track.box[1] + track.box[3]) * 0.5)
            for detection_index, detection in enumerate(detections):
                box = detection.box
                center = ((box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5)
                distance = math.hypot(center[0] - old_center[0], center[1] - old_center[1]) / diagonal
                iou = _box_iou(track.box, box)
                if iou >= self.track_iou_threshold or distance <= self.track_center_threshold:
                    candidates.append((iou - 0.20 * distance, track_index, detection_index))
        candidates.sort(reverse=True)
        used_tracks = set()
        used_detections = set()
        for _, track_index, detection_index in candidates:
            if track_index in used_tracks or detection_index in used_detections:
                continue
            track = self._tracks[track_index]
            detection = detections[detection_index]
            alpha = self.track_smoothing
            track.box = (1.0 - alpha) * track.box + alpha * np.asarray(
                detection.box, dtype=np.float32
            )
            track.score = detection.score
            track.hits += 1
            track.missed = 0
            used_tracks.add(track_index)
            used_detections.add(detection_index)

        for index, track in enumerate(self._tracks):
            if index not in used_tracks:
                track.missed += 1
        for index, detection in enumerate(detections):
            if index in used_detections:
                continue
            self._tracks.append(
                PersonTrack(
                    track_id=self._allocate_track_id(),
                    box=np.asarray(detection.box, dtype=np.float32),
                    score=detection.score,
                )
            )
        locked_limit = self.track_max_missed * 3
        self._tracks = [
            track
            for track in self._tracks
            if track.missed
            <= (locked_limit if track.track_id == self._locked_track_id else self.track_max_missed)
        ]
        if self._locked_track_id >= 0 and not any(
            track.track_id == self._locked_track_id for track in self._tracks
        ):
            self._locked_track_id = -1
        self._visible_tracks = [track for track in self._tracks if track.missed == 0]
        return self._visible_tracks

    def _allocate_track_id(self) -> int:
        used = {track.track_id for track in self._tracks}
        for _ in range(32766):
            candidate = self._next_track_id
            self._next_track_id = 1 if candidate >= 32766 else candidate + 1
            if candidate not in used:
                return candidate
        raise RuntimeError("no Bodyposture-compatible track IDs are available")

    def _apply_pending_lock(
        self, tracks: List[PersonTrack], width: int, height: int
    ) -> None:
        command = self._pending_lock_command
        if command is None:
            return
        self._pending_lock_command = None
        if command in {"center", "lock_center"}:
            self._locked_track_id = self._select_center(tracks, width, height)
        elif command in {"largest", "lock_largest"}:
            self._locked_track_id = (
                max(tracks, key=lambda track: _box_area(track.box)).track_id if tracks else -1
            )
        else:
            value = command
            for prefix in ("track:", "id:"):
                if value.startswith(prefix):
                    value = value[len(prefix) :].strip()
                    break
            try:
                requested = int(value)
            except ValueError:
                self.get_logger().warning("Unknown lock command: %s" % command)
                return
            if any(track.track_id == requested for track in self._tracks):
                self._locked_track_id = requested
            else:
                self.get_logger().warning("Requested track is unavailable: %d" % requested)

    def _select_relock(
        self, tracks: List[PersonTrack], width: int, height: int
    ) -> int:
        """Prefer the candidate closest to the last confirmed locked position."""
        if not tracks:
            return -1
        now = time.monotonic()
        if (
            self._last_locked_center is not None
            and self.relock_memory_s > 0.0
            and (now - self._last_locked_time) <= self.relock_memory_s
        ):
            last_x, last_y = self._last_locked_center
            best = None
            best_distance = None
            for track in tracks:
                cx = ((track.box[0] + track.box[2]) * 0.5) / max(1.0, float(width))
                cy = ((track.box[1] + track.box[3]) * 0.5) / max(1.0, float(height))
                distance = math.hypot(cx - last_x, cy - last_y)
                if best_distance is None or distance < best_distance:
                    best = track
                    best_distance = distance
            if best is not None and best_distance is not None:
                if best_distance <= self.relock_max_distance:
                    return best.track_id
        return self._select_center(tracks, width, height)

    def _note_lock_state(
        self, locked_track, lock_confirmed: bool, width: int, height: int
    ) -> None:
        now = time.monotonic()
        if lock_confirmed and locked_track is not None:
            self._last_locked_center = (
                ((locked_track.box[0] + locked_track.box[2]) * 0.5)
                / max(1.0, float(width)),
                ((locked_track.box[1] + locked_track.box[3]) * 0.5)
                / max(1.0, float(height)),
            )
            self._last_locked_time = now
            self._last_locked_track_id = int(locked_track.track_id)
            self._unlocked_since = 0.0
        elif self._unlocked_since <= 0.0:
            self._unlocked_since = now

    @staticmethod
    def _select_center(tracks: List[PersonTrack], width: int, height: int) -> int:
        if not tracks:
            return -1
        cx, cy = width * 0.5, height * 0.5
        return min(
            tracks,
            key=lambda track: math.hypot(
                (track.box[0] + track.box[2]) * 0.5 - cx,
                (track.box[1] + track.box[3]) * 0.5 - cy,
            ),
        ).track_id

    def _publish_outputs(
        self,
        image_msg: Image,
        tracks: List[PersonTrack],
        width: int,
        height: int,
        detection_count: int = 0,
    ) -> None:
        all_msg = Detection2DArray()
        all_msg.header = image_msg.header
        locked_msg = Detection2DArray()
        locked_msg.header = image_msg.header
        locked_track = next(
            (track for track in tracks if track.track_id == self._locked_track_id), None
        )
        position = self._position_from_depth(
            locked_track, width, height, image_msg.header.stamp
        )
        targets = []
        for track in tracks:
            locked = track.track_id == self._locked_track_id
            detection_msg = self._detection_message(image_msg, track, locked)
            all_msg.detections.append(detection_msg)
            if locked:
                locked_msg.detections.append(detection_msg)
            box = [round(float(value), 2) for value in track.box]
            depth_m = position[2] if locked and position is not None else None
            targets.append(
                {
                    "track_id": track.track_id,
                    "locked": locked,
                    "score": round(track.score, 4),
                    "hits": track.hits,
                    "stable": track.hits >= self.lock_min_hits,
                    "class_id": 0,
                    "class_name": "person",
                    "depth_m": round(depth_m, 4) if depth_m is not None else None,
                    "height_ratio": round(max(0.0, box[3] - box[1]) / max(1, height), 4),
                    "person_bbox": box,
                    "center": [round((box[0] + box[2]) * 0.5, 2), round((box[1] + box[3]) * 0.5, 2)],
                }
            )
        self.detections_pub.publish(all_msg)
        self.locked_pub.publish(locked_msg)
        features = String()
        features.data = json.dumps(
            {
                "stamp": {
                    "sec": int(image_msg.header.stamp.sec),
                    "nanosec": int(image_msg.header.stamp.nanosec),
                },
                "image_size": [width, height],
                "locked_track_id": self._locked_track_id,
                "locked_visible": locked_track is not None,
                "appearance_reidentification": "not_implemented",
                "targets": targets,
            },
            separators=(",", ":"),
        )
        self.features_pub.publish(features)

        lock_confirmed = locked_track is not None and locked_track.hits >= self.lock_min_hits
        self._diag_note_frame(
            len(tracks),
            int(detection_count),
            locked_track is not None,
            bool(lock_confirmed),
            position is not None,
        )
        self._note_lock_state(locked_track, bool(lock_confirmed), width, height)
        if lock_confirmed and position is not None:
            left_m, up_m, forward_m = position
            body = Bodyposture()
            body.bodyid = int(locked_track.track_id)
            body.centerofmass_x = float(left_m * 1000.0)
            body.centerofmass_y = float(up_m * 1000.0)
            body.centerofmass_z = float(forward_m * 1000.0)
            body.lock_status = 2
            self.bodyposture_pub.publish(body)
            self._last_valid_body = True
            self._publish_state("locked_visible", depth_m=forward_m)
        else:
            self._publish_invalid_body()
            if locked_track is not None and not lock_confirmed:
                state = "lock_confirming"
            else:
                state = "locked_without_depth" if locked_track is not None else "detecting"
            self._publish_state(state)

    @staticmethod
    def _detection_message(
        image_msg: Image, track: PersonTrack, locked: bool
    ) -> Detection2D:
        x1, y1, x2, y2 = (float(value) for value in track.box)
        msg = Detection2D()
        msg.header = image_msg.header
        msg.bbox.center.position.x = (x1 + x2) * 0.5
        msg.bbox.center.position.y = (y1 + y2) * 0.5
        msg.bbox.center.theta = 0.0
        msg.bbox.size_x = max(0.0, x2 - x1)
        msg.bbox.size_y = max(0.0, y2 - y1)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "locked_person" if locked else "person"
        hypothesis.hypothesis.score = float(track.score)
        msg.results.append(hypothesis)
        return msg

    def _position_from_depth(
        self,
        track: Optional[PersonTrack],
        color_width: int,
        color_height: int,
        color_stamp,
    ) -> Optional[Tuple[float, float, float]]:
        if track is None:
            return None
        with self._data_lock:
            depth_msg = self._latest_depth
            depth_received = self._latest_depth_received
            camera_info = self._camera_info
        if depth_msg is None or camera_info is None:
            self._diag_reason_current = "no_depth_msg"
            return None
        if time.monotonic() - depth_received > self.depth_max_age_sec:
            self._diag_reason_current = "depth_stale"
            return None
        color_stamp_ns = int(color_stamp.sec) * 1_000_000_000 + int(color_stamp.nanosec)
        depth_stamp_ns = (
            int(depth_msg.header.stamp.sec) * 1_000_000_000
            + int(depth_msg.header.stamp.nanosec)
        )
        self._diag_depth_age_ms = round((time.monotonic() - depth_received) * 1000.0, 1)
        self._diag_depth_skew_ms = (
            round(abs(color_stamp_ns - depth_stamp_ns) * 1.0e-6, 1)
            if color_stamp_ns > 0 and depth_stamp_ns > 0
            else None
        )
        if (
            self.depth_max_skew_sec > 0.0
            and color_stamp_ns > 0
            and depth_stamp_ns > 0
            and abs(color_stamp_ns - depth_stamp_ns) * 1.0e-9 > self.depth_max_skew_sec
        ):
            self._diag_reason_current = "depth_skew"
            return None
        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError:
            self._diag_reason_current = "depth_decode_failed"
            return None
        if depth.ndim != 2 or depth.size == 0:
            self._diag_reason_current = "depth_bad_shape"
            return None

        depth_height, depth_width = depth.shape
        scale_x = depth_width / max(1.0, float(color_width))
        scale_y = depth_height / max(1.0, float(color_height))
        x1, y1, x2, y2 = (float(value) for value in track.box)
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        roi_width = max(3.0, (x2 - x1) * self.depth_roi_width_ratio)
        roi_height = max(3.0, (y2 - y1) * self.depth_roi_height_ratio)
        dx1 = max(0, int(round((center_x - roi_width * 0.5) * scale_x)))
        dx2 = min(depth_width, int(round((center_x + roi_width * 0.5) * scale_x)))
        dy1 = max(0, int(round((center_y - roi_height * 0.5) * scale_y)))
        dy2 = min(depth_height, int(round((center_y + roi_height * 0.5) * scale_y)))
        if dx2 <= dx1 or dy2 <= dy1:
            self._diag_reason_current = "roi_empty"
            return None
        values = np.asarray(depth[dy1:dy2, dx1:dx2], dtype=np.float32)
        # 16-bit Astra depth is millimetres; 32FC1 is already metres.
        if depth_msg.encoding.upper() in {"16UC1", "MONO16"} or np.issubdtype(
            depth.dtype, np.integer
        ):
            values *= self.depth_unit_scale
        valid = values[
            np.isfinite(values)
            & (values >= self.min_depth_m)
            & (values <= self.max_depth_m)
        ]
        self._diag_roi_valid_px = int(valid.size)
        if valid.size < 5:
            self._diag_reason_current = "roi_few_valid_px"
            return None
        forward_m = float(np.median(valid))

        k = camera_info.k
        if len(k) != 9 or k[0] <= 1.0e-6 or k[4] <= 1.0e-6:
            self._diag_reason_current = "bad_intrinsics"
            return None
        info_width = camera_info.width or color_width
        info_height = camera_info.height or color_height
        intrinsics_scale_x = color_width / max(1.0, float(info_width))
        intrinsics_scale_y = color_height / max(1.0, float(info_height))
        fx = float(k[0]) * intrinsics_scale_x
        fy = float(k[4]) * intrinsics_scale_y
        cx = float(k[2]) * intrinsics_scale_x
        cy = float(k[5]) * intrinsics_scale_y
        # Camera optical pixels are right/down positive.  Bodyposture consumers in
        # this workspace interpret x/y as left/up and z as forward, in millimetres.
        left_m = -(center_x - cx) * forward_m / fx
        up_m = -(center_y - cy) * forward_m / fy
        self._diag_reason_current = "ok"
        return left_m, up_m, forward_m

    def _diag_reset(self) -> None:
        self._diag_frames_total = 0
        self._diag_frames_valid = 0
        self._diag_frames_no_detection = 0
        self._diag_frames_no_track = 0
        self._diag_frames_no_lock = 0
        self._diag_frames_lock_unconfirmed = 0
        self._diag_reasons = {}
        self._diag_gap_list = []
        self._diag_gap_max_s = 0.0
        self._diag_gap_over_count = 0
        self._diag_invalid_since = 0.0
        self._diag_active_since = 0.0
        self._diag_window = []

    def _diag_note_frame(
        self,
        track_count: int,
        detection_count: int,
        locked_visible: bool,
        lock_confirmed: bool,
        depth_ok: bool,
    ) -> None:
        if not self.diag_enabled:
            return
        now = time.monotonic()
        if self._diag_active_since <= 0.0:
            self._diag_active_since = now
        self._diag_frames_total += 1
        if detection_count <= 0:
            self._diag_frames_no_detection += 1
        if track_count <= 0:
            self._diag_frames_no_track += 1
        reason = "ok"
        if not locked_visible:
            reason = "no_lock"
            self._diag_frames_no_lock += 1
        elif not lock_confirmed:
            reason = "lock_unconfirmed"
            self._diag_frames_lock_unconfirmed += 1
        elif not depth_ok:
            reason = self._diag_reason_current
        if reason == "ok":
            self._diag_frames_valid += 1
            if self._diag_invalid_since > 0.0:
                gap = now - self._diag_invalid_since
                self._diag_invalid_since = 0.0
                self._diag_gap_max_s = max(self._diag_gap_max_s, gap)
                self._diag_gap_list.append(round(gap, 3))
                if len(self._diag_gap_list) > 20:
                    self._diag_gap_list = self._diag_gap_list[-20:]
                if gap > self.diag_gap_threshold_s:
                    self._diag_gap_over_count += 1
        else:
            self._diag_reasons[reason] = self._diag_reasons.get(reason, 0) + 1
            if self._diag_invalid_since <= 0.0:
                self._diag_invalid_since = now
        self._diag_window.append((now, reason == "ok"))
        cutoff = now - 10.0
        while self._diag_window and self._diag_window[0][0] < cutoff:
            self._diag_window.pop(0)

    def _maybe_publish_diag(self) -> None:
        if not self.diag_enabled:
            return
        now = time.monotonic()
        if now - self._diag_last_publish < self.diag_heartbeat_sec:
            return
        self._diag_last_publish = now
        active_s = now - self._diag_active_since if self._diag_active_since > 0.0 else 0.0
        window = [ok for _, ok in self._diag_window]
        payload = {
            "state": self._last_state_name,
            "heartbeat": True,
            "diag": {
                "frames": self._diag_frames_total,
                "valid": self._diag_frames_valid,
                "valid_pct": round(
                    100.0 * self._diag_frames_valid / max(1, self._diag_frames_total), 2
                ),
                "valid_pct_10s": round(
                    100.0 * sum(1 for ok in window if ok) / max(1, len(window)), 2
                ),
                "rate_hz": (
                    round(self._diag_frames_total / active_s, 2) if active_s > 0.5 else None
                ),
                "active_s": round(active_s, 1),
                "no_detection": self._diag_frames_no_detection,
                "no_track": self._diag_frames_no_track,
                "no_lock": self._diag_frames_no_lock,
                "lock_unconfirmed": self._diag_frames_lock_unconfirmed,
                "reasons": dict(
                    sorted(self._diag_reasons.items(), key=lambda kv: -kv[1])[:8]
                ),
                "gaps_s": self._diag_gap_list[-12:],
                "gap_max_s": round(self._diag_gap_max_s, 3),
                "gap_over_threshold": self._diag_gap_over_count,
                "gap_threshold_s": self.diag_gap_threshold_s,
                "depth_age_ms": self._diag_depth_age_ms,
                "depth_skew_ms": self._diag_depth_skew_ms,
                "roi_valid_px": self._diag_roi_valid_px,
                "lock_memory_track": self._last_locked_track_id,
                "unlocked_s": (
                    round(time.monotonic() - self._unlocked_since, 1)
                    if self._unlocked_since > 0.0
                    else 0.0
                ),
            },
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        self.state_pub.publish(msg)

    def _publish_invalid_body(self) -> None:
        if not self._last_valid_body:
            return
        body = Bodyposture()
        body.lock_status = 0
        self.bodyposture_pub.publish(body)
        self._last_valid_body = False

    def _publish_state(self, state: str, **extra) -> None:
        msg = String()
        payload = {
            "state": state,
            "ready": self.ready,
            "mode": self.current_mode,
            "required_mode": self.required_mode,
            "locked_track_id": self._locked_track_id,
            "inference_backend": self.inference_backend,
        }
        payload.update(extra)
        self._last_state_name = state
        msg.data = json.dumps(payload, separators=(",", ":"), ensure_ascii=True)
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TorsoCpuDemoNode()
    try:
        # SingleThreadedExecutor plus depth-1 QoS guarantees that image callbacks
        # cannot stack up behind a slow CPU inference.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
