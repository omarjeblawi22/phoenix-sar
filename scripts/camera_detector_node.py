#!/usr/bin/env python3
"""
camera_detector_node.py — PHOENIX SAR Camera Target Detection Node
===================================================================

Captures frames from the Raspberry Pi Camera Module 3 via rpicam-vid
(primary — Pi 5 / Ubuntu 24.04) or any OpenCV-compatible camera (fallback),
runs TFLite inference on each frame, and publishes when a target is detected.

Camera priority:
  1. rpicam-vid MJPEG pipe  — works on Pi 5 after building Pi libcamera from source
  2. OpenCV VideoCapture    — works with USB webcams or /dev/video0

Topics published:
  /phoenix/target_detected   (std_msgs/Bool)    — True when stably detected
  /phoenix/detection_prob    (std_msgs/Float32) — raw probability each frame

Parameters:
  model_path          Path to .tflite model file
  metadata_path       Path to metadata.json
  use_rpicam          Use rpicam-vid as primary capture (default True)
  camera_index        OpenCV fallback index (default 0)
  camera_device       OpenCV fallback device path e.g. '/dev/video0'
  threshold           Detection threshold (-1.0 = use metadata value)
  smoothing_window    Rolling window size (default 5)
  required_positives  Votes needed to confirm detection (default 3)
  frame_width         Capture width  (default 640)
  frame_height        Capture height (default 480)
"""

from __future__ import annotations

import datetime
import json
import queue
import shutil
import subprocess
import sys
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32

try:
    import tflite_runtime.interpreter as tflite
except ImportError:
    try:
        import tensorflow.lite as tflite
    except ImportError:
        from tensorflow import lite as tflite


class CameraDetectorNode(Node):

    def __init__(self):
        super().__init__('camera_detector')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('model_path',
                               '/home/phoenix/model/target_classifier_int8.tflite')
        self.declare_parameter('metadata_path',
                               '/home/phoenix/model/metadata.json')
        self.declare_parameter('use_rpicam',        True)
        self.declare_parameter('camera_index',      0)
        self.declare_parameter('camera_device',     '')
        self.declare_parameter('threshold',        -1.0)
        self.declare_parameter('smoothing_window',  5)
        self.declare_parameter('required_positives', 3)
        self.declare_parameter('frame_width',      640)
        self.declare_parameter('frame_height',     480)
        self.declare_parameter('save_video',        True)
        self.declare_parameter('video_dir',
                               '/home/phoenix/ros2_ws/maps/camera_runs')

        model_path       = self.get_parameter('model_path').value
        metadata_path    = self.get_parameter('metadata_path').value
        use_rpicam       = bool(self.get_parameter('use_rpicam').value)
        camera_index     = self.get_parameter('camera_index').value
        camera_device    = self.get_parameter('camera_device').value
        threshold_param  = float(self.get_parameter('threshold').value)
        smoothing_window = self.get_parameter('smoothing_window').value
        self._req_pos    = self.get_parameter('required_positives').value
        frame_width      = self.get_parameter('frame_width').value
        frame_height     = self.get_parameter('frame_height').value
        save_video       = bool(self.get_parameter('save_video').value)
        video_dir        = self.get_parameter('video_dir').value

        # ── Metadata ──────────────────────────────────────────────────
        meta = json.loads(Path(metadata_path).read_text(encoding='utf-8'))
        self._img_size       = int(meta['img_size'])
        self._positive_class = meta.get('positive_class', 'target')
        self._threshold      = (threshold_param if threshold_param > 0.0
                                else float(meta.get('recommended_threshold', 0.5)))

        # ── TFLite model ──────────────────────────────────────────────
        self._interp = tflite.Interpreter(model_path=str(model_path))
        self._interp.allocate_tensors()
        self._in_det  = self._interp.get_input_details()[0]
        self._out_det = self._interp.get_output_details()[0]

        # ── Video writer ──────────────────────────────────────────────
        self._video_writer = None
        if save_video:
            Path(video_dir).mkdir(parents=True, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            video_path = str(Path(video_dir) / f'mission_{ts}.mp4')
            self._video_writer = cv2.VideoWriter(
                video_path,
                cv2.VideoWriter_fourcc(*'mp4v'),
                10.0,
                (frame_width, frame_height),
            )
            if self._video_writer.isOpened():
                self.get_logger().info(f'Recording camera to: {video_path}')
            else:
                self.get_logger().warn(f'Could not open video writer: {video_path}')
                self._video_writer = None

        # ── Camera ────────────────────────────────────────────────────
        self._use_rpicam   = False
        self._rpicam_proc  = None
        self._frame_queue  = queue.Queue(maxsize=2)
        self._cap          = None

        if use_rpicam and self._init_rpicam(frame_width, frame_height):
            self.get_logger().info('rpicam-vid capture started.')
        else:
            self._init_opencv(camera_device, camera_index, frame_width, frame_height)

        # ── State ─────────────────────────────────────────────────────
        self._recent       = deque(maxlen=smoothing_window)
        self._target_found = False

        # ── Publishers ────────────────────────────────────────────────
        self._pub_detected = self.create_publisher(Bool,            '/phoenix/target_detected', 10)
        self._pub_prob     = self.create_publisher(Float32,         '/phoenix/detection_prob',   10)
        self._pub_frame    = self.create_publisher(CompressedImage, '/phoenix/camera_frame',     1)

        self.create_timer(0.1, self._inference_tick)   # 10 Hz

        self.get_logger().info(
            f'Camera detector ready | '
            f'backend={"rpicam-vid" if self._use_rpicam else "opencv"} | '
            f'threshold={self._threshold:.2f} | '
            f'positive_class="{self._positive_class}"'
        )

    # ── rpicam-vid backend ────────────────────────────────────────────

    def _init_rpicam(self, width: int, height: int) -> bool:
        rpicam = (shutil.which('rpicam-vid')
                  or '/usr/local/bin/rpicam-vid'
                  or '/usr/bin/rpicam-vid')
        if not Path(rpicam).exists():
            self.get_logger().warn(f'rpicam-vid not found at {rpicam}, falling back to OpenCV.')
            return False

        try:
            self._rpicam_proc = subprocess.Popen(
                [rpicam, '-t', '0', '--codec', 'mjpeg', '-o', '-',
                 '--width', str(width), '--height', str(height),
                 '--nopreview', '--flush'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception as e:
            self.get_logger().warn(f'rpicam-vid failed to start: {e}')
            return False

        self._use_rpicam = True
        t = threading.Thread(target=self._mjpeg_reader, daemon=True)
        t.start()
        return True

    def _mjpeg_reader(self):
        """Background thread: reads MJPEG stream from rpicam-vid stdout."""
        SOI = b'\xff\xd8'
        EOI = b'\xff\xd9'
        buf = b''
        while True:
            try:
                chunk = self._rpicam_proc.stdout.read(8192)
            except Exception:
                break
            if not chunk:
                break
            buf += chunk

            while True:
                start = buf.find(SOI)
                if start == -1:
                    buf = b''
                    break
                end = buf.find(EOI, start + 2)
                if end == -1:
                    buf = buf[start:]   # keep partial JPEG
                    break
                jpeg_bytes = buf[start: end + 2]
                buf = buf[end + 2:]

                arr   = np.frombuffer(jpeg_bytes, np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    # Keep only the latest frame; discard stale ones
                    try:
                        self._frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    self._frame_queue.put(frame)

    # ── OpenCV backend ────────────────────────────────────────────────

    def _init_opencv(self, camera_device: str, camera_index: int,
                     width: int, height: int):
        cam_src = camera_device if camera_device else camera_index
        self._cap = cv2.VideoCapture(cam_src)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera {cam_src}. '
                'Try: --ros-args -p camera_device:=/dev/videoN')
            sys.exit(1)
        self.get_logger().info(f'OpenCV VideoCapture opened: {cam_src}')

    # ── Frame read ────────────────────────────────────────────────────

    def _read_frame(self):
        if self._use_rpicam:
            try:
                frame = self._frame_queue.get(timeout=0.15)
                return frame, True
            except queue.Empty:
                return None, False
        else:
            ok, frame = self._cap.read()
            return frame, ok

    # ── Inference ─────────────────────────────────────────────────────

    def _inference_tick(self):
        frame, ok = self._read_frame()
        if not ok or frame is None:
            self.get_logger().debug('No frame yet.', throttle_duration_sec=5.0)
            return

        if self._video_writer:
            self._video_writer.write(frame)

        # Publish frame for dashboard (JPEG-compressed, quality 70)
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            img_msg = CompressedImage()
            img_msg.header.stamp  = self.get_clock().now().to_msg()
            img_msg.format        = 'jpeg'
            img_msg.data          = buf.tobytes()
            self._pub_frame.publish(img_msg)

        prob    = self._infer(frame)
        instant = prob >= self._threshold
        self._recent.append(1 if instant else 0)
        stable  = sum(self._recent) >= self._req_pos

        prob_msg      = Float32()
        prob_msg.data = float(prob)
        self._pub_prob.publish(prob_msg)

        det_msg      = Bool()
        det_msg.data = bool(stable)
        self._pub_detected.publish(det_msg)

        if stable and not self._target_found:
            self._target_found = True
            self.get_logger().info(
                f'★  TARGET DETECTED  ★  prob={prob:.3f}  votes={list(self._recent)}'
            )

    def _infer(self, frame_bgr: np.ndarray) -> float:
        gray    = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        resized = cv2.resize(gray, (self._img_size, self._img_size),
                             interpolation=cv2.INTER_AREA)
        x = np.expand_dims(np.expand_dims(resized.astype(np.float32), -1), 0)

        in_dtype = self._in_det['dtype']
        if in_dtype != np.float32:
            scale, zp = self._in_det['quantization']
            if scale != 0.0:
                x = np.clip(np.round(x / scale + zp),
                            np.iinfo(in_dtype).min,
                            np.iinfo(in_dtype).max).astype(in_dtype)
            else:
                x = x.astype(in_dtype)

        self._interp.set_tensor(self._in_det['index'], x)
        self._interp.invoke()
        y    = self._interp.get_tensor(self._out_det['index'])
        prob = float(np.squeeze(y))

        out_dtype = self._out_det['dtype']
        if out_dtype != np.float32:
            scale, zp = self._out_det['quantization']
            if scale != 0.0:
                prob = (prob - zp) * scale

        return float(np.clip(prob, 0.0, 1.0))

    # ── Cleanup ───────────────────────────────────────────────────────

    def destroy_node(self):
        if self._video_writer:
            self._video_writer.release()
            self.get_logger().info('Camera video saved.')
        if self._rpicam_proc:
            self._rpicam_proc.terminate()
        if self._cap:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
