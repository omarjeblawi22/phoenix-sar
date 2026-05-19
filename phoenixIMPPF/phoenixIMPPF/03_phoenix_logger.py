#!/usr/bin/env python3
"""
PHOENIX Dataset Logger (ROS2 / rclpy)
=====================================

Runs on the Raspberry Pi 5. Reads the ESP32-S3 over a serial / USB-CDC link,
subscribes to `/map` (OccupancyGrid) and looks up `map -> base_link` from TF
to pin each FTM burst to a ground-truth SLAM pose at burst *start*.

Outputs in `output_dir/`:
    rtt.csv          one row per FTM burst with aggregated distance + pose
    trajectory.csv   high-rate SLAM pose log (independent of bursts)
    map.npy          latest OccupancyGrid as int8 array (ROS convention)
    map_meta.json    resolution + origin + dims
    csi.csv          optional, one row per CSI packet (subcarrier amps as '|'-joined string)

CSV schema — rtt.csv:
    t_sec, seq, n_frames, d_median_m, d_mad_m, rssi_median_dbm,
    pose_x, pose_y, pose_yaw, pose_ok, label

The IMPPF prototype consumes `t_sec, d_median_m` from this file, and the
trajectory CSV's `t_sec, x, y, yaw`. Map is loaded from `map.npy` + meta JSON.

Run:
    ros2 run phoenix_logger phoenix_logger \\
        --ros-args -p serial_port:=/dev/ttyACM0 \\
                  -p baud:=921600 \\
                  -p output_dir:=/home/pi/datasets/run01

Standalone (no ros2 run wrapper, useful for ad-hoc testing once on the path):
    python3 phoenix_logger.py --ros-args -p output_dir:=/tmp/run

Dependencies:
    rclpy, tf2_ros, nav_msgs, geometry_msgs, std_msgs (all standard ROS 2),
    pyserial, numpy
"""

import csv
import json
import threading
import time
from pathlib import Path

import numpy as np
import serial

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

C_LIGHT = 299_792_458.0  # m/s


def yaw_from_quat(q) -> float:
    """ZYX yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class PhoenixLogger(Node):
    def __init__(self):
        super().__init__("phoenix_logger")

        # ---------- Parameters ----------
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("output_dir", "/tmp/phoenix_run")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("save_csi", True)
        self.declare_parameter("traj_log_period_s", 0.05)  # 20 Hz
        self.declare_parameter("min_frames_for_burst", 4)
        self.declare_parameter("tf_lookup_timeout_s", 0.3)

        self.port = self.get_parameter("serial_port").value
        self.baud = int(self.get_parameter("baud").value)
        self.out = Path(self.get_parameter("output_dir").value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.save_csi = bool(self.get_parameter("save_csi").value)
        self.traj_period = float(self.get_parameter("traj_log_period_s").value)
        self.min_frames = int(self.get_parameter("min_frames_for_burst").value)
        self.tf_timeout = float(self.get_parameter("tf_lookup_timeout_s").value)

        # ---------- Output files ----------
        self.rtt_f = open(self.out / "rtt.csv", "w", newline="")
        self.rtt_w = csv.writer(self.rtt_f)
        self.rtt_w.writerow([
            "t_sec", "seq", "n_frames",
            "d_median_m", "d_mad_m", "rssi_median_dbm",
            "pose_x", "pose_y", "pose_yaw", "pose_ok", "label",
        ])

        self.traj_f = open(self.out / "trajectory.csv", "w", newline="")
        self.traj_w = csv.writer(self.traj_f)
        self.traj_w.writerow(["t_sec", "x", "y", "yaw"])

        self.csi_f = None
        self.csi_w = None
        if self.save_csi:
            self.csi_f = open(self.out / "csi.csv", "w", newline="")
            self.csi_w = csv.writer(self.csi_f)
            self.csi_w.writerow([
                "t_sec", "seq", "rssi_dbm", "noise_dbm",
                "n_sub", "amps_pipe", "label",
            ])

        # ---------- TF + map ----------
        self.tf_buf = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buf, self)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.have_map = False

        # ---------- Label override topic ----------
        # Optional: publishing a std_msgs/String on /phoenix/label is an
        # alternative to typing labels into the ESP32 serial console.
        self.create_subscription(String, "/phoenix/label", self.on_label_topic, 10)

        # ---------- Trajectory timer ----------
        self.create_timer(self.traj_period, self.tick_trajectory)

        # ---------- Burst aggregation state ----------
        self._lock = threading.Lock()
        self.burst_frames = {}     # seq -> list[(rtt_ps, rssi, label)]
        self.burst_tx_time = {}    # seq -> rclpy.time.Time captured locally
        self.cur_label = "UNKNOWN"
        self.last_seq_seen = -1
        self.fallback_counts = {"used_burst_start_as_tx": 0,
                                "pose_lookup_failed": 0}

        # ---------- Serial ----------
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {self.port}: {e}")
            raise
        self.get_logger().info(f"Serial open: {self.port} @ {self.baud}")
        self._stop = threading.Event()
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()

        # ---------- Periodic flush / health log ----------
        self.create_timer(5.0, self._health)

        self.get_logger().info(f"Logging to {self.out.resolve()}")

    # =================================================================
    # ROS callbacks
    # =================================================================
    def on_map(self, msg: OccupancyGrid):
        """Save the latest map every time it updates (cheap; map is small)."""
        H, W = msg.info.height, msg.info.width
        grid = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(H, W).copy()
        np.save(self.out / "map.npy", grid)
        meta = {
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "width": int(W),
            "height": int(H),
            "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
        }
        with open(self.out / "map_meta.json", "w") as f:
            json.dump(meta, f, indent=2)
        if not self.have_map:
            self.get_logger().info(
                f"Map: {W}x{H} @ {msg.info.resolution:.3f} m/cell "
                f"origin=({meta['origin_x']:.2f}, {meta['origin_y']:.2f})"
            )
            self.have_map = True

    def on_label_topic(self, msg: String):
        lbl = msg.data.strip()
        if lbl:
            with self._lock:
                self.cur_label = lbl
            self.get_logger().info(f"Label (topic) -> {lbl}")

    def tick_trajectory(self):
        """High-rate trajectory log — independent of bursts."""
        try:
            tf = self.tf_buf.lookup_transform(
                self.global_frame, self.robot_frame, Time(),
                timeout=Duration(seconds=0.01),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        self.traj_w.writerow([
            f"{t:.6f}",
            f"{tf.transform.translation.x:.4f}",
            f"{tf.transform.translation.y:.4f}",
            f"{yaw_from_quat(tf.transform.rotation):.4f}",
        ])

    def _lookup_pose_at(self, when: Time):
        """Look up TF at `when`, falling back to latest if past the buffer."""
        for stamp in (when, Time()):
            try:
                tf: TransformStamped = self.tf_buf.lookup_transform(
                    self.global_frame, self.robot_frame, stamp,
                    timeout=Duration(seconds=self.tf_timeout),
                )
                t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                return (
                    t,
                    float(tf.transform.translation.x),
                    float(tf.transform.translation.y),
                    yaw_from_quat(tf.transform.rotation),
                    True,
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue
        return (0.0, 0.0, 0.0, 0.0, False)

    # =================================================================
    # Serial parsing
    # =================================================================
    def _serial_loop(self):
        buf = b""
        while not self._stop.is_set() and rclpy.ok():
            try:
                data = self.ser.read(8192)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.5)
                continue
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    self._handle_line(line.decode("utf-8", errors="ignore").strip())
                except Exception as e:
                    self.get_logger().warning(f"Parse error on '{line!r}': {e}")

    def _handle_line(self, line: str):
        if not line or line.startswith("#"):
            return
        parts = line.split(",")
        tag = parts[0]
        now = self.get_clock().now()

        if tag == "BURST_TX":
            # Preferred path (new firmware): BURST_TX,seq,mcu_us,label
            if len(parts) >= 4:
                seq = int(parts[1])
                with self._lock:
                    self.burst_tx_time[seq] = now
                    self.cur_label = parts[3]

        elif tag == "BURST_START":
            # Old firmware (no BURST_TX) OR new firmware (then this just confirms).
            # Layouts seen:
            #   BURST_START,seq,n_frames,label                    (old)
            #   BURST_START,seq,mcu_us,n_frames,label             (new, if you applied §1.1 patch)
            if len(parts) < 4:
                return
            seq = int(parts[1])
            label = parts[-1]
            with self._lock:
                if seq not in self.burst_tx_time:
                    # Fallback: timestamp at BURST_START arrival. Less accurate
                    # but better than nothing.
                    self.burst_tx_time[seq] = now
                    self.fallback_counts["used_burst_start_as_tx"] += 1
                self.cur_label = label

        elif tag == "FTM_F":
            # FTM_F,seq,frame_idx,rtt_ps,t1,t2,t3,t4,rssi,label
            if len(parts) < 10:
                return
            try:
                seq = int(parts[1])
                rtt_ps = int(parts[3])
                rssi = int(parts[8])
                label = parts[9]
            except ValueError:
                return
            with self._lock:
                self.burst_frames.setdefault(seq, []).append((rtt_ps, rssi, label))
                # Flush any older bursts: once we see frames for seq N, anything
                # < N can be considered complete.
                if seq > self.last_seq_seen:
                    to_flush = [s for s in self.burst_frames if s < seq]
                    self.last_seq_seen = seq
                else:
                    to_flush = []
            for s in to_flush:
                self._flush_burst(s)

        elif tag == "CSI" and self.save_csi:
            # CSI,seq,rssi,noise,n_sub,amp0,...,ampN-1,label
            if len(parts) < 6:
                return
            try:
                seq = int(parts[1])
                rssi = int(parts[2])
                noise = int(parts[3])
                n_sub = int(parts[4])
            except ValueError:
                return
            amps = parts[5:5 + n_sub]
            label = parts[-1] if len(parts) > 5 + n_sub else self.cur_label
            t_now = now.nanoseconds * 1e-9
            self.csi_w.writerow([
                f"{t_now:.6f}", seq, rssi, noise, n_sub,
                "|".join(amps), label,
            ])

        elif tag == "LABEL":
            if len(parts) >= 2:
                with self._lock:
                    self.cur_label = parts[1]
                self.get_logger().info(f"Label (serial) -> {parts[1]}")

    def _flush_burst(self, seq: int):
        with self._lock:
            frames = self.burst_frames.pop(seq, [])
            tx_time = self.burst_tx_time.pop(seq, None)
        if not frames or tx_time is None:
            return
        if len(frames) < self.min_frames:
            return

        rtt_ps = np.array([f[0] for f in frames], dtype=np.float64)
        rssi = np.array([f[1] for f in frames], dtype=np.float64)
        label = frames[0][2]

        # Picoseconds -> metres, PDF eq. (1)
        d = rtt_ps * C_LIGHT / (2.0 * 1e12)

        # Reject obvious garbage (negative or absurd) before aggregating.
        valid = (d > 0.0) & (d < 100.0)
        if valid.sum() < self.min_frames:
            return
        d = d[valid]
        rssi = rssi[valid]

        d_med = float(np.median(d))
        # MAD scaled to a normal-equivalent σ. Useful as a per-burst confidence.
        d_mad = float(np.median(np.abs(d - d_med)) * 1.4826)
        rssi_med = float(np.median(rssi))

        # Pin to the SLAM pose at burst start.
        t, x, y, yaw, ok = self._lookup_pose_at(tx_time)
        if not ok:
            self.fallback_counts["pose_lookup_failed"] += 1
            t = tx_time.nanoseconds * 1e-9  # at least record when MCU sent

        self.rtt_w.writerow([
            f"{t:.6f}", seq, len(frames),
            f"{d_med:.4f}", f"{d_mad:.4f}", f"{rssi_med:.1f}",
            f"{x:.4f}", f"{y:.4f}", f"{yaw:.4f}",
            int(ok), label,
        ])
        self.rtt_f.flush()

    # =================================================================
    # Health / shutdown
    # =================================================================
    def _health(self):
        try:
            sz = (self.out / "rtt.csv").stat().st_size
        except FileNotFoundError:
            sz = 0
        with self._lock:
            pending = len(self.burst_frames)
            counts = dict(self.fallback_counts)
        self.get_logger().info(
            f"rtt.csv={sz}B pending_bursts={pending} "
            f"fallbacks={counts} map={'Y' if self.have_map else 'N'}"
        )

    def destroy_node(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass
        for f in (self.rtt_f, self.traj_f, self.csi_f):
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PhoenixLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
