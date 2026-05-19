#!/usr/bin/env python3
"""
phoenix_explorer.py — PHOENIX SAR Autonomous Exploration Node
=============================================================

Deploys the robot into an unknown area and:
  1. Records the start pose (map -> base_link TF at launch).
  2. Continuously finds frontier cells in the live SLAM map and sends
     the nearest unexplored frontier as a Nav2 NavigateToPose goal.
  3. After reaching each frontier, spins 360° in place so the camera
     gets a full panoramic scan for the target (spin_scan=True, default).
  4. After `explore_duration` seconds (fallback): stops, saves map,
     computes and publishes the shortest path start → final.

Camera integration:
  Subscribes to /phoenix/target_detected (std_msgs/Bool).
  When a stable True arrives, _mission_complete() is called immediately:
  robot stops (including any active spin), map is saved, and the shortest
  path start → target is computed. explore_duration is a fallback timeout.

Topics subscribed:
  /phoenix/target_detected (std_msgs/Bool) — from camera_detector_node

Topics published:
  /phoenix/shortest_path  (nav_msgs/Path)            — optimal start→target path
  /phoenix/start_pose     (geometry_msgs/PoseStamped) — start position marker
  /phoenix/final_pose     (geometry_msgs/PoseStamped) — target position marker
  /phoenix/status         (std_msgs/String)           — WAITING/EXPLORING/SPINNING/TARGET_FOUND/MISSION_COMPLETE

Actions used:
  navigate_to_pose     (nav2_msgs/action/NavigateToPose)
  compute_path_to_pose (nav2_msgs/action/ComputePathToPose)
  spin                 (nav2_msgs/action/Spin)

Parameters:
  explore_duration     Fallback timeout in seconds (default 3600)
  goal_timeout_s       Cancel nav goal if stuck this long (default 20)
  min_goal_dist_m      Ignore frontiers closer than this (default 0.3)
  max_goal_dist_m      Ignore frontiers farther than this (default 2.0)
  min_frontier_cells   Minimum cluster size (default 3)
  spin_scan            Spin 360° at each frontier to scan for target (default True)
  spin_yaw             How many radians to spin (default 6.28 = 360°)
  map_save_path        Path prefix for saved map files
"""

import math
import time
import subprocess

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from std_msgs.msg import Bool, String

import tf2_ros


class PhoenixExplorer(Node):

    def __init__(self):
        super().__init__('phoenix_explorer')

        # ---------- Parameters ----------
        self.declare_parameter('explore_duration',    3600.0)
        self.declare_parameter('goal_timeout_s',        20.0)
        self.declare_parameter('min_goal_dist_m',        0.3)
        self.declare_parameter('max_goal_dist_m',        2.0)
        self.declare_parameter('min_frontier_cells',       3)
        self.declare_parameter('fixed_frame',           'map')
        self.declare_parameter('robot_frame',      'base_link')
        self.declare_parameter('map_save_path',
                               '/home/phoenix/ros2_ws/maps/exploration_run')
        self.declare_parameter('spin_scan',             True)
        self.declare_parameter('spin_yaw',         math.pi * 2.0)  # 360°

        self.explore_duration   = self.get_parameter('explore_duration').value
        self.goal_timeout_s     = self.get_parameter('goal_timeout_s').value
        self.min_goal_dist_m    = self.get_parameter('min_goal_dist_m').value
        self.max_goal_dist_m    = self.get_parameter('max_goal_dist_m').value
        self.min_frontier_cells = self.get_parameter('min_frontier_cells').value
        self.fixed_frame        = self.get_parameter('fixed_frame').value
        self.robot_frame        = self.get_parameter('robot_frame').value
        self.map_save_path      = self.get_parameter('map_save_path').value
        self.spin_scan          = bool(self.get_parameter('spin_scan').value)
        self.spin_yaw           = float(self.get_parameter('spin_yaw').value)

        # ---------- State ----------
        self.start_pose          = None
        self.final_pose          = None
        self.explore_start_time  = None
        self.goal_sent_time      = None
        self.current_map         = None
        self.mission_done        = False
        self._active_goal_handle = None
        self._recent_goals       = []
        self._target_detected    = False
        self._spinning           = False
        self._spin_goal_handle   = None

        # ---------- TF ----------
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=60.0))
        self.tf_lst = tf2_ros.TransformListener(self.tf_buf, self)

        # ---------- Action clients ----------
        self._nav_client  = ActionClient(self, NavigateToPose,    'navigate_to_pose')
        self._path_client = ActionClient(self, ComputePathToPose,  'compute_path_to_pose')
        self._spin_client = ActionClient(self, Spin,               'spin')

        # ---------- Subscribers ----------
        map_qos = QoSProfile(depth=1)
        map_qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(Bool, '/phoenix/target_detected',
                                 self._on_target_detected, 10)

        # ---------- Publishers ----------
        self.path_pub       = self.create_publisher(Path,        '/phoenix/shortest_path', 10)
        self.start_pose_pub = self.create_publisher(PoseStamped, '/phoenix/start_pose',    10)
        self.final_pose_pub = self.create_publisher(PoseStamped, '/phoenix/final_pose',    10)
        self.status_pub     = self.create_publisher(String,      '/phoenix/status',        10)

        # ---------- Timers ----------
        self.create_timer(1.0,  self._exploration_tick)
        self.create_timer(5.0,  self._republish_markers)
        self.create_timer(30.0, self._periodic_save_map)

        self.get_logger().info(
            f"Phoenix Explorer ready — fallback timeout {self.explore_duration:.0f} s | "
            f"spin_scan={'ON' if self.spin_scan else 'OFF'} | "
            f"Waiting for SLAM map and Nav2..."
        )
        self._publish_status("WAITING")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        self.current_map = msg

    def _on_target_detected(self, msg: Bool):
        if msg.data and not self._target_detected and not self.mission_done:
            self._target_detected = True
            self.get_logger().info('★  Camera confirmed target — stopping exploration!')
            self._publish_status("TARGET_FOUND")
            # Cancel spin immediately so mission_complete can record the pose
            if self._spin_goal_handle is not None:
                self._spin_goal_handle.cancel_goal_async()
                self._spin_goal_handle = None
            self._spinning = False

    def _republish_markers(self):
        if self.start_pose is not None:
            self.start_pose_pub.publish(self.start_pose)
        if self.final_pose is not None:
            self.final_pose_pub.publish(self.final_pose)

    # ------------------------------------------------------------------
    # Main exploration tick  (1 Hz)
    # ------------------------------------------------------------------

    def _exploration_tick(self):
        if self.mission_done:
            return

        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            self.get_logger().warn("Waiting for map → base_link TF...",
                                   throttle_duration_sec=5.0)
            return

        # Record start pose once (requires first map + Nav2 ready)
        if self.start_pose is None:
            if self.current_map is None:
                self.get_logger().info("Waiting for first /map...",
                                       throttle_duration_sec=5.0)
                return
            if not self._nav_client.wait_for_server(timeout_sec=0.5):
                self.get_logger().info("Waiting for Nav2 navigate_to_pose...",
                                       throttle_duration_sec=5.0)
                return
            self.start_pose = robot_pose
            self.explore_start_time = time.time()
            self.start_pose_pub.publish(self.start_pose)
            self.get_logger().info(
                f"Start pose recorded: ({robot_pose.pose.position.x:.3f}, "
                f"{robot_pose.pose.position.y:.3f})"
            )
            self._publish_status("EXPLORING")

        # Target detection is the PRIMARY goal — stop everything immediately
        if self._target_detected:
            self._mission_complete(robot_pose)
            return

        # Fallback timeout
        elapsed = time.time() - self.explore_start_time
        if elapsed >= self.explore_duration:
            self._mission_complete(robot_pose)
            return

        # Cancel nav goal if it has been running too long (robot stuck)
        if (self._active_goal_handle is not None
                and self.goal_sent_time is not None
                and (time.time() - self.goal_sent_time) > self.goal_timeout_s):
            self.get_logger().warn("Goal timed out — cancelling and finding new frontier.")
            self._cancel_nav_goal()

        # Send next goal only when idle (not navigating AND not spinning)
        if self._active_goal_handle is None and not self._spinning:
            self._send_next_goal(robot_pose)

        # Status log
        state = ('spinning' if self._spinning
                 else ('navigating' if self._active_goal_handle else 'idle'))
        self.get_logger().info(
            f"[{elapsed:.0f}/{self.explore_duration:.0f}s] "
            f"pos=({robot_pose.pose.position.x:.2f},{robot_pose.pose.position.y:.2f}) "
            f"state={state}"
        )

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def _find_frontiers(self):
        if self.current_map is None:
            return []
        info = self.current_map.info
        W, H = info.width, info.height
        if W == 0 or H == 0:
            return []

        grid    = np.array(self.current_map.data, dtype=np.int8).reshape(H, W)
        free    = (grid >= 0) & (grid <= 50)
        unknown = (grid < 0)

        unk_pad  = np.pad(unknown, 1, constant_values=False)
        frontier = free & (
            unk_pad[0:H,   1:W+1] |
            unk_pad[2:H+2, 1:W+1] |
            unk_pad[1:H+1, 0:W  ] |
            unk_pad[1:H+1, 2:W+2]
        )

        rows, cols = np.where(frontier)
        if len(rows) < self.min_frontier_cells:
            return []

        res = info.resolution
        ox  = info.origin.position.x
        oy  = info.origin.position.y
        wx  = cols.astype(float) * res + ox + res / 2.0
        wy  = rows.astype(float) * res + oy + res / 2.0

        if len(wx) > 800:
            idx = np.random.choice(len(wx), 800, replace=False)
            wx, wy = wx[idx], wy[idx]

        return list(zip(wx.tolist(), wy.tolist()))

    def _select_goal(self, robot_pose, frontiers):
        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y

        scored = []
        for (wx, wy) in frontiers:
            dist = math.hypot(wx - rx, wy - ry)
            if any(math.hypot(wx - gx, wy - gy) < 0.3 for gx, gy in self._recent_goals):
                continue
            if dist >= self.min_goal_dist_m:
                scored.append((dist, wx, wy))

        scored.sort()
        preferred = [(d, x, y) for d, x, y in scored if d <= self.max_goal_dist_m]
        candidates = preferred if preferred else scored[:5]

        if candidates:
            _, wx, wy = candidates[0]
            return (wx, wy)

        return self._random_free_goal(robot_pose)

    def _random_free_goal(self, robot_pose):
        if self.current_map is None:
            return None
        info = self.current_map.info
        grid = np.array(self.current_map.data, dtype=np.int8).reshape(
            info.height, info.width)
        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y
        rng = np.random.default_rng()
        for _ in range(20):
            angle = rng.uniform(0, 2 * math.pi)
            dist  = rng.uniform(1.0, 2.0)
            wx    = rx + dist * math.cos(angle)
            wy    = ry + dist * math.sin(angle)
            col = int((wx - info.origin.position.x) / info.resolution)
            row = int((wy - info.origin.position.y) / info.resolution)
            if 0 <= row < info.height and 0 <= col < info.width:
                if 0 <= grid[row, col] <= 50:
                    return (wx, wy)
        return None

    # ------------------------------------------------------------------
    # Nav2 goal management
    # ------------------------------------------------------------------

    def _send_next_goal(self, robot_pose):
        frontiers = self._find_frontiers()
        goal_pos  = self._select_goal(robot_pose, frontiers)

        if goal_pos is None:
            self.get_logger().warn(
                "No explorable frontier found — map may be fully explored.",
                throttle_duration_sec=10.0
            )
            return

        wx, wy = goal_pos
        self._recent_goals.append((wx, wy))
        if len(self._recent_goals) > 10:
            self._recent_goals.pop(0)

        self._send_nav_goal(wx, wy)

    def _send_nav_goal(self, x: float, y: float, yaw: float = 0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = self.fixed_frame
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = x
        goal_msg.pose.pose.position.y    = y
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_accepted_cb)
        self.goal_sent_time = time.time()
        self.get_logger().info(f"→ Frontier goal: ({x:.2f}, {y:.2f})")

    def _goal_accepted_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Nav2 rejected goal — will retry next tick.")
            self._active_goal_handle = None
            return
        self._active_goal_handle = gh
        gh.get_result_async().add_done_callback(self._goal_done_cb)

    def _goal_done_cb(self, future):
        wrapped = future.result()
        self._active_goal_handle = None
        # Only spin after a successful arrival — not on cancellation/abort
        if (wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and not self.mission_done
                and not self._target_detected
                and self.spin_scan):
            self._start_spin_scan()

    def _cancel_nav_goal(self):
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    # ------------------------------------------------------------------
    # Spin scan — 360° panoramic camera scan at each frontier
    # ------------------------------------------------------------------

    def _start_spin_scan(self):
        if not self._spin_client.server_is_ready():
            self.get_logger().warn("Spin behavior server not ready — skipping scan.")
            return
        self._spinning = True
        goal = Spin.Goal()
        goal.target_yaw = self.spin_yaw
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(self._spin_accepted_cb)
        self.get_logger().info(
            f"Spinning {math.degrees(self.spin_yaw):.0f}° to scan for target..."
        )
        self._publish_status("SPINNING")

    def _spin_accepted_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Spin behavior rejected — moving to next frontier.")
            self._spinning = False
            self._publish_status("EXPLORING")
            return
        self._spin_goal_handle = gh
        gh.get_result_async().add_done_callback(self._spin_done_cb)

    def _spin_done_cb(self, future):
        self._spinning = False
        self._spin_goal_handle = None
        if not self.mission_done:
            self._publish_status("EXPLORING")

    # ------------------------------------------------------------------
    # Mission complete
    # ------------------------------------------------------------------

    def _mission_complete(self, robot_pose):
        self.mission_done = True
        self._cancel_nav_goal()
        if self._spin_goal_handle is not None:
            self._spin_goal_handle.cancel_goal_async()
            self._spin_goal_handle = None
        self._spinning = False

        self.final_pose = robot_pose
        self.final_pose_pub.publish(self.final_pose)

        elapsed = time.time() - self.explore_start_time
        stop_reason = "TARGET FOUND" if self._target_detected else "TIMEOUT"
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  MISSION COMPLETE  ({elapsed:.1f} s) — {stop_reason}\n"
            f"  Start  : ({self.start_pose.pose.position.x:.3f}, "
            f"{self.start_pose.pose.position.y:.3f})\n"
            f"  Target : ({robot_pose.pose.position.x:.3f}, "
            f"{robot_pose.pose.position.y:.3f})\n"
            f"{'='*60}"
        )
        self._publish_status("MISSION_COMPLETE")

        self._save_map()
        self.create_timer(3.0, self._compute_return_path)

    def _save_map(self):
        self.get_logger().info(f"Saving map to {self.map_save_path}.*")
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', self.map_save_path,
                 '--ros-args', '-p', 'save_map_timeout:=5.0'],
                capture_output=True, text=True, timeout=20
            )
            if result.returncode == 0:
                self.get_logger().info("Map saved successfully.")
            else:
                self.get_logger().error(f"map_saver_cli failed:\n{result.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("map_saver_cli timed out.")
        except Exception as e:
            self.get_logger().error(f"Map save exception: {e}")

    # ------------------------------------------------------------------
    # Return path computation
    # ------------------------------------------------------------------

    def _compute_return_path(self):
        if self.start_pose is None or self.final_pose is None:
            return
        if not self._path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("compute_path_to_pose server not available.")
            return

        goal = ComputePathToPose.Goal()
        goal.start      = self.start_pose
        goal.goal       = self.final_pose
        goal.use_start  = True
        goal.planner_id = ''

        future = self._path_client.send_goal_async(goal)
        future.add_done_callback(self._path_accepted_cb)
        self.get_logger().info("Computing shortest path: start → target...")

    def _path_accepted_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Path request rejected by Nav2 planner.")
            return
        gh.get_result_async().add_done_callback(self._path_result_cb)

    def _path_result_cb(self, future):
        result = future.result().result
        path: Path = result.path

        if not path.poses:
            self.get_logger().error(
                "Planner returned an empty path. "
                "Try manually sending a Nav2 goal from start → target in RViz."
            )
            return

        length = sum(
            math.hypot(
                path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,
                path.poses[i].pose.position.y - path.poses[i-1].pose.position.y
            )
            for i in range(1, len(path.poses))
        )

        self.path_pub.publish(path)
        self.get_logger().info(
            f"Shortest path published to /phoenix/shortest_path\n"
            f"  Waypoints : {len(path.poses)}\n"
            f"  Length    : {length:.2f} m\n"
            f"  View in RViz: Add → Path → /phoenix/shortest_path"
        )
        self._publish_status("MISSION_COMPLETE")
        self.create_timer(5.0, lambda: self.path_pub.publish(path))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_robot_pose(self):
        try:
            tf = self.tf_buf.lookup_transform(
                self.fixed_frame, self.robot_frame,
                Time(), timeout=Duration(seconds=0.3)
            )
        except Exception:
            return None

        p = PoseStamped()
        p.header.frame_id = self.fixed_frame
        p.header.stamp    = tf.header.stamp
        t = tf.transform.translation
        r = tf.transform.rotation
        p.pose.position.x  = t.x
        p.pose.position.y  = t.y
        p.pose.position.z  = t.z
        p.pose.orientation = r
        return p

    def _periodic_save_map(self):
        if self.explore_start_time is not None and not self.mission_done:
            self._save_map()

    def destroy_node(self):
        if self.explore_start_time is not None and not self.mission_done:
            self.get_logger().info('Saving map before shutdown...')
            self._save_map()
        super().destroy_node()

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PhoenixExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
