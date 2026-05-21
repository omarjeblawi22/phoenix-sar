#!/usr/bin/env python3
"""
Phoenix circle patrol — drives robot in a circle while SLAM maps the area.
Waits for Nav2 to activate, computes 8 waypoints in a 1.2m-radius circle
around the start pose, then loops continuously until stopped.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


RADIUS  = 1.2   # metres
N_PTS   = 8     # waypoints around the circle


class CirclePatrol(Node):
    def __init__(self):
        super().__init__('phoenix_circle')
        self._ac  = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._tf  = Buffer()
        TransformListener(self._tf, self)
        self._wps   = []
        self._idx   = 0
        self._busy  = False
        self._ready = False
        self.create_timer(1.0, self._tick)
        self.get_logger().info('Circle patrol: waiting for Nav2...')

    def _tick(self):
        if self._ready or self._busy:
            return
        if not self._ac.wait_for_server(timeout_sec=0.1):
            return
        try:
            t = self._tf.lookup_transform('map', 'base_link', Time())
            sx = t.transform.translation.x
            sy = t.transform.translation.y
        except Exception:
            return
        self._wps = []
        for i in range(N_PTS):
            a = 2 * math.pi * i / N_PTS
            self._wps.append((sx + RADIUS * math.cos(a),
                               sy + RADIUS * math.sin(a),
                               a + math.pi / 2))
        self._ready = True
        self.get_logger().info(
            f'Circle patrol ready: {N_PTS} pts, radius={RADIUS}m, '
            f'start=({sx:.2f},{sy:.2f})')
        print(f'>> Circle patrol active — {N_PTS} waypoints, radius={RADIUS}m',
              flush=True)
        self._send_next()

    def _send_next(self):
        x, y, yaw = self._wps[self._idx % N_PTS]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self._busy = True
        self._idx += 1
        self.get_logger().info(
            f'Waypoint {self._idx}/{N_PTS}: ({x:.2f},{y:.2f})')
        future = self._ac.send_goal_async(goal)
        future.add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if handle.accepted:
            handle.get_result_async().add_done_callback(self._on_done)
        else:
            self.get_logger().warn('Waypoint rejected — skipping')
            self._busy = False
            self._send_next()

    def _on_done(self, future):
        self._busy = False
        self._send_next()   # immediately go to next waypoint (loops forever)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CirclePatrol())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
