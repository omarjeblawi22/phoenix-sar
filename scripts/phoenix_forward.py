#!/usr/bin/env python3
"""
Phoenix forward navigator — drives robot 3m forward with obstacle avoidance.
Waits for Nav2 to activate, reads current pose from TF, computes a goal
3m ahead in the robot's current heading, then sends it to Nav2.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener


DISTANCE = 3.0   # metres forward


class ForwardNav(Node):
    def __init__(self):
        super().__init__('phoenix_forward')
        self._ac   = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._tf   = Buffer()
        TransformListener(self._tf, self)
        self._sent = False
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Forward nav: waiting for Nav2 ({DISTANCE}m)...')

    def _tick(self):
        if self._sent:
            return
        if not self._ac.wait_for_server(timeout_sec=0.1):
            return
        try:
            t = self._tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        tx  = t.transform.translation.x
        ty  = t.transform.translation.y
        qz  = t.transform.rotation.z
        qw  = t.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        gx  = tx + DISTANCE * math.cos(yaw)
        gy  = ty + DISTANCE * math.sin(yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        self._sent = True
        self.get_logger().info(
            f'Forward goal: ({gx:.2f},{gy:.2f}), yaw={math.degrees(yaw):.1f}°')
        print(f'>> Forward nav: driving {DISTANCE}m to ({gx:.2f},{gy:.2f})',
              flush=True)
        future = self._ac.send_goal_async(goal)
        future.add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if handle.accepted:
            self.get_logger().info('Goal accepted — navigating forward...')
            handle.get_result_async().add_done_callback(self._on_done)
        else:
            self.get_logger().warn('Goal rejected — check map coverage ahead')
            print('>> Forward nav: goal rejected (obstacle or unmapped area)', flush=True)

    def _on_done(self, future):
        status = future.result().status
        if status == 4:   # SUCCEEDED
            self.get_logger().info('Forward navigation complete.')
            print('>> Forward nav: reached goal.', flush=True)
        else:
            self.get_logger().warn(f'Navigation ended with status {status}')
            print(f'>> Forward nav: ended (status {status})', flush=True)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ForwardNav())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
