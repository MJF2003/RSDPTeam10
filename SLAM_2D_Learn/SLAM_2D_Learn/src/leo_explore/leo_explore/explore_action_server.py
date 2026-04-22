#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from leo_explore_interfaces.action import Explore


class ExploreActionServer(Node):
    MODE_FORWARD = 'FORWARD'
    MODE_COMMIT_LEFT = 'COMMIT_LEFT'
    MODE_COMMIT_RIGHT = 'COMMIT_RIGHT'

    def __init__(self):
        super().__init__('leo_explore_action_server')

        self.cb_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()

        # ===== Parameters =====
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('forward_speed', 0.18)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('commit_forward_speed', 0.08)
        self.declare_parameter('min_forward_speed', 0.04)

        self.declare_parameter('turn_speed', 0.58)
        self.declare_parameter('emergency_turn_speed', 0.85)

        self.declare_parameter('safe_dist', 0.30)
        self.declare_parameter('slow_dist', 0.50)
        self.declare_parameter('emergency_dist', 0.16)
        self.declare_parameter('side_safe_dist', 0.24)

        self.declare_parameter('release_front_dist', 0.42)
        self.declare_parameter('release_side_dist', 0.30)

        self.declare_parameter('commit_min_time', 1.80)
        self.declare_parameter('commit_max_time', 5.00)
        self.declare_parameter('direction_deadband', 0.12)

        self.declare_parameter('robot_stop_timeout', 0.50)
        self.declare_parameter('angular_sign', -1.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.slow_speed = float(self.get_parameter('slow_speed').value)
        self.commit_forward_speed = float(self.get_parameter('commit_forward_speed').value)
        self.min_forward_speed = float(self.get_parameter('min_forward_speed').value)

        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.emergency_turn_speed = float(self.get_parameter('emergency_turn_speed').value)

        self.safe_dist = float(self.get_parameter('safe_dist').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.emergency_dist = float(self.get_parameter('emergency_dist').value)
        self.side_safe_dist = float(self.get_parameter('side_safe_dist').value)

        self.release_front_dist = float(self.get_parameter('release_front_dist').value)
        self.release_side_dist = float(self.get_parameter('release_side_dist').value)

        self.commit_min_time = float(self.get_parameter('commit_min_time').value)
        self.commit_max_time = float(self.get_parameter('commit_max_time').value)
        self.direction_deadband = float(self.get_parameter('direction_deadband').value)

        self.robot_stop_timeout = float(self.get_parameter('robot_stop_timeout').value)
        self.angular_sign = float(self.get_parameter('angular_sign').value)

        # ===== Publishers / Subscribers =====
        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
            callback_group=self.cb_group
        )

        # ===== Internal state =====
        self.front = 10.0
        self.front_left = 10.0
        self.front_right = 10.0
        self.left = 10.0
        self.right = 10.0
        self.left_open_score = 10.0
        self.right_open_score = 10.0

        self.scan_ready = False
        self.last_scan_time = self.get_clock().now()

        self.mode = self.MODE_FORWARD
        self.mode_start_time = self.get_clock().now()
        self.last_turn_sign = 1.0

        self.exploring_active = False
        self.active_goal_handle = None
        self.goal_start_time = None

        # ===== Control timer =====
        self.timer = self.create_timer(
            0.1,
            self.control_loop,
            callback_group=self.cb_group
        )

        # ===== Action server =====
        self._action_server = ActionServer(
            self,
            Explore,
            'explore',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Explore action server started')

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def reset_behavior_state(self):
        self.front = 10.0
        self.front_left = 10.0
        self.front_right = 10.0
        self.left = 10.0
        self.right = 10.0
        self.left_open_score = 10.0
        self.right_open_score = 10.0

        self.scan_ready = False
        self.last_scan_time = self.get_clock().now()

        self.mode = self.MODE_FORWARD
        self.mode_start_time = self.get_clock().now()
        self.last_turn_sign = 1.0

    def _elapsed_since_goal_start(self):
        if self.goal_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9

    def _make_result(self, success: bool, message: str) -> Explore.Result:
        result = Explore.Result()
        result.success = success
        result.message = message
        result.elapsed_time_sec = float(self._elapsed_since_goal_start())
        result.final_mode = self.mode
        return result

    def _angle_to_index(self, angle, angle_min, angle_increment, length):
        idx = int((angle - angle_min) / angle_increment)
        return max(0, min(idx, length - 1))

    def _sector_values(self, ranges, angle_min, angle_increment, deg_start, deg_end):
        if not ranges:
            return []

        start = math.radians(deg_start)
        end = math.radians(deg_end)
        i0 = self._angle_to_index(start, angle_min, angle_increment, len(ranges))
        i1 = self._angle_to_index(end, angle_min, angle_increment, len(ranges))
        if i0 > i1:
            i0, i1 = i1, i0

        values = []
        for i in range(i0, i1 + 1):
            r = ranges[i]
            if math.isfinite(r) and r > 0.03:
                values.append(min(r, 3.0))
        return values

    def _sector_min(self, ranges, angle_min, angle_increment, deg_start, deg_end):
        values = self._sector_values(ranges, angle_min, angle_increment, deg_start, deg_end)
        if not values:
            return 10.0
        return min(values)

    def _sector_mean(self, ranges, angle_min, angle_increment, deg_start, deg_end):
        values = self._sector_values(ranges, angle_min, angle_increment, deg_start, deg_end)
        if not values:
            return 3.0
        return sum(values) / len(values)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def _set_mode(self, mode):
        if self.mode != mode:
            self.mode = mode
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info(f'Switch mode -> {mode}')

    def _mode_elapsed(self):
        return (self.get_clock().now() - self.mode_start_time).nanoseconds / 1e9

    def _need_commit(self):
        return (
            self.front < self.safe_dist or
            self.front_left < self.side_safe_dist or
            self.front_right < self.side_safe_dist
        )

    def _choose_turn_sign(self):
        diff = self.left_open_score - self.right_open_score
        if diff > self.direction_deadband:
            return 1.0
        if diff < -self.direction_deadband:
            return -1.0
        return self.last_turn_sign

    def _release_left(self):
        return self.front > self.release_front_dist and self.front_left > self.release_side_dist

    def _release_right(self):
        return self.front > self.release_front_dist and self.front_right > self.release_side_dist

    def _publish_commit(self, sign):
        cmd = Twist()

        if self.front < self.emergency_dist:
            cmd.linear.x = 0.0
            turn = self.emergency_turn_speed
        else:
            cmd.linear.x = self.commit_forward_speed
            turn = self.turn_speed

        cmd.angular.z = self.angular_sign * sign * turn
        self.cmd_pub.publish(cmd)

    # -------------------------------------------------------------------------
    # ROS callbacks
    # -------------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        self.front = self._sector_min(ranges, msg.angle_min, msg.angle_increment, -15, 15)
        self.front_left = self._sector_min(ranges, msg.angle_min, msg.angle_increment, 15, 50)
        self.front_right = self._sector_min(ranges, msg.angle_min, msg.angle_increment, -50, -15)
        self.left = self._sector_min(ranges, msg.angle_min, msg.angle_increment, 50, 95)
        self.right = self._sector_min(ranges, msg.angle_min, msg.angle_increment, -95, -50)

        self.left_open_score = self._sector_mean(ranges, msg.angle_min, msg.angle_increment, 20, 100)
        self.right_open_score = self._sector_mean(ranges, msg.angle_min, msg.angle_increment, -100, -20)

        self.scan_ready = True
        self.last_scan_time = self.get_clock().now()

    # -------------------------------------------------------------------------
    # Action server callbacks
    # -------------------------------------------------------------------------
    def goal_callback(self, goal_request):
        with self.state_lock:
            if self.exploring_active:
                self.get_logger().warn('Rejecting goal: exploration already running')
                return GoalResponse.REJECT

        self.get_logger().info(
            f"Received goal: max_runtime_sec={goal_request.max_runtime_sec}, "
            f"run_until_cancelled={goal_request.run_until_cancelled}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self.state_lock:
            self.exploring_active = True
            self.active_goal_handle = goal_handle
            self.goal_start_time = self.get_clock().now()
            self.reset_behavior_state()

        self.get_logger().info('Explore goal accepted, exploration started')

        max_runtime = float(goal_handle.request.max_runtime_sec)
        run_until_cancelled = bool(goal_handle.request.run_until_cancelled)

        feedback_msg = Explore.Feedback()

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Explore goal canceled')
                    self.stop_robot()
                    goal_handle.canceled()
                    return self._make_result(False, 'Goal canceled')

                elapsed = self._elapsed_since_goal_start()

                feedback_msg.current_mode = self.mode
                feedback_msg.front = float(self.front)
                feedback_msg.left_open_score = float(self.left_open_score)
                feedback_msg.right_open_score = float(self.right_open_score)
                feedback_msg.elapsed_time_sec = float(elapsed)
                goal_handle.publish_feedback(feedback_msg)

                if (not run_until_cancelled) and max_runtime > 0.0 and elapsed >= max_runtime:
                    self.get_logger().info('Explore goal finished by timeout')
                    self.stop_robot()
                    goal_handle.succeed()
                    return self._make_result(True, 'Reached requested runtime')

                time.sleep(0.2)

            self.stop_robot()
            goal_handle.abort()
            return self._make_result(False, 'ROS shutdown')

        finally:
            with self.state_lock:
                self.exploring_active = False
                self.active_goal_handle = None
                self.goal_start_time = None

    # -------------------------------------------------------------------------
    # Control loop
    # -------------------------------------------------------------------------
    def control_loop(self):
        if not self.exploring_active:
            return

        if not self.scan_ready:
            self.stop_robot()
            return

        dt = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if dt > self.robot_stop_timeout:
            self.get_logger().warn('Laser scan timeout, stop robot.')
            self.stop_robot()
            return

        # Enter committed turn mode
        if self.mode == self.MODE_FORWARD and self._need_commit():
            sign = self._choose_turn_sign()
            self.last_turn_sign = sign
            if sign > 0.0:
                self._set_mode(self.MODE_COMMIT_LEFT)
            else:
                self._set_mode(self.MODE_COMMIT_RIGHT)

        if self.mode == self.MODE_COMMIT_LEFT:
            elapsed = self._mode_elapsed()
            if elapsed >= self.commit_min_time and self._release_left():
                self._set_mode(self.MODE_FORWARD)
            elif elapsed >= self.commit_max_time:
                self.mode_start_time = self.get_clock().now()
            self._publish_commit(1.0)
            return

        if self.mode == self.MODE_COMMIT_RIGHT:
            elapsed = self._mode_elapsed()
            if elapsed >= self.commit_min_time and self._release_right():
                self._set_mode(self.MODE_FORWARD)
            elif elapsed >= self.commit_max_time:
                self.mode_start_time = self.get_clock().now()
            self._publish_commit(-1.0)
            return

        # FORWARD mode
        cmd = Twist()

        if self.front < self.slow_dist:
            ratio = (self.front - self.safe_dist) / max(self.slow_dist - self.safe_dist, 0.05)
            ratio = max(0.0, min(1.0, ratio))
            cmd.linear.x = self.slow_speed + ratio * (self.forward_speed - self.slow_speed)
        else:
            cmd.linear.x = self.forward_speed

        balance = (self.front_left + 0.5 * self.left) - (self.front_right + 0.5 * self.right)
        balance = max(-1.0, min(1.0, balance))
        cmd.angular.z = self.angular_sign * (-0.22 * balance)

        if cmd.linear.x < self.min_forward_speed:
            cmd.linear.x = self.min_forward_speed

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = ExploreActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()