import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rover_interface.action import NavigateToBlock


class NavigationStub(Node):
    def __init__(self):
        super().__init__("navigation_stub")

        self.approach_threshold_m = float(
            self.declare_parameter("approach_threshold_m", 0.3).value
        )
        self.max_linear_speed_mps = float(
            self.declare_parameter("max_linear_speed_mps", 0.30).value
        )
        self.max_angular_speed_radps = float(
            self.declare_parameter("max_angular_speed_radps", 1.20).value
        )
        self.linear_kp = float(self.declare_parameter("linear_kp", 0.50).value)
        self.angular_kp = float(self.declare_parameter("angular_kp", 1.50).value)
        self.heading_tolerance_rad = float(
            self.declare_parameter("heading_tolerance_rad", 0.20).value
        )
        self.odom_timeout_sec = float(
            self.declare_parameter("odom_timeout_sec", 1.0).value
        )
        self.goal_timeout_sec = float(
            self.declare_parameter("goal_timeout_sec", 45.0).value
        )
        self.control_period_sec = float(
            self.declare_parameter("control_period_sec", 0.05).value
        )

        self._latest_odom_lock = threading.Lock()
        self._latest_odom: Optional[Odometry] = None
        self._latest_odom_time = None

        self._odom_callback_group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            10,
            callback_group=self._odom_callback_group,
        )

        self.action_server = ActionServer(
            self,
            NavigateToBlock,
            "/navigate_to_block",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"Navigation stub ready on /navigate_to_block "
            f"(approach_threshold_m={self.approach_threshold_m:.2f})"
        )

    def goal_callback(self, goal_request: NavigateToBlock.Goal):
        frame_id = goal_request.target_block.position.header.frame_id.strip()
        target = goal_request.target_block.position.point

        if frame_id and frame_id != "odom":
            self.get_logger().warn(
                f"Rejecting goal with unsupported frame_id '{frame_id}'. "
                "Expected empty or 'odom'."
            )
            return GoalResponse.REJECT

        if not math.isfinite(target.x) or not math.isfinite(target.y):
            self.get_logger().warn("Rejecting goal with non-finite target coordinates.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target_block.position.point
        target_x = float(target.x)
        target_y = float(target.y)

        self.get_logger().info(
            f"Executing navigate goal to block id={goal_handle.request.target_block.id} "
            f"at ({target_x:.3f}, {target_y:.3f})"
        )

        result = NavigateToBlock.Result()
        feedback = NavigateToBlock.Feedback()
        start_time = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                goal_handle.canceled()
                result.success = False
                result.final_distance = float("inf")
                result.message = "Goal canceled."
                return result

            pose = self._get_current_pose()
            if pose is None:
                self._publish_stop()
                goal_handle.abort()
                result.success = False
                result.final_distance = float("inf")
                result.message = "No recent /odom received."
                return result

            x, y, yaw = pose
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)

            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)

            if distance <= self.approach_threshold_m:
                self._publish_stop()
                goal_handle.succeed()
                result.success = True
                result.final_distance = float(distance)
                result.message = "Reached approach threshold."
                return result

            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_sec >= self.goal_timeout_sec:
                self._publish_stop()
                goal_handle.abort()
                result.success = False
                result.final_distance = float(distance)
                result.message = "Timed out before reaching goal."
                return result

            target_heading = math.atan2(dy, dx)
            heading_error = self._normalize_angle(target_heading - yaw)

            twist = Twist()
            twist.angular.z = self._clamp(
                self.angular_kp * heading_error,
                -self.max_angular_speed_radps,
                self.max_angular_speed_radps,
            )

            if abs(heading_error) < self.heading_tolerance_rad:
                twist.linear.x = self._clamp(
                    self.linear_kp * distance,
                    0.0,
                    self.max_linear_speed_mps,
                )

            self.cmd_vel_pub.publish(twist)
            time.sleep(self.control_period_sec)

        self._publish_stop()
        result.success = False
        result.final_distance = float("inf")
        result.message = "ROS shutdown during goal execution."
        return result

    def _odom_callback(self, msg: Odometry):
        with self._latest_odom_lock:
            self._latest_odom = msg
            self._latest_odom_time = self.get_clock().now()

    def _get_current_pose(self):
        with self._latest_odom_lock:
            if self._latest_odom is None or self._latest_odom_time is None:
                return None

            odom = self._latest_odom
            odom_time = self._latest_odom_time

        age_sec = (self.get_clock().now() - odom_time).nanoseconds / 1e9
        if age_sec > self.odom_timeout_sec:
            return None

        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        yaw = self._yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        return position.x, position.y, yaw

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def destroy_node(self):
        self._publish_stop()
        self.action_server.destroy()
        super().destroy_node()


def main():
    try:
        rclpy.init()
        node = NavigationStub()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
