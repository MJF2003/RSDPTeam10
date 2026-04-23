import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rover_interface.action import NavigateToPos


class NavigationStub(Node):
    def __init__(self):
        super().__init__("navigation_stub")

        self.approach_threshold_m = float(
            self.declare_parameter("approach_threshold_m", 0.5).value
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
        odom_timeout_sec = float(self.declare_parameter("odom_timeout_sec", 1.0).value)
        self.pose_timeout_sec = float(
            self.declare_parameter("ground_truth_timeout_sec", odom_timeout_sec).value
        )
        self.goal_timeout_sec = float(
            self.declare_parameter("goal_timeout_sec", 45.0).value
        )
        self.control_period_sec = float(
            self.declare_parameter("control_period_sec", 0.05).value
        )
        self.goal_frame = str(self.declare_parameter("goal_frame", "map").value)

        self._latest_pose_lock = threading.Lock()
        self._latest_pose: Optional[tuple[float, float, float]] = None
        self._latest_pose_time = None
        self._pose_topic_logged = False
        self._empty_frame_warned = False
        self._unexpected_frame_warned = False

        self._pose_callback_group = ReentrantCallbackGroup()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            PoseArray,
            "/sim/rover_pose",
            self._ground_truth_callback,
            10,
            callback_group=self._pose_callback_group,
        )

        self.action_server = ActionServer(
            self,
            NavigateToPos,
            "/navigate_to_pos",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"Navigation stub ready on /navigate_to_pos "
            f"(approach_threshold_m={self.approach_threshold_m:.2f}, "
            f"goal_frame='{self.goal_frame}')"
        )

    def goal_callback(self, goal_request: NavigateToPos.Goal):
        frame_id = goal_request.target_pose.header.frame_id.strip() or self.goal_frame
        target = goal_request.target_pose.pose.position

        if frame_id != self.goal_frame:
            self.get_logger().warn(
                f"Rejecting goal with unsupported frame_id '{frame_id}'. "
                f"Expected '{self.goal_frame}'."
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
        goal_pose = goal_handle.request.target_pose.pose
        target = goal_pose.position
        target_x = float(target.x)
        target_y = float(target.y)
        goal_yaw = self._yaw_from_quaternion(
            goal_pose.orientation.x,
            goal_pose.orientation.y,
            goal_pose.orientation.z,
            goal_pose.orientation.w,
        )

        self.get_logger().info(
            "Executing navigate goal to target at "
            f"({target_x:.3f}, {target_y:.3f}) with yaw {goal_yaw:.3f} rad"
        )

        result = NavigateToPos.Result()
        feedback = NavigateToPos.Feedback()
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
                result.message = "No recent global pose received."
                return result

            x, y, yaw = pose
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            goal_yaw_error = self._normalize_angle(goal_yaw - yaw)

            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)

            if (
                distance <= self.approach_threshold_m
                and abs(goal_yaw_error) <= self.heading_tolerance_rad
            ):
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

            if distance > self.approach_threshold_m:
                desired_heading = math.atan2(dy, dx)
            else:
                desired_heading = goal_yaw
            heading_error = self._normalize_angle(desired_heading - yaw)

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

    def _ground_truth_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        pose_msg = msg.poses[0]
        frame_id = msg.header.frame_id.strip()
        if not self._pose_topic_logged:
            self.get_logger().info("Using rover global pose feed from /sim/rover_pose")
            self._pose_topic_logged = True

        if not frame_id and not self._empty_frame_warned:
            self.get_logger().warning(
                f"/sim/rover_pose arrived with an empty frame_id; "
                f"treating it as '{self.goal_frame}'."
            )
            self._empty_frame_warned = True
        elif frame_id and frame_id not in {self.goal_frame, "world"}:
            if not self._unexpected_frame_warned:
                self.get_logger().warning(
                    f"/sim/rover_pose arrived in frame '{frame_id}'; "
                    f"treating it as '{self.goal_frame}'."
                )
                self._unexpected_frame_warned = True

        yaw = self._yaw_from_quaternion(
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w,
        )

        with self._latest_pose_lock:
            self._latest_pose = (
                float(pose_msg.position.x),
                float(pose_msg.position.y),
                float(yaw),
            )
            self._latest_pose_time = self.get_clock().now()

    def _get_current_pose(self):
        with self._latest_pose_lock:
            if self._latest_pose is None or self._latest_pose_time is None:
                return None

            pose = self._latest_pose
            pose_time = self._latest_pose_time

        age_sec = (self.get_clock().now() - pose_time).nanoseconds / 1e9
        if age_sec > self.pose_timeout_sec:
            return None

        return pose

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
