import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rover_interface.action import ManipulateBlock


class ManipulationStub(Node):
    def __init__(self):
        super().__init__("manipulation_stub")

        self.manipulation_radius_m = float(
            self.declare_parameter("manipulation_radius_m", 0.75).value
        )
        self.manipulation_duration_sec = float(
            self.declare_parameter("manipulation_duration_sec", 1.0).value
        )
        self.pose_timeout_sec = float(
            self.declare_parameter("ground_truth_timeout_sec", 1.0).value
        )
        self.goal_timeout_sec = float(
            self.declare_parameter("goal_timeout_sec", 5.0).value
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
        self.create_subscription(
            PoseArray,
            "/sim/rover_pose",
            self._ground_truth_callback,
            10,
            callback_group=self._pose_callback_group,
        )

        self.action_server = ActionServer(
            self,
            ManipulateBlock,
            "/manipulate_block",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"Manipulation stub ready on /manipulate_block "
            f"(manipulation_radius_m={self.manipulation_radius_m:.2f}, "
            f"goal_frame='{self.goal_frame}')"
        )

    def goal_callback(self, goal_request: ManipulateBlock.Goal):
        frame_id = goal_request.target_pos.header.frame_id.strip()
        target = goal_request.target_pos.point

        if goal_request.operation not in (
            ManipulateBlock.Goal.GRASP,
            ManipulateBlock.Goal.DEPOSIT,
        ):
            self.get_logger().warn(
                f"Rejecting goal with unsupported operation "
                f"{goal_request.operation}."
            )
            return GoalResponse.REJECT

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
        target = goal_handle.request.target_pos.point
        target_x = float(target.x)
        target_y = float(target.y)
        operation = int(goal_handle.request.operation)
        operation_name = self._operation_name(operation)

        self.get_logger().info(
            f"Executing {operation_name} goal at "
            f"({target_x:.3f}, {target_y:.3f})"
        )

        result = ManipulateBlock.Result()
        feedback = ManipulateBlock.Feedback()
        start_time = self.get_clock().now()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled."
                return result

            pose = self._get_current_pose()
            elapsed_sec = (self.get_clock().now() - start_time).nanoseconds / 1e9

            if pose is None:
                feedback.current_stage = "waiting_for_pose"
                goal_handle.publish_feedback(feedback)
                if elapsed_sec >= self.goal_timeout_sec:
                    goal_handle.abort()
                    result.success = False
                    result.message = "No recent global pose received."
                    return result
                time.sleep(self.control_period_sec)
                continue

            x, y, _ = pose
            distance = math.hypot(target_x - x, target_y - y)
            if distance <= self.manipulation_radius_m:
                break

            feedback.current_stage = "checking_range"
            goal_handle.publish_feedback(feedback)
            if elapsed_sec >= self.goal_timeout_sec:
                goal_handle.abort()
                result.success = False
                result.message = (
                    f"Timed out waiting to enter manipulation range "
                    f"({distance:.3f} m away)."
                )
                return result
            time.sleep(self.control_period_sec)

        feedback.current_stage = f"executing_{operation_name}"
        goal_handle.publish_feedback(feedback)
        execute_start = time.monotonic()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Goal canceled."
                return result

            if time.monotonic() - execute_start >= self.manipulation_duration_sec:
                feedback.current_stage = "complete"
                goal_handle.publish_feedback(feedback)
                goal_handle.succeed()
                result.success = True
                result.message = f"{operation_name.capitalize()} completed."
                return result

            time.sleep(self.control_period_sec)

        result.success = False
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

        with self._latest_pose_lock:
            self._latest_pose = (
                float(pose_msg.position.x),
                float(pose_msg.position.y),
                0.0,
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

    @staticmethod
    def _operation_name(operation: int) -> str:
        if operation == ManipulateBlock.Goal.GRASP:
            return "grasp"
        if operation == ManipulateBlock.Goal.DEPOSIT:
            return "deposit"
        return f"unknown({operation})"

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main():
    try:
        rclpy.init()
        node = ManipulationStub()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
