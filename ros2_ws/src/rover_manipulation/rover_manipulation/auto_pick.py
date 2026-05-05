import math
import threading
import time
from collections import defaultdict, deque
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rover_interface.action import ManipulateBlock
from rover_interface.msg import BlockPoseObservation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
        roll / 2
    ) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
        roll / 2
    ) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
        roll / 2
    ) * math.cos(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def rot_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def mat_vec_mul(R, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


class AutoPickNode(Node):
    ARM_SPEED = 0.55
    GRIPPER_OPEN = 100
    GRIPPER_CLOSE = 20
    GRIPPER_LEN = 0.09
    HOVER_HEIGHT = 0.055
    LIFT_HEIGHT = 0.075
    COOLDOWN_SEC = 4.0
    MIN_CONFIDENCE = 0.25
    TABLE_Z = 0.020
    GRASP_DEPTH = 0.010
    BLOCK_RADIUS = 0.0075

    ARM_XYZ = (0.0880, 0.0, 0.0585)
    ARM_RPY = (0.0, 0.0, 0.0)
    CAMERA_XYZ = (0.1746, 0.0, 0.0906)
    CAMERA_RPY = (0.0, 0.4712, 0.0)

    ARM_CORRECTION_X = 0.04
    ARM_CORRECTION_Y = 0.03
    ARM_CORRECTION_Z = 0.000

    MIN_REACH_R = 0.12
    MAX_REACH_R = 0.40
    MAX_ABS_Y = 0.22

    SMOOTH_WINDOW = 8
    MIN_SMOOTH_SAMPLES = 5
    MAX_SAMPLE_STD = 0.012
    TARGET_LOCK_SEC = 3.0

    WAIT_CLOSE = 1.0

    def __init__(self):
        super().__init__("auto_pick_node")

        self._moveit_client = ActionClient(self, MoveGroup, "move_action")
        self.end_effector_link = "joint6_flange"
        self.base_frame = "g_base"

        self.current_joint_state = None
        self.initial_joint_state = None
        self.current_grip_val = self.GRIPPER_OPEN

        self.locked_target_id: Optional[int] = None
        self.locked_until = 0.0
        self.block_buffers = defaultdict(lambda: deque(maxlen=self.SMOOTH_WINDOW))

        self.R_cam = rot_from_rpy(*self.CAMERA_RPY)
        self.R_arm = rot_from_rpy(*self.ARM_RPY)

        self._action_cb_group = ReentrantCallbackGroup()
        self._sensor_cb_group = ReentrantCallbackGroup()

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10,
            callback_group=self._sensor_cb_group,
        )
        self.vision_sub = self.create_subscription(
            BlockPoseObservation,
            "/cv/block_poses",
            self.vision_callback,
            10,
            callback_group=self._sensor_cb_group,
        )
        self.gripper_pub = self.create_publisher(Int32, "/mycobot/gripper_cmd", 10)
        self.grip_state_sub = self.create_subscription(
            Int32,
            "/mycobot/gripper_state",
            self.grip_state_callback,
            10,
            callback_group=self._sensor_cb_group,
        )

        self.action_server = ActionServer(
            self,
            ManipulateBlock,
            "/manipulate_block",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._action_cb_group,
        )

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._moveit_client.wait_for_server()

        deadline = time.time() + 5.0
        while (
            self.initial_joint_state is None and rclpy.ok() and time.time() < deadline
        ):
            time.sleep(0.1)

        self.control_gripper(self.GRIPPER_OPEN)
        self.get_logger().info("Ready - /manipulate_block action server active")

    def joint_callback(self, msg):
        self.current_joint_state = msg
        if self.initial_joint_state is None:
            self.initial_joint_state = msg

    def grip_state_callback(self, msg):
        self.current_grip_val = msg.data

    def vision_callback(self, msg):
        for b in msg.observations:
            if b.confidence < self.MIN_CONFIDENCE:
                continue
            arm_xyz = self.block_pose_to_arm_xyz(b)
            if arm_xyz is None:
                continue
            block_id = int(getattr(b, "id", 0))
            self.block_buffers[block_id].append(
                (arm_xyz[0], arm_xyz[1], arm_xyz[2], float(b.confidence))
            )

    def goal_callback(self, goal_request):
        op = goal_request.operation
        if op not in (ManipulateBlock.Goal.GRASP, ManipulateBlock.Goal.DEPOSIT):
            self.get_logger().warn(f"Rejecting unsupported operation {op}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        operation = goal_handle.request.operation
        target = goal_handle.request.target_pos.point
        feedback = ManipulateBlock.Feedback()
        result = ManipulateBlock.Result()

        op_name = "grasp" if operation == ManipulateBlock.Goal.GRASP else "deposit"
        self.get_logger().info(
            f"Executing {op_name} at ({target.x:.3f}, {target.y:.3f})"
        )

        if operation == ManipulateBlock.Goal.GRASP:
            feedback.current_stage = "waiting_for_stable_target"
            goal_handle.publish_feedback(feedback)

            target_xyz = self.wait_for_stable_target(goal_handle, timeout=15.0)
            if target_xyz is None:
                goal_handle.abort()
                result.success = False
                result.message = "No stable block target found."
                return result

            arm_x, arm_y = target_xyz
            self.get_logger().info(f"Stable target: ({arm_x:.4f}, {arm_y:.4f})")

            feedback.current_stage = "moving_to_hover"
            goal_handle.publish_feedback(feedback)
            ok = self.moveit_move(
                arm_x, arm_y, self.TABLE_Z + self.HOVER_HEIGHT, "HOVER"
            )
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = "HOVER failed."
                return result

            feedback.current_stage = "grasping"
            goal_handle.publish_feedback(feedback)
            ok = self.moveit_move(arm_x, arm_y, self.TABLE_Z, "GRASP")
            if not ok:
                self.back_to_start()
                goal_handle.abort()
                result.success = False
                result.message = "GRASP failed."
                return result

            self.control_gripper(self.GRIPPER_CLOSE)
            time.sleep(self.WAIT_CLOSE)

            feedback.current_stage = "lifting"
            goal_handle.publish_feedback(feedback)
            self.moveit_move(arm_x, arm_y, self.TABLE_Z + self.LIFT_HEIGHT, "LIFT")

            feedback.current_stage = "returning_home"
            goal_handle.publish_feedback(feedback)
            self.back_to_start()

            feedback.current_stage = "complete"
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.message = "Grasp completed."
            return result

        elif operation == ManipulateBlock.Goal.DEPOSIT:
            place_x = float(target.x)
            place_y = float(target.y)
            place_z = float(target.z) if target.z != 0.0 else self.TABLE_Z

            feedback.current_stage = "moving_to_place_hover"
            goal_handle.publish_feedback(feedback)
            ok = self.moveit_move(
                place_x, place_y, place_z + self.HOVER_HEIGHT, "PLACE_HOVER"
            )
            if not ok:
                goal_handle.abort()
                result.success = False
                result.message = "PLACE_HOVER failed."
                return result

            feedback.current_stage = "depositing"
            goal_handle.publish_feedback(feedback)
            ok = self.moveit_move(place_x, place_y, place_z, "PLACE_GRASP")
            if not ok:
                self.back_to_start()
                goal_handle.abort()
                result.success = False
                result.message = "DEPOSIT move failed."
                return result

            self.control_gripper(self.GRIPPER_OPEN)
            time.sleep(1.0)

            feedback.current_stage = "lifting_after_deposit"
            goal_handle.publish_feedback(feedback)
            self.moveit_move(
                place_x, place_y, place_z + self.HOVER_HEIGHT, "PLACE_LIFT"
            )

            feedback.current_stage = "returning_home"
            goal_handle.publish_feedback(feedback)
            self.back_to_start()

            feedback.current_stage = "complete"
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.message = "Deposit completed."
            return result

        result.success = False
        result.message = "Unknown operation."
        return result

    def wait_for_stable_target(self, goal_handle, timeout=15.0):
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            if goal_handle.is_cancel_requested:
                return None
            target = self.select_stable_target()
            if target is not None:
                _, arm_x, arm_y, _, _, _ = target
                return arm_x, arm_y
            time.sleep(0.2)
        return None

    def block_pose_to_arm_xyz(self, block) -> Optional[Tuple[float, float, float]]:
        cam_x = float(block.pose.position.x)
        cam_y = float(block.pose.position.y)
        cam_z = float(block.pose.position.z)

        if not math.isfinite(cam_x + cam_y + cam_z):
            return None
        if cam_z <= 0.03:
            return None

        cam_z_center = cam_z + self.BLOCK_RADIUS
        p_cam_link = (cam_z_center, -cam_x, -cam_y)
        p_parent_rot = mat_vec_mul(self.R_cam, p_cam_link)
        p_parent = (
            self.CAMERA_XYZ[0] + p_parent_rot[0],
            self.CAMERA_XYZ[1] + p_parent_rot[1],
            self.CAMERA_XYZ[2] + p_parent_rot[2],
        )
        p_rel = (
            p_parent[0] - self.ARM_XYZ[0],
            p_parent[1] - self.ARM_XYZ[1],
            p_parent[2] - self.ARM_XYZ[2],
        )
        R = self.R_arm
        p_arm = (
            R[0][0] * p_rel[0] + R[1][0] * p_rel[1] + R[2][0] * p_rel[2],
            R[0][1] * p_rel[0] + R[1][1] * p_rel[1] + R[2][1] * p_rel[2],
            R[0][2] * p_rel[0] + R[1][2] * p_rel[1] + R[2][2] * p_rel[2],
        )
        return (
            p_arm[0] + self.ARM_CORRECTION_X,
            p_arm[1] + self.ARM_CORRECTION_Y,
            p_arm[2] + self.ARM_CORRECTION_Z,
        )

    def select_stable_target(self):
        candidates = []
        now = time.time()

        for block_id, buf in self.block_buffers.items():
            if len(buf) < self.MIN_SMOOTH_SAMPLES:
                continue
            xs = [v[0] for v in buf]
            ys = [v[1] for v in buf]
            zs = [v[2] for v in buf]
            cs = [v[3] for v in buf]
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            mean_z = sum(zs) / len(zs)
            mean_c = sum(cs) / len(cs)
            std_xy = max(self.std(xs), self.std(ys))
            r = math.sqrt(mean_x * mean_x + mean_y * mean_y)
            if std_xy > self.MAX_SAMPLE_STD:
                continue
            if r < self.MIN_REACH_R or r > self.MAX_REACH_R:
                continue
            if abs(mean_y) > self.MAX_ABS_Y:
                continue
            candidates.append((block_id, mean_x, mean_y, mean_z, mean_c, std_xy))

        if not candidates:
            return None

        if self.locked_target_id is not None and now < self.locked_until:
            for c in candidates:
                if c[0] == self.locked_target_id:
                    return c

        candidates.sort(key=lambda c: (math.sqrt(c[1] * c[1] + c[2] * c[2]), -c[4]))
        return candidates[0]

    @staticmethod
    def std(values):
        if len(values) <= 1:
            return 0.0
        m = sum(values) / len(values)
        return math.sqrt(sum((v - m) ** 2 for v in values) / len(values))

    def control_gripper(self, value):
        msg = Int32()
        msg.data = int(value)
        self.gripper_pub.publish(msg)

    def moveit_move(self, x, y, z_tcp, task_type) -> bool:
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 40
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -0.35
        goal_msg.request.workspace_parameters.min_corner.y = -0.35
        goal_msg.request.workspace_parameters.min_corner.z = 0.0
        goal_msg.request.workspace_parameters.max_corner.x = 0.35
        goal_msg.request.workspace_parameters.max_corner.y = 0.35
        goal_msg.request.workspace_parameters.max_corner.z = 0.5
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"

        if task_type in ["GRASP", "LIFT", "PLACE_GRASP", "PLACE_LIFT"]:
            goal_msg.request.max_velocity_scaling_factor = 0.25
            goal_msg.request.max_acceleration_scaling_factor = 0.25
        else:
            goal_msg.request.max_velocity_scaling_factor = self.ARM_SPEED
            goal_msg.request.max_acceleration_scaling_factor = self.ARM_SPEED

        flange_z = z_tcp + self.GRIPPER_LEN - self.GRASP_DEPTH
        flange_z = max(0.03, min(0.45, flange_z))

        target_orientation = euler_to_quaternion(0.0, math.pi, 0.0)

        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.end_effector_link
        pc.constraint_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        if task_type in ["GRASP", "PLACE_GRASP"]:
            primitive.dimensions = [0.010]
        elif task_type in ["HOVER", "LIFT", "PLACE_HOVER", "PLACE_LIFT"]:
            primitive.dimensions = [0.018]
        else:
            primitive.dimensions = [0.025]
        pc.constraint_region.primitives.append(primitive)

        target_point = Pose()
        target_point.position.x = float(x)
        target_point.position.y = float(y)
        target_point.position.z = float(flange_z)
        target_point.orientation = target_orientation
        pc.constraint_region.primitive_poses.append(target_point)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.end_effector_link
        oc.orientation = target_orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 0.5

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        goal_msg.request.goal_constraints.append(constraints)

        self.get_logger().info(
            f"[MOVE] {task_type}: ({x:.4f}, {y:.4f}, {flange_z:.4f})"
        )

        future = self._moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"Planner rejected: {task_type}")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        err_code = result_future.result().result.error_code.val

        if err_code != 1:
            self.get_logger().error(
                f"Execution failed for {task_type}. Code: {err_code}"
            )
            return False

        self.get_logger().info(f"[OK] {task_type}")
        return True

    def back_to_start(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_group"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 8.0
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -0.35
        goal_msg.request.workspace_parameters.min_corner.y = -0.35
        goal_msg.request.workspace_parameters.min_corner.z = 0.0
        goal_msg.request.workspace_parameters.max_corner.x = 0.35
        goal_msg.request.workspace_parameters.max_corner.y = 0.35
        goal_msg.request.workspace_parameters.max_corner.z = 0.5
        goal_msg.request.pipeline_id = "ompl"
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.max_velocity_scaling_factor = self.ARM_SPEED
        goal_msg.request.max_acceleration_scaling_factor = self.ARM_SPEED

        if self.initial_joint_state is None:
            return

        names = list(self.initial_joint_state.name)
        arm = [
            n for n in names if "gripper" not in n.lower() and "wheel" not in n.lower()
        ]
        if len(arm) < 6:
            return
        arm_joints = arm[:6]
        safe_home = [0.0, -0.017, 0.024, -0.070, -0.008, 0.012]

        constraints = Constraints()
        for i, name in enumerate(arm_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = safe_home[i]
            jc.tolerance_above = 0.08
            jc.tolerance_below = 0.08
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        self.get_logger().info("[MOVE] HOME")

        future = self._moveit_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("[OK] HOME")

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AutoPickNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
