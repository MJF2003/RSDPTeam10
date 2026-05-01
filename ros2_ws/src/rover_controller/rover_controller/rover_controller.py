from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from rclpy.action.client import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from rover_controller.navigation_goals import (
    compute_standoff_pose,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from rover_controller.semantic_obstacles import (
    SemanticObstacle,
    build_obstacle_cloud,
)
from rover_interface.action import Explore as ExploreAction
from rover_interface.action import ManipulateBlock, NavigateToPos
from rover_interface.msg import (
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
)


class Phase:
    class StartUp(Enum):
        WAIT = auto()
        OBSERVE_BINS = auto()

    class Explore(Enum):
        EXPLORE = auto()

    class ApproachBlock(Enum):
        PROPOSE_NAV_POSE = auto()  # TODO(Alex): Implement this logic
        EXECUTE_NAV = auto()

    class GraspBlock(Enum):
        ATTEMPT_GRASP = auto()
        RECOMPUTE_POSITION = auto()
        TRANSITION_TO_CARRY = auto()  # TODO(Alex): Can the manipulator node do this?

    class ApproachBin(Enum):
        PROPOSE_NAV_POSE = auto()
        EXECUTE_NAV = auto()

    class DepositBlock(Enum):
        ATTEMPT_DEPOSIT = auto()
        REGRASP_BLOCK = auto()  # TODO(Alex): TBD whether we need this

    class PostDeposit(Enum):
        TERMINATE = auto()

    class Stopped(Enum):
        STOPPED = auto()


def color_to_str(color: BlockBinColor | int) -> str:
    "Converts BlockBinColor enums to strings"
    color_value = int(color.color) if isinstance(color, BlockBinColor) else int(color)
    return {
        BlockBinColor.BLUE: "BLUE",
        BlockBinColor.RED: "RED",
        BlockBinColor.YELLOW: "YELLOW",
        BlockBinColor.PURPLE: "PURPLE",
        BlockBinColor.PINK: "PINK",
        BlockBinColor.GREEN: "GREEN",
    }.get(color_value, f"UNKNOWN({color_value})")


def state_to_str(state: Enum) -> str:
    return f"{state.__class__.__name__}.{state.name}"


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.state = Phase.StartUp.WAIT
        self.get_logger().info(f"Launching controller node with state {self.state=}")

        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.robot_base_frame = str(
            self.declare_parameter("robot_base_frame", "base_link").value
        )
        self.navigation_standoff_distance_m = float(
            self.declare_parameter("navigation_standoff_distance_m", 0.5).value
        )
        self.min_goal_vector_length_m = float(
            self.declare_parameter("min_goal_vector_length_m", 1e-3).value
        )
        self.explore_goal_timeout_sec = float(
            self.declare_parameter("explore_goal_timeout_sec", 30.0).value
        )
        self.cmd_vel_topic = str(
            self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        )
        self.state_marker_topic = str(
            self.declare_parameter(
                "state_marker_topic", "/controller/state_marker"
            ).value
        )
        self.state_marker_frame = str(
            self.declare_parameter(
                "state_marker_frame",
                self.robot_base_frame,
            ).value
        )
        self.state_marker_z_offset_m = float(
            self.declare_parameter("state_marker_z_offset_m", 0.8).value
        )
        self.state_marker_text_height_m = float(
            self.declare_parameter("state_marker_text_height_m", 0.2).value
        )
        self.startup_observation_duration_sec = float(
            self.declare_parameter("startup_observation_duration_sec", 5.0).value
        )
        self.startup_observation_angular_z = float(
            self.declare_parameter("startup_observation_angular_z", 1.0).value
        )
        self.startup_required_bin_count = int(
            self.declare_parameter("startup_required_bin_count", 3).value
        )
        self.semantic_obstacles_enabled = bool(
            self.declare_parameter("semantic_obstacles_enabled", True).value
        )
        self.semantic_obstacle_topic = str(
            self.declare_parameter(
                "semantic_obstacle_topic",
                "/controller/semantic_obstacles",
            ).value
        )
        self.semantic_obstacle_publish_period_sec = float(
            self.declare_parameter(
                "semantic_obstacle_publish_period_sec",
                0.2,
            ).value
        )
        self.semantic_block_radius_m = float(
            self.declare_parameter("semantic_block_radius_m", 0.25).value
        )
        self.semantic_bin_radius_m = float(
            self.declare_parameter("semantic_bin_radius_m", 0.35).value
        )
        self.semantic_fixed_keepout_radius_m = float(
            self.declare_parameter("semantic_fixed_keepout_radius_m", 0.35).value
        )
        self.semantic_obstacle_point_spacing_m = float(
            self.declare_parameter("semantic_obstacle_point_spacing_m", 0.05).value
        )
        self.semantic_obstacle_point_z_m = float(
            self.declare_parameter("semantic_obstacle_point_z_m", 0.25).value
        )
        self.tf_lookup_timeout = Duration(
            seconds=float(self.declare_parameter("tf_lookup_timeout_sec", 0.2).value)
        )
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mission_start_requested = False
        self.mission_stopped = False
        self.last_startup_readiness_log_time = 0.0
        self.start_mission_srv = self.create_service(
            Trigger,
            "/controller/start_mission",
            self.start_mission_callback,
        )
        self.stop_mission_srv = self.create_service(
            Trigger,
            "/controller/stop_mission",
            self.stop_mission_callback,
        )
        self.state_marker_pub = self.create_publisher(
            Marker,
            self.state_marker_topic,
            10,
        )
        self.semantic_obstacle_pub = self.create_publisher(
            PointCloud2,
            self.semantic_obstacle_topic,
            1,
        )
        self.startup_observation_start_time = None

        block_topic = "/controller/block_poses"
        # Observation Management
        self.obs_sub = self.create_subscription(
            BlockPoseSmoothedArray,
            topic=block_topic,
            callback=self.block_pose_callback,
            qos_profile=1,
        )
        self.get_logger().info(f"Starting subscription to {block_topic}")

        bin_topic = "/controller/bin_poses"
        self.bin_obs_sub = self.create_subscription(
            BinPoseSmoothedArray,
            topic=bin_topic,
            callback=self.bin_pose_callback,
            qos_profile=1,
        )
        self.get_logger().info(f"Starting subscription to {bin_topic}")

        self.blocks: list[BlockPoseSmoothed] = []
        self.collected: set[int] = set()  # stores ids of collected blocks
        self.fixed_keepout_positions: dict[int, PointStamped] = {}
        self.bins: list[BinPoseSmoothed] = []
        self.target_block: None | BlockPoseSmoothed = None
        self.target_bin: None | BinPoseSmoothed = None

        # Action client for navigation
        action_topic = "/navigate_to_pos"
        self.get_logger().info(
            f"Connecting to navigation action server {action_topic=}"
        )
        self.nav_action_client = ActionClient(
            self,
            NavigateToPos,
            action_topic,
        )
        self.nav_goal_in_flight = False
        self.nav_goal_handle = None
        self.nav_cancel_in_flight = False

        self.explore_action_topic = str(
            self.declare_parameter("explore_action_name", "/explore").value
        )
        self.get_logger().info(
            f"Connecting to exploration action server {self.explore_action_topic=}"
        )
        self.explore_action_client = ActionClient(
            self,
            ExploreAction,
            self.explore_action_topic,
        )
        self.explore_goal_in_flight = False
        self.explore_goal_handle = None
        self.explore_cancel_in_flight = False
        self.last_explore_wait_log_time = 0.0

        manip_action_topic = "/manipulate_block"
        self.get_logger().info(
            f"Connecting to manipulation action server {manip_action_topic=}"
        )
        self.manip_action_client = ActionClient(
            self,
            ManipulateBlock,
            manip_action_topic,
        )
        self.manip_goal_in_flight = False
        self.manip_goal_operation: int | None = None
        self.manip_goal_handle = None
        self.manip_cancel_in_flight = False
        self.required_startup_action_clients = (
            (action_topic, self.nav_action_client),
            (manip_action_topic, self.manip_action_client),
        )

        # Spin main controller loop
        controller_period = 0.1
        self.main_loop_timer = self.create_timer(controller_period, callback=self.loop)
        self.get_logger().info(
            f"Setting up main loop timer with frequency {1 / controller_period:.1f}Hz"
        )
        if self.semantic_obstacles_enabled:
            self.semantic_obstacle_timer = self.create_timer(
                self.semantic_obstacle_publish_period_sec,
                callback=self.publish_semantic_obstacles,
            )
            self.get_logger().info(
                "Publishing semantic costmap obstacles on "
                f"{self.semantic_obstacle_topic}"
            )

        self.loop_count = 0  # var for controlling frequency of state logging

    def loop(self):
        self.loop_count += 1
        if self.loop_count % 50 == 0:  # logs generic status every 5s
            self.log_status()
        match self.state:
            case Phase.StartUp.WAIT:
                self.wait()
            case Phase.StartUp.OBSERVE_BINS:
                self.observe_bins()
            case Phase.Explore.EXPLORE:
                self.explore()
            case Phase.ApproachBlock.EXECUTE_NAV:
                self.navigate_to_pose(self.target_block)
            case Phase.GraspBlock.ATTEMPT_GRASP:
                self.grasp_block(self.target_block)
            case Phase.ApproachBin.PROPOSE_NAV_POSE:
                self.select_target_bin(self.target_block)
            case Phase.ApproachBin.EXECUTE_NAV:
                self.navigate_to_pose(self.target_bin)
            case Phase.DepositBlock.ATTEMPT_DEPOSIT:
                self.deposit_block()
            case Phase.PostDeposit.TERMINATE:
                pass
            case Phase.Stopped.STOPPED:
                self.cmd_vel_pub.publish(Twist())
            case _:
                self.get_logger().warn(f"Unexpected state {self.state=}")

        self.publish_state_marker()

    def start_mission_callback(self, _request, response):
        if self.mission_stopped:
            response.success = False
            response.message = (
                "Mission is stopped; relaunch the controller to start again."
            )
            return response

        if self.mission_start_requested:
            response.success = True
            response.message = "Mission start was already requested."
            return response

        self.mission_start_requested = True
        response.success = True
        response.message = (
            "Mission start requested; controller will start when interfaces are ready."
        )
        self.get_logger().info(response.message)
        return response

    def stop_mission_callback(self, _request, response):
        if self.mission_stopped:
            self.cmd_vel_pub.publish(Twist())
            response.success = True
            response.message = "Mission is already stopped."
            return response

        self.stop_mission()
        response.success = True
        response.message = "Mission stop requested."
        return response

    def stop_mission(self):
        self.get_logger().warning("Stopping mission: canceling active goals.")
        self.mission_stopped = True
        self.mission_start_requested = False
        self.state = Phase.Stopped.STOPPED

        self.cancel_exploration_goal(reason="mission stop", require_target=False)
        self.cancel_navigation_goal()
        self.cancel_manipulation_goal()
        self.cmd_vel_pub.publish(Twist())

    def publish_state_marker(self):
        marker = Marker()
        marker.header.frame_id = self.state_marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "controller_state"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = self.state_marker_z_offset_m
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.state_marker_text_height_m
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = state_to_str(self.state)
        self.state_marker_pub.publish(marker)

    def log_status(self):
        def log_block_bin(block_or_bin: BlockPoseSmoothed | BinPoseSmoothed) -> str:
            p = block_or_bin.position.point
            return f"{color_to_str(block_or_bin.color)} at ({p.x:.2f}, {p.y:.2f})"

        blocks_str = ", ".join(log_block_bin(b) for b in self.blocks)
        bins_str = ", ".join(log_block_bin(b) for b in self.bins)

        self.get_logger().info(
            f"Current phase: {self.state}\n"
            f"Known blocks: [{blocks_str}]\n"
            f"Known bins: [{bins_str}]"
        )

    def wait(self):
        "Waits until all expected topics and action servers are ready"
        readiness = self.get_startup_readiness()
        self.log_startup_readiness(readiness)
        if not all(ready for _, ready in readiness):
            return

        if not self.mission_start_requested:
            return

        self.get_logger().info(
            "All interfaces ready and mission start requested; "
            "progressing to bin observation."
        )
        self.state = Phase.StartUp.OBSERVE_BINS

    def get_startup_readiness(self) -> list[tuple[str, bool]]:
        readiness = [
            (
                f"publisher:{sub.topic_name}",
                self.count_publishers(sub.topic_name) > 0,
            )
            for sub in self.subscriptions
        ]
        readiness.extend(
            (f"action:{name}", client.server_is_ready())
            for name, client in self.required_startup_action_clients
        )
        return readiness

    def log_startup_readiness(self, readiness: list[tuple[str, bool]]):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_startup_readiness_log_time < 1.0:
            return

        self.last_startup_readiness_log_time = now_sec
        ready_count = sum(1 for _, ready in readiness if ready)
        total_count = len(readiness)
        start_status = (
            "start requested"
            if self.mission_start_requested
            else "waiting for /controller/start_mission"
        )
        missing = ", ".join(name for name, ready in readiness if not ready)
        message = (
            f"Startup readiness: {ready_count}/{total_count} interfaces ready; "
            f"{start_status}."
        )
        if missing:
            message = f"{message} Waiting for: {missing}"
        self.get_logger().info(message)

    def observe_bins(self):
        if self.startup_observation_start_time is None:
            self.startup_observation_start_time = self.get_clock().now()
            self.get_logger().info(
                "Starting startup bin observation sweep for "
                f"{self.startup_observation_duration_sec:.1f}s"
            )

        elapsed = (
            self.get_clock().now() - self.startup_observation_start_time
        ).nanoseconds / 1e9
        if elapsed < self.startup_observation_duration_sec:
            twist = Twist()
            twist.angular.z = self.startup_observation_angular_z
            self.cmd_vel_pub.publish(twist)
            return

        self.cmd_vel_pub.publish(Twist())
        observed_bin_count = len(self.bins)
        if observed_bin_count >= self.startup_required_bin_count:
            self.get_logger().info(
                f"Observed {observed_bin_count} bins during startup sweep."
            )
        else:
            self.get_logger().warning(
                "Startup sweep completed with "
                f"{observed_bin_count}/{self.startup_required_bin_count} "
                "required bins observed; continuing to exploration."
            )

        self.state = Phase.Explore.EXPLORE

    def explore(self):
        if self.explore_goal_in_flight:
            return

        if self.target_block is not None:
            self.state = Phase.ApproachBlock.EXECUTE_NAV
            return

        self.send_exploration_goal()

    # ------------------------ EXPLORATION ----------------------------------------
    def send_exploration_goal(self) -> bool:
        if self.mission_stopped or self.explore_goal_in_flight:
            return False

        if not self.explore_action_client.wait_for_server(timeout_sec=0.5):
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_explore_wait_log_time >= 5.0:
                self.last_explore_wait_log_time = now_sec
                self.get_logger().warn(
                    "Exploration action server "
                    f"'{self.explore_action_topic}' is not available."
                )
            return False

        goal = ExploreAction.Goal()
        goal.max_runtime_sec = self.explore_goal_timeout_sec
        goal.run_until_cancelled = False

        self.explore_goal_in_flight = True
        self.get_logger().info(
            f"Sending exploration goal for {self.explore_goal_timeout_sec:.1f}s"
        )

        goal_future = self.explore_action_client.send_goal_async(
            goal,
            feedback_callback=self._explore_feedback_callback,
        )
        goal_future.add_done_callback(self._explore_goal_response_callback)
        return True

    def _explore_feedback_callback(self, _feedback_msg):
        pass

    def _explore_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.explore_goal_in_flight = False
            self.explore_goal_handle = None
            self.explore_cancel_in_flight = False
            self.get_logger().error("Exploration goal was rejected.")
            return

        self.explore_goal_handle = goal_handle
        self.get_logger().info("Exploration goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._explore_result_callback)
        if self.mission_stopped:
            self.cancel_exploration_goal(reason="mission stop", require_target=False)
            return
        if self.target_block is not None:
            self.cancel_exploration_goal()

    def cancel_exploration_goal(
        self,
        reason: str = "finding target block",
        require_target: bool = True,
    ) -> bool:
        if require_target and self.target_block is None:
            return False

        if self.explore_goal_handle is None:
            return False

        if self.explore_cancel_in_flight:
            return False

        self.explore_cancel_in_flight = True
        self.get_logger().info(f"Canceling exploration after {reason}.")
        cancel_future = self.explore_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._explore_cancel_callback)
        return True

    def _explore_cancel_callback(self, future):
        self.explore_cancel_in_flight = False
        cancel_response = future.result()
        goals_canceling = len(cancel_response.goals_canceling)
        if goals_canceling > 0:
            self.get_logger().info("Exploration cancel request accepted.")
        else:
            self.get_logger().warn("Exploration cancel request was not accepted.")

    def _explore_result_callback(self, future):
        self.explore_goal_in_flight = False
        self.explore_goal_handle = None
        self.explore_cancel_in_flight = False
        result = future.result().result
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Exploration result {status}: final_state='{result.final_state}', "
            f"goals_sent={result.goals_sent}, "
            f"goals_succeeded={result.goals_succeeded}, "
            f"goals_failed={result.goals_failed}, message='{result.message}'"
        )

        if self.mission_stopped:
            return

        if self.target_block is not None:
            self.state = Phase.ApproachBlock.EXECUTE_NAV

    def block_pose_callback(self, msg: BlockPoseSmoothedArray):
        if self.mission_stopped:
            return

        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store block poses
        self.blocks = msg.blocks  # type: ignore
        # if we have any uncollected blocks - set it as the target
        for block in self.blocks:
            if (block.id not in self.collected) and (self.target_block is None):
                self.get_logger().info(
                    f"Targeting {color_to_str(block.color)} block, ignoring collected blocks {self.collected=}"
                )
                self.target_block = block
                if self.state == Phase.Explore.EXPLORE:
                    self.cancel_exploration_goal()

    def bin_pose_callback(self, msg: BinPoseSmoothedArray):
        if self.mission_stopped:
            return

        # don't pay attention if we're doing a grasp or deposition
        if (self.state in Phase.GraspBlock) or (self.state in Phase.DepositBlock):
            return
        # parse and store bin poses
        self.bins = msg.bins  # type: ignore

    # ------------------------ SEMANTIC COSTMAP OBSTACLES ------------------------
    def publish_semantic_obstacles(self):
        obstacles = self._build_semantic_obstacles()
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()

        try:
            cloud = build_obstacle_cloud(
                header=header,
                obstacles=obstacles,
                point_spacing=self.semantic_obstacle_point_spacing_m,
                point_z=self.semantic_obstacle_point_z_m,
            )
        except ValueError as exc:
            self.get_logger().error(f"Cannot publish semantic obstacles: {exc}")
            return

        self.semantic_obstacle_pub.publish(cloud)

    def _build_semantic_obstacles(self) -> list[SemanticObstacle]:
        obstacles: list[SemanticObstacle] = []

        for block in self.blocks:
            if block.id in self.collected:
                continue

            obstacle = self._obstacle_from_point(
                block.position,
                self.semantic_block_radius_m,
            )
            if obstacle is not None:
                obstacles.append(obstacle)

        for bin_pose in self.bins:
            obstacle = self._obstacle_from_point(
                bin_pose.position,
                self.semantic_bin_radius_m,
            )
            if obstacle is not None:
                obstacles.append(obstacle)

        for point in self.fixed_keepout_positions.values():
            obstacle = self._obstacle_from_point(
                point,
                self.semantic_fixed_keepout_radius_m,
            )
            if obstacle is not None:
                obstacles.append(obstacle)

        return obstacles

    def _obstacle_from_point(
        self,
        point: PointStamped,
        radius: float,
    ) -> SemanticObstacle | None:
        point_in_map = self._point_in_map_frame(point)
        if point_in_map is None:
            return None

        return SemanticObstacle(
            x=float(point_in_map.point.x),
            y=float(point_in_map.point.y),
            radius=radius,
        )

    def _point_in_map_frame(self, point: PointStamped) -> PointStamped | None:
        source_frame = point.header.frame_id.strip() or self.map_frame
        if source_frame == self.map_frame:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                point.header.stamp,
                timeout=self.tf_lookup_timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                "Cannot transform semantic obstacle point from "
                f"'{source_frame}' to '{self.map_frame}': {exc}"
            )
            return None

        return do_transform_point(point, transform)

    @staticmethod
    def _copy_point_stamped(point: PointStamped) -> PointStamped:
        point_copy = PointStamped()
        point_copy.header.frame_id = point.header.frame_id
        point_copy.header.stamp = point.header.stamp
        point_copy.point.x = point.point.x
        point_copy.point.y = point.point.y
        point_copy.point.z = point.point.z
        return point_copy

    # ------------------------ NAVIGATION -----------------------------------------
    def navigate_to_pose(
        self, target: BlockPoseSmoothed | BinPoseSmoothed | None
    ) -> bool:
        """Send a navigation action goal for a specific block target."""
        # select a block to go to

        if self.mission_stopped or self.nav_goal_in_flight or (target is None):
            # this check for block is None is purely for typing. It can't be None
            # when this function is called (I think)
            return False

        if not self.nav_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "Navigation action server '/navigate_to_pos' is not available."
            )
            return False

        goal_pose = self._build_navigation_goal_pose(target.position)
        if goal_pose is None:
            return False

        goal = NavigateToPos.Goal()
        goal.target_pose = goal_pose

        self.nav_goal_in_flight = True
        goal_yaw = quaternion_to_yaw(
            x=goal_pose.pose.orientation.x,
            y=goal_pose.pose.orientation.y,
            z=goal_pose.pose.orientation.z,
            w=goal_pose.pose.orientation.w,
        )
        self.get_logger().info(
            "Sending navigation goal for target "
            f"at ({target.position.point.x:.2f}, {target.position.point.y:.2f}) "
            f"via approach pose ({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f}, yaw={goal_yaw:.2f} rad)"
        )

        goal_future = self.nav_action_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback_callback,
        )
        goal_future.add_done_callback(self._nav_goal_response_callback)
        return True

    def _build_navigation_goal_pose(
        self, target_position: PointStamped
    ) -> PoseStamped | None:
        goal_frame = target_position.header.frame_id.strip() or self.map_frame
        try:
            robot_transform = self.tf_buffer.lookup_transform(
                goal_frame,
                self.robot_base_frame,
                Time(),
                timeout=self.tf_lookup_timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Cannot compute navigation approach pose in frame '{goal_frame}': {exc}"
            )
            return None

        robot_x = float(robot_transform.transform.translation.x)
        robot_y = float(robot_transform.transform.translation.y)
        robot_yaw = quaternion_to_yaw(
            x=float(robot_transform.transform.rotation.x),
            y=float(robot_transform.transform.rotation.y),
            z=float(robot_transform.transform.rotation.z),
            w=float(robot_transform.transform.rotation.w),
        )

        approach_pose = compute_standoff_pose(
            robot_x=robot_x,
            robot_y=robot_y,
            robot_yaw=robot_yaw,
            target_x=float(target_position.point.x),
            target_y=float(target_position.point.y),
            stand_off_distance=self.navigation_standoff_distance_m,
            min_goal_vector_length=self.min_goal_vector_length_m,
        )
        goal_qz, goal_qw = yaw_to_quaternion(approach_pose.yaw)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = goal_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(approach_pose.x)
        goal_pose.pose.position.y = float(approach_pose.y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.z = float(goal_qz)
        goal_pose.pose.orientation.w = float(goal_qw)
        return goal_pose

    def _nav_feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        # self.get_logger().info(f"Navigation feedback: distance_remaining={dist:.3f} m")

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.nav_goal_in_flight = False
            self.nav_goal_handle = None
            self.nav_cancel_in_flight = False
            self.get_logger().error("Navigation goal was rejected.")
            return

        self.nav_goal_handle = goal_handle
        self.get_logger().info("Navigation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)
        if self.mission_stopped:
            self.cancel_navigation_goal()

    def cancel_navigation_goal(self) -> bool:
        if self.nav_goal_handle is None:
            return False

        if self.nav_cancel_in_flight:
            return False

        self.nav_cancel_in_flight = True
        self.get_logger().info("Canceling navigation goal.")
        cancel_future = self.nav_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._nav_cancel_callback)
        return True

    def _nav_cancel_callback(self, future):
        self.nav_cancel_in_flight = False
        cancel_response = future.result()
        goals_canceling = len(cancel_response.goals_canceling)
        if goals_canceling > 0:
            self.get_logger().info("Navigation cancel request accepted.")
        else:
            self.get_logger().warn("Navigation cancel request was not accepted.")

    def _nav_result_callback(self, future):
        self.nav_goal_in_flight = False
        self.nav_goal_handle = None
        self.nav_cancel_in_flight = False
        result = future.result().result
        if self.mission_stopped:
            status = "SUCCESS" if result.success else "FAILURE"
            self.get_logger().info(
                f"Navigation result after stop {status}: "
                f"final_distance={result.final_distance:.3f} m, "
                f"message='{result.message}'"
            )
            return

        if result.success:
            status = "SUCCESS"
            if self.state in Phase.ApproachBlock:
                self.get_logger().info(
                    f"Succesful navigation. Transitioning to phase {Phase.GraspBlock.ATTEMPT_GRASP}"
                )
                self.state = Phase.GraspBlock.ATTEMPT_GRASP
            elif self.state in Phase.ApproachBin:
                self.get_logger().info(
                    f"Succesfully navigated to {color_to_str(self.target_bin.color)} bin. Transitioning to phase {Phase.DepositBlock.ATTEMPT_DEPOSIT}"  # type: ignore
                )
                self.state = Phase.DepositBlock.ATTEMPT_DEPOSIT
            else:
                self.get_logger().warn(
                    f"Unexpected state at end of navigation: {self.state=}"
                )
        # TODO(Alex): what do we do if the navigation failed?
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Navigation result {status}: "
            f"final_distance={result.final_distance:.3f} m, message='{result.message}'"
        )

    # ------------------------ MANIPULATION -----------------------------------------
    def grasp_block(self, target_block: BlockPoseSmoothed | None):
        if target_block is None:
            self.get_logger().warn(f"Received invalid {target_block=}")
            self.state = Phase.Explore.EXPLORE
            return

        color = target_block.color
        self.send_manipulation_goal(
            operation=ManipulateBlock.Goal.GRASP,
            target=target_block,
            description=f"Grasping block with color: {color_to_str(color)}",
        )

    def select_target_bin(self, target_block: BlockPoseSmoothed | None):
        # TODO(Alex): Remove this assert
        assert target_block is not None
        color = target_block.color
        color_value = int(color.color)
        for bin in self.bins:
            if int(bin.color.color) != color_value:
                self.get_logger().info(
                    f"Ignoring bin {color_to_str(bin.color)} for targetting"
                )
                continue
            self.target_bin = bin
            self.state = Phase.ApproachBin.EXECUTE_NAV
            self.get_logger().info(
                f"Succesfully identified matching {color_to_str(color)} bin"
            )
            return
        self.get_logger().info(
            f"No matching bin known for block color {color_to_str(color)}"
        )

    def deposit_block(self):
        if self.target_block is None or self.target_bin is None:
            self.get_logger().warn(
                "Cannot deposit without both a target block and target bin."
            )
            self.state = Phase.Explore.EXPLORE
            return

        self.send_manipulation_goal(
            operation=ManipulateBlock.Goal.DEPOSIT,
            target=self.target_bin,
            description=(
                f"Depositing {color_to_str(self.target_block.color)} block in "
                f"{color_to_str(self.target_bin.color)} bin"
            ),
        )

    def send_manipulation_goal(
        self,
        operation: int,
        target: BlockPoseSmoothed | BinPoseSmoothed | None,
        description: str,
    ) -> bool:
        if self.mission_stopped or self.manip_goal_in_flight or (target is None):
            return False

        if not self.manip_action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn(
                "Manipulation action server '/manipulate_block' is not available."
            )
            return False

        goal = ManipulateBlock.Goal()
        goal.operation = operation
        goal.target_pos = target.position

        self.manip_goal_in_flight = True
        self.manip_goal_operation = operation
        self.get_logger().info(
            f"{description} at "
            f"({target.position.point.x:.2f}, {target.position.point.y:.2f})"
        )

        goal_future = self.manip_action_client.send_goal_async(
            goal,
            feedback_callback=self._manip_feedback_callback,
        )
        goal_future.add_done_callback(self._manip_goal_response_callback)
        return True

    def _manip_feedback_callback(self, feedback_msg):
        stage = feedback_msg.feedback.current_stage
        # self.get_logger().info(f"Manipulation feedback: current_stage='{stage}'")

    def _manip_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Manipulation goal was rejected.")
            self._handle_manipulation_result(success=False, message="Goal rejected.")
            return

        self.manip_goal_handle = goal_handle
        self.get_logger().info("Manipulation goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._manip_result_callback)
        if self.mission_stopped:
            self.cancel_manipulation_goal()

    def cancel_manipulation_goal(self) -> bool:
        if self.manip_goal_handle is None:
            return False

        if self.manip_cancel_in_flight:
            return False

        self.manip_cancel_in_flight = True
        self.get_logger().info("Canceling manipulation goal.")
        cancel_future = self.manip_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._manip_cancel_callback)
        return True

    def _manip_cancel_callback(self, future):
        self.manip_cancel_in_flight = False
        cancel_response = future.result()
        goals_canceling = len(cancel_response.goals_canceling)
        if goals_canceling > 0:
            self.get_logger().info("Manipulation cancel request accepted.")
        else:
            self.get_logger().warn("Manipulation cancel request was not accepted.")

    def _manip_result_callback(self, future):
        result = future.result().result
        status = "SUCCESS" if result.success else "FAILURE"
        self.get_logger().info(
            f"Manipulation result {status}: message='{result.message}'"
        )
        self._handle_manipulation_result(success=result.success, message=result.message)

    def _handle_manipulation_result(self, success: bool, message: str):
        operation = self.manip_goal_operation
        self.manip_goal_in_flight = False
        self.manip_goal_operation = None
        self.manip_goal_handle = None
        self.manip_cancel_in_flight = False

        if self.mission_stopped:
            return

        if operation == ManipulateBlock.Goal.GRASP:
            if success:
                if self.target_block is None:
                    self.get_logger().warn(
                        "Grasp reported success but there is no target block."
                    )
                    self.state = Phase.Explore.EXPLORE
                    return

                self.collected.add(self.target_block.id)
                self.fixed_keepout_positions[self.target_block.id] = (
                    self._copy_point_stamped(self.target_block.position)
                )
                self.get_logger().info(f"Adding target block to {self.collected=}")
                self.state = Phase.ApproachBin.PROPOSE_NAV_POSE
                return

            self.get_logger().warning(
                f"Grasp failed with message '{message}'. Re-approaching block."
            )
            self.state = Phase.ApproachBlock.EXECUTE_NAV
            return

        if operation == ManipulateBlock.Goal.DEPOSIT:
            if success:
                self.target_block = None
                self.target_bin = None

                # if we have successfully collected 3 blocks, terminate the mission,
                # or return to the exploration phase to find more
                if len(self.collected) < 3:
                    self.state = Phase.Explore.EXPLORE
                else:
                    self.state = Phase.PostDeposit.TERMINATE
                return

            self.get_logger().warning(
                f"Deposit failed with message '{message}'. Re-approaching bin."
            )
            self.state = Phase.ApproachBin.EXECUTE_NAV
            return

        self.get_logger().warn(
            f"Manipulation completed with unknown operation {operation=}; "
            f"returning to explore."
        )
        self.state = Phase.Explore.EXPLORE


def main():
    try:
        rclpy.init()
        node = ControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
