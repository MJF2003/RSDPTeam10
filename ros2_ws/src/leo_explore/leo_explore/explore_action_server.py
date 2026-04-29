#!/usr/bin/env python3
from collections import deque
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rover_interface.action import Explore
import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class ExploreActionServer(Node):
    STATE_IDLE = 'IDLE'
    STATE_WAITING_FOR_MAP = 'WAITING_FOR_MAP'
    STATE_WAITING_FOR_NAV2 = 'WAITING_FOR_NAV2'
    STATE_WAITING_FOR_TF = 'WAITING_FOR_TF'
    STATE_PLANNING = 'PLANNING'
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_ROTATING = 'ROTATING'
    STATE_NO_FRONTIER = 'NO_FRONTIER'
    STATE_CANCELING = 'CANCELING'
    STATE_FINISHED = 'FINISHED'

    def __init__(self):
        super().__init__('leo_explore_action_server')

        self.cb_group = ReentrantCallbackGroup()
        self.state_lock = threading.Lock()

        # ------------------------------------------------------------
        # Parameters for frontier extraction and Nav2 goal dispatch.
        # ------------------------------------------------------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('nav_action_name', '/navigate_to_pose')
        self.declare_parameter('bt_navigator_state_service', 'bt_navigator/get_state')
        self.declare_parameter('bt_navigator_state_timeout_sec', 0.5)

        # Cartographer free space is not always exactly 0.
        self.declare_parameter('free_threshold', 60)
        self.declare_parameter('occupied_threshold', 65)

        self.declare_parameter('min_frontier_size', 20)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('goal_backoff_cells', 8)

        self.declare_parameter('blacklist_radius', 0.60)
        self.declare_parameter('blacklist_time', 30.0)

        self.declare_parameter('plan_period', 3.0)
        self.declare_parameter('no_frontier_finish_count', 5)
        self.declare_parameter('finish_when_no_frontiers', True)
        self.declare_parameter('final_rotation_duration_sec', 5.0)
        self.declare_parameter('final_rotation_angular_z', 1.0)

        self.declare_parameter('unknown_gain_radius_cells', 10)
        self.declare_parameter('unknown_gain_weight', 0.025)
        self.declare_parameter('frontier_size_weight', 0.03)
        self.declare_parameter('distance_weight', 0.80)

        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.nav_action_name = self.get_parameter('nav_action_name').value
        self.bt_navigator_state_service = self.get_parameter(
            'bt_navigator_state_service'
        ).value
        self.bt_navigator_state_timeout_sec = float(
            self.get_parameter('bt_navigator_state_timeout_sec').value
        )

        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.goal_backoff_cells = int(self.get_parameter('goal_backoff_cells').value)

        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.blacklist_time = float(self.get_parameter('blacklist_time').value)

        self.plan_period = float(self.get_parameter('plan_period').value)
        self.no_frontier_finish_count = int(self.get_parameter('no_frontier_finish_count').value)
        self.finish_when_no_frontiers = bool(self.get_parameter('finish_when_no_frontiers').value)
        self.final_rotation_duration_sec = float(
            self.get_parameter('final_rotation_duration_sec').value
        )
        self.final_rotation_angular_z = float(
            self.get_parameter('final_rotation_angular_z').value
        )

        self.unknown_gain_radius_cells = int(self.get_parameter('unknown_gain_radius_cells').value)
        self.unknown_gain_weight = float(self.get_parameter('unknown_gain_weight').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)

        # ------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
            callback_group=self.cb_group,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_name,
            callback_group=self.cb_group,
        )

        self.bt_navigator_state_client = self.create_client(
            GetState,
            self.bt_navigator_state_service,
            callback_group=self.cb_group,
        )

        self._action_server = ActionServer(
            self,
            Explore,
            'explore',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.map_msg = None
        self.exploring_active = False
        self.active_goal_handle = None
        self.goal_start_time = None
        self.final_rotation_active = False
        self.final_rotation_start_time = None
        self.final_rotation_finish_message = ''

        self.current_state = self.STATE_IDLE
        self.finish_requested = False
        self.finish_success = False
        self.finish_message = ''

        self.nav_goal_in_progress = False
        self.current_nav_goal_handle = None
        self.current_goal_xy = None
        self.nav_cancel_requested = False
        self.bt_navigator_state = 'unknown'
        self.bt_navigator_state_error = None
        self.bt_navigator_state_future = None
        self.bt_navigator_state_request_started = 0.0
        self.last_nav2_wait_log_time = 0.0
        self.last_nav2_wait_reason = ''

        self.blacklist = []  # [(x, y, expire_time_sec)]

        self.latest_robot_xy = None
        self.latest_frontier_cluster_count = 0
        self.latest_frontier_candidate_count = 0
        self.latest_chosen_frontier_size = 0
        self.latest_unknown_gain = 0

        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.no_frontier_count = 0

        self.timer = self.create_timer(
            self.plan_period,
            self.plan_once,
            callback_group=self.cb_group,
        )

        self.get_logger().info('Explore action server started in IDLE state')

    # ------------------------------------------------------------
    # Time / state helpers
    # ------------------------------------------------------------
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed_since_goal_start(self) -> float:
        if self.goal_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9

    def set_state(self, state: str):
        with self.state_lock:
            if self.current_state != state:
                self.current_state = state
                self.get_logger().info(f'Explore state -> {state}')

    def request_finish(self, success: bool, message: str):
        with self.state_lock:
            if not self.finish_requested:
                self.finish_requested = True
                self.finish_success = success
                self.finish_message = message
                self.current_state = self.STATE_FINISHED if success else self.current_state

    def reset_explore_state_for_new_goal(self, goal_handle):
        self.exploring_active = True
        self.active_goal_handle = goal_handle
        self.goal_start_time = self.get_clock().now()

        self.current_state = self.STATE_WAITING_FOR_MAP
        self.finish_requested = False
        self.finish_success = False
        self.finish_message = ''

        self.nav_goal_in_progress = False
        self.current_nav_goal_handle = None
        self.current_goal_xy = None
        self.nav_cancel_requested = False

        self.blacklist = []
        self.latest_robot_xy = None
        self.latest_frontier_cluster_count = 0
        self.latest_frontier_candidate_count = 0
        self.latest_chosen_frontier_size = 0
        self.latest_unknown_gain = 0

        self.goals_sent = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.no_frontier_count = 0

    def make_result(self, success: bool, message: str) -> Explore.Result:
        result = Explore.Result()
        result.success = bool(success)
        result.message = message
        result.elapsed_time_sec = float(self.elapsed_since_goal_start())
        result.final_state = self.current_state
        result.goals_sent = int(self.goals_sent)
        result.goals_succeeded = int(self.goals_succeeded)
        result.goals_failed = int(self.goals_failed)
        return result

    def make_feedback(self) -> Explore.Feedback:
        feedback = Explore.Feedback()
        with self.state_lock:
            feedback.current_state = self.current_state
            feedback.elapsed_time_sec = float(self.elapsed_since_goal_start())

            if self.latest_robot_xy is None:
                feedback.robot_x = math.nan
                feedback.robot_y = math.nan
            else:
                feedback.robot_x = float(self.latest_robot_xy[0])
                feedback.robot_y = float(self.latest_robot_xy[1])

            if self.current_goal_xy is None:
                feedback.current_goal_x = math.nan
                feedback.current_goal_y = math.nan
            else:
                feedback.current_goal_x = float(self.current_goal_xy[0])
                feedback.current_goal_y = float(self.current_goal_xy[1])

            feedback.frontier_cluster_count = int(self.latest_frontier_cluster_count)
            feedback.frontier_candidate_count = int(self.latest_frontier_candidate_count)
            feedback.goals_sent = int(self.goals_sent)
            feedback.goals_succeeded = int(self.goals_succeeded)
            feedback.goals_failed = int(self.goals_failed)

        return feedback

    def cleanup_blacklist(self):
        now = self.now_sec()
        self.blacklist = [b for b in self.blacklist if b[2] > now]

    def is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by, _ in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def add_blacklist(self, x: float, y: float):
        expire = self.now_sec() + self.blacklist_time
        self.blacklist.append((x, y, expire))
        self.get_logger().warn(f'Blacklisted failed frontier goal: x={x:.2f}, y={y:.2f}')

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_final_rotation_cmd(self):
        twist = Twist()
        twist.angular.z = self.final_rotation_angular_z
        self.cmd_pub.publish(twist)

    def start_final_rotation(self, finish_message: str):
        with self.state_lock:
            if self.final_rotation_active:
                return

            self.final_rotation_active = True
            self.final_rotation_start_time = self.get_clock().now()
            self.final_rotation_finish_message = finish_message
            self.current_state = self.STATE_ROTATING

        self.get_logger().info(
            'Starting final camera sweep: '
            f'duration={self.final_rotation_duration_sec:.1f}s, '
            f'angular_z={self.final_rotation_angular_z:.2f}rad/s'
        )
        self.cancel_current_nav_goal()
        self.publish_final_rotation_cmd()

    def final_rotation_elapsed(self) -> float:
        if self.final_rotation_start_time is None:
            return 0.0
        return (
            self.get_clock().now() - self.final_rotation_start_time
        ).nanoseconds / 1e9

    def final_rotation_complete(self) -> bool:
        if not self.final_rotation_active:
            return False
        return (
            self.final_rotation_elapsed()
            >= max(0.0, self.final_rotation_duration_sec)
        )

    def deactivate_and_stop(self, final_state=None):
        if final_state is None:
            final_state = self.STATE_CANCELING

        with self.state_lock:
            self.exploring_active = False
            self.final_rotation_active = False
            self.current_state = final_state
        self.cancel_current_nav_goal()
        self.stop_robot()

    # ------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def goal_callback(self, goal_request):
        with self.state_lock:
            if self.exploring_active:
                self.get_logger().warn('Rejecting /explore goal: exploration already running')
                return GoalResponse.REJECT

        self.get_logger().info(
            f'Received /explore goal: max_runtime_sec={goal_request.max_runtime_sec}, '
            f'run_until_cancelled={goal_request.run_until_cancelled}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received /explore cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        with self.state_lock:
            self.reset_explore_state_for_new_goal(goal_handle)

        self.get_logger().info('Explore goal accepted. Frontier exploration is now active.')
        self.plan_once()

        max_runtime = float(goal_handle.request.max_runtime_sec)
        run_until_cancelled = bool(goal_handle.request.run_until_cancelled)

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Explore goal canceled by client')
                    self.deactivate_and_stop(self.STATE_CANCELING)
                    goal_handle.canceled()
                    return self.make_result(False, 'Goal canceled')

                elapsed = self.elapsed_since_goal_start()
                goal_handle.publish_feedback(self.make_feedback())

                with self.state_lock:
                    finish_requested = self.finish_requested
                    finish_success = self.finish_success
                    finish_message = self.finish_message
                    final_rotation_active = self.final_rotation_active
                    final_rotation_message = self.final_rotation_finish_message

                if finish_requested:
                    if finish_success and not final_rotation_active:
                        self.start_final_rotation(finish_message)
                        time.sleep(0.2)
                        continue
                    if (
                        finish_success
                        and final_rotation_active
                        and not self.final_rotation_complete()
                    ):
                        self.publish_final_rotation_cmd()
                        time.sleep(0.2)
                        continue

                    final_state = (
                        self.STATE_FINISHED
                        if finish_success
                        else self.current_state
                    )
                    self.deactivate_and_stop(final_state)
                    if finish_success:
                        goal_handle.succeed()
                    else:
                        goal_handle.abort()
                    return self.make_result(
                        finish_success,
                        final_rotation_message or finish_message,
                    )

                if (not run_until_cancelled) and max_runtime > 0.0 and elapsed >= max_runtime:
                    if not final_rotation_active:
                        self.get_logger().info('Explore goal reached final rotation window')
                        self.start_final_rotation('Reached requested runtime')
                        time.sleep(0.2)
                        continue
                    if not self.final_rotation_complete():
                        self.publish_final_rotation_cmd()
                        time.sleep(0.2)
                        continue

                    self.get_logger().info('Explore goal finished by requested timeout')
                    self.deactivate_and_stop(self.STATE_FINISHED)
                    goal_handle.succeed()
                    return self.make_result(
                        True,
                        final_rotation_message or 'Reached requested runtime',
                    )

                if (
                    not run_until_cancelled
                    and max_runtime > self.final_rotation_duration_sec
                    and elapsed >= max_runtime - self.final_rotation_duration_sec
                ):
                    if not final_rotation_active:
                        self.get_logger().info('Explore goal entered final rotation window')
                        self.start_final_rotation('Reached requested runtime')
                    else:
                        self.publish_final_rotation_cmd()

                time.sleep(0.2)

            self.deactivate_and_stop()
            goal_handle.abort()
            return self.make_result(False, 'ROS shutdown')

        finally:
            with self.state_lock:
                self.exploring_active = False
                self.active_goal_handle = None
                self.goal_start_time = None
                self.final_rotation_active = False
                self.final_rotation_start_time = None
                self.final_rotation_finish_message = ''
                if self.current_state != self.STATE_CANCELING:
                    self.current_state = self.STATE_IDLE
            self.stop_robot()

    # ------------------------------------------------------------
    # TF and grid helpers
    # ------------------------------------------------------------
    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time(),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Cannot get {self.global_frame}->{self.base_frame}: {e}')
            return None

    def world_to_grid(self, wx, wy, info):
        gx = int((wx - info.origin.position.x) / info.resolution)
        gy = int((wy - info.origin.position.y) / info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, info):
        wx = info.origin.position.x + (gx + 0.5) * info.resolution
        wy = info.origin.position.y + (gy + 0.5) * info.resolution
        return wx, wy

    def is_free(self, v) -> bool:
        return 0 <= v <= self.free_threshold

    # ------------------------------------------------------------
    # Frontier extraction and scoring
    # ------------------------------------------------------------
    def extract_frontiers(self, grid):
        h, w = grid.shape
        frontier_mask = np.zeros_like(grid, dtype=bool)

        free_count = 0
        frontier_count = 0

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                if not self.is_free(grid[y, x]):
                    continue

                free_count += 1
                neighbors = grid[y - 1:y + 2, x - 1:x + 2]
                if np.any(neighbors == -1):
                    frontier_mask[y, x] = True
                    frontier_count += 1

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        for y in range(h):
            for x in range(w):
                if not frontier_mask[y, x] or visited[y, x]:
                    continue

                q = deque([(x, y)])
                visited[y, x] = True
                cluster = []

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))

                    for nx in range(cx - 1, cx + 2):
                        for ny in range(cy - 1, cy + 2):
                            if not (0 <= nx < w and 0 <= ny < h):
                                continue
                            if visited[ny, nx]:
                                continue
                            if frontier_mask[ny, nx]:
                                visited[ny, nx] = True
                                q.append((nx, ny))

                if len(cluster) >= self.min_frontier_size:
                    clusters.append(cluster)

        self.latest_frontier_cluster_count = len(clusters)
        self.get_logger().info(
            f'Map stats: free_cells={free_count}, '
            f'frontier_cells={frontier_count}, clusters={len(clusters)}'
        )
        return clusters

    def pick_cluster_representative(self, cluster):
        xs = np.array([p[0] for p in cluster], dtype=float)
        ys = np.array([p[1] for p in cluster], dtype=float)
        cx = xs.mean()
        cy = ys.mean()
        return min(cluster, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)

    def backoff_goal_from_frontier(self, grid, frontier_gx, frontier_gy, robot_gx, robot_gy):
        if frontier_gx == robot_gx and frontier_gy == robot_gy:
            if self.is_free(grid[frontier_gy, frontier_gx]):
                return frontier_gx, frontier_gy
            return None

        dx = robot_gx - frontier_gx
        dy = robot_gy - frontier_gy
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            norm = 1.0

        ux = dx / norm
        uy = dy / norm
        h, w = grid.shape

        for step in range(self.goal_backoff_cells, -1, -1):
            gx = int(round(frontier_gx + ux * step))
            gy = int(round(frontier_gy + uy * step))

            if not (0 <= gx < w and 0 <= gy < h):
                continue
            if not self.is_free(grid[gy, gx]):
                continue
            return gx, gy

        return None

    def count_unknown_around(self, grid, gx, gy):
        h, w = grid.shape
        r = self.unknown_gain_radius_cells

        x0 = max(0, gx - r)
        x1 = min(w - 1, gx + r)
        y0 = max(0, gy - r)
        y1 = min(h - 1, gy + r)

        patch = grid[y0:y1 + 1, x0:x1 + 1]
        return int(np.sum(patch == -1))

    def pick_frontier_goal(self, robot_xy):
        if self.map_msg is None:
            return None

        info = self.map_msg.info
        grid = np.array(self.map_msg.data, dtype=np.int16).reshape((info.height, info.width))
        clusters = self.extract_frontiers(grid)

        if not clusters:
            self.latest_frontier_candidate_count = 0
            self.get_logger().info('No frontier clusters found')
            return None

        rx, ry = robot_xy
        robot_gx, robot_gy = self.world_to_grid(rx, ry, info)

        best_goal = None
        best_score = None
        passed_count = 0
        no_backoff_count = 0
        blacklisted_count = 0
        too_close_count = 0

        for cluster in clusters:
            frontier_gx, frontier_gy = self.pick_cluster_representative(cluster)
            backed = self.backoff_goal_from_frontier(
                grid,
                frontier_gx,
                frontier_gy,
                robot_gx,
                robot_gy,
            )
            if backed is None:
                no_backoff_count += 1
                continue

            goal_gx, goal_gy = backed
            wx, wy = self.grid_to_world(goal_gx, goal_gy, info)

            if self.is_blacklisted(wx, wy):
                blacklisted_count += 1
                continue

            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.min_goal_distance:
                too_close_count += 1
                continue

            size = len(cluster)
            unknown_count = self.count_unknown_around(grid, goal_gx, goal_gy)
            score = (
                self.distance_weight * dist
                - self.frontier_size_weight * size
                - self.unknown_gain_weight * unknown_count
            )

            passed_count += 1
            if best_goal is None or score < best_score:
                yaw = math.atan2(wy - ry, wx - rx)
                best_goal = (wx, wy, yaw, size, unknown_count)
                best_score = score

        self.latest_frontier_candidate_count = passed_count
        self.get_logger().info(f'Frontier candidates after filtering: {passed_count}')

        if best_goal is None:
            self.get_logger().info(
                'Clusters exist but all filtered out: '
                f'no_backoff={no_backoff_count}, '
                f'blacklisted={blacklisted_count}, '
                f'too_close={too_close_count}, '
                f'min_goal_distance={self.min_goal_distance:.2f}, '
                f'goal_backoff_cells={self.goal_backoff_cells}'
            )
            return None

        return best_goal

    # ------------------------------------------------------------
    # Nav2 interaction
    # ------------------------------------------------------------
    def pump_bt_navigator_state(self):
        if self.bt_navigator_state_future is not None:
            if self.bt_navigator_state_future.done():
                future = self.bt_navigator_state_future
                self.bt_navigator_state_future = None
                try:
                    result = future.result()
                except Exception as e:
                    self.bt_navigator_state = 'unknown'
                    self.bt_navigator_state_error = (
                        f'Failed to query {self.bt_navigator_state_service}: '
                        f'{type(e).__name__}: {e}'
                    )
                    return

                if result is None:
                    self.bt_navigator_state = 'unknown'
                    self.bt_navigator_state_error = (
                        f'{self.bt_navigator_state_service} returned no result.'
                    )
                    return

                self.bt_navigator_state = result.current_state.label or 'unknown'
                self.bt_navigator_state_error = None
                return

            if (
                time.monotonic() - self.bt_navigator_state_request_started
                >= self.bt_navigator_state_timeout_sec
            ):
                self.bt_navigator_state = 'unknown'
                self.bt_navigator_state_error = (
                    f'Timed out querying {self.bt_navigator_state_service}.'
                )
                self.bt_navigator_state_future = None
            return

        if not self.bt_navigator_state_client.wait_for_service(timeout_sec=0.0):
            self.bt_navigator_state = 'unknown'
            self.bt_navigator_state_error = (
                f'Lifecycle service {self.bt_navigator_state_service} is unavailable.'
            )
            return

        self.bt_navigator_state_request_started = time.monotonic()
        self.bt_navigator_state_future = self.bt_navigator_state_client.call_async(
            GetState.Request()
        )

    def get_nav2_not_ready_reason(self):
        if not self.nav_client.wait_for_server(timeout_sec=0.0):
            return f'NavigateToPose action server {self.nav_action_name} is unavailable.'

        self.pump_bt_navigator_state()

        if self.bt_navigator_state_error is not None:
            return self.bt_navigator_state_error

        if self.bt_navigator_state != 'active':
            return (
                'bt_navigator lifecycle state is '
                f"'{self.bt_navigator_state}', not 'active'."
            )

        return None

    def log_nav2_not_ready(self, reason):
        now = time.monotonic()
        if (
            reason != self.last_nav2_wait_reason
            or now - self.last_nav2_wait_log_time >= 5.0
        ):
            self.last_nav2_wait_log_time = now
            self.last_nav2_wait_reason = reason
            self.get_logger().warn(f'Waiting for Nav2 readiness: {reason}')

    def send_nav_goal(self, x, y, yaw, size, unknown_count):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        with self.state_lock:
            if (
                not self.exploring_active
                or self.finish_requested
                or self.final_rotation_active
            ):
                return

            self.nav_goal_in_progress = True
            self.current_goal_xy = (float(x), float(y))
            self.current_nav_goal_handle = None
            self.nav_cancel_requested = False
            self.latest_chosen_frontier_size = int(size)
            self.latest_unknown_gain = int(unknown_count)
            self.goals_sent += 1
            self.current_state = self.STATE_NAVIGATING

        self.get_logger().info(
            f'Sending frontier goal #{self.goals_sent}: '
            f'x={x:.2f}, y={y:.2f}, size={size}, unknown_gain={unknown_count}'
        )

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        try:
            nav_goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send NavigateToPose goal: {e}')
            reason = self.get_nav2_not_ready_reason()
            if reason is not None:
                self.log_nav2_not_ready(reason)
            with self.state_lock:
                self.nav_goal_in_progress = False
                self.current_nav_goal_handle = None
                self.current_goal_xy = None
                self.goals_failed += 1
                if self.exploring_active and not self.final_rotation_active:
                    self.current_state = (
                        self.STATE_WAITING_FOR_NAV2
                        if reason is not None
                        else self.STATE_PLANNING
                    )
            return

        if not nav_goal_handle.accepted:
            reason = self.get_nav2_not_ready_reason()
            if reason is None:
                self.get_logger().warn('Frontier NavigateToPose goal rejected')
            else:
                self.get_logger().warn(
                    'Frontier NavigateToPose goal rejected while Nav2 is not ready: '
                    f'{reason}'
                )
                self.log_nav2_not_ready(reason)
            with self.state_lock:
                self.nav_goal_in_progress = False
                self.current_nav_goal_handle = None
                self.current_goal_xy = None
                self.goals_failed += 1
                if self.exploring_active and not self.final_rotation_active:
                    self.current_state = (
                        self.STATE_WAITING_FOR_NAV2
                        if reason is not None
                        else self.STATE_PLANNING
                    )
            return

        self.get_logger().info('Frontier NavigateToPose goal accepted')
        with self.state_lock:
            self.current_nav_goal_handle = nav_goal_handle
            cancel_immediately = (
                not self.exploring_active
                or self.nav_cancel_requested
                or self.final_rotation_active
            )

        if cancel_immediately:
            self.get_logger().info(
                'Canceling NavigateToPose goal accepted after exploration stopped'
            )
            self.cancel_nav_goal_handle(nav_goal_handle)

        result_future = nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_goal_result_callback)

    def nav_goal_result_callback(self, future):
        try:
            result_wrap = future.result()
            status = result_wrap.status
        except Exception as e:
            self.get_logger().error(f'Failed to get NavigateToPose result: {e}')
            status = GoalStatus.STATUS_ABORTED

        with self.state_lock:
            goal_xy = self.current_goal_xy
            cancel_requested = self.nav_cancel_requested
            still_exploring = self.exploring_active
            final_rotation_active = self.final_rotation_active

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier goal succeeded')
            with self.state_lock:
                self.goals_succeeded += 1
                self.no_frontier_count = 0
        else:
            self.get_logger().warn(f'Frontier goal finished with status={status}')
            with self.state_lock:
                self.goals_failed += 1
            if goal_xy is not None and still_exploring and not cancel_requested:
                self.add_blacklist(*goal_xy)

        with self.state_lock:
            self.nav_goal_in_progress = False
            self.current_nav_goal_handle = None
            self.current_goal_xy = None
            self.nav_cancel_requested = False
            if self.exploring_active and not final_rotation_active:
                self.current_state = self.STATE_PLANNING

    def cancel_nav_goal_handle(self, nav_goal_handle):
        try:
            cancel_future = nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda _future: self.get_logger().info(
                    'Cancel request sent to NavigateToPose goal'
                )
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to cancel NavigateToPose goal: {e}')

    def cancel_current_nav_goal(self):
        with self.state_lock:
            nav_goal_handle = self.current_nav_goal_handle
            self.nav_cancel_requested = True

        if nav_goal_handle is None:
            return

        self.cancel_nav_goal_handle(nav_goal_handle)

    # ------------------------------------------------------------
    # Main exploration planner loop
    # ------------------------------------------------------------
    def plan_once(self):
        with self.state_lock:
            active = self.exploring_active
            finish_requested = self.finish_requested
            nav_goal_in_progress = self.nav_goal_in_progress
            final_rotation_active = self.final_rotation_active

        if not active or finish_requested or final_rotation_active:
            return

        self.cleanup_blacklist()

        if self.map_msg is None:
            self.set_state(self.STATE_WAITING_FOR_MAP)
            return

        if nav_goal_in_progress:
            return

        nav2_not_ready_reason = self.get_nav2_not_ready_reason()
        if nav2_not_ready_reason is not None:
            self.set_state(self.STATE_WAITING_FOR_NAV2)
            self.log_nav2_not_ready(nav2_not_ready_reason)
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.set_state(self.STATE_WAITING_FOR_TF)
            return

        with self.state_lock:
            self.latest_robot_xy = robot_xy
            self.current_state = self.STATE_PLANNING

        self.get_logger().info(f'Robot pose in map: x={robot_xy[0]:.2f}, y={robot_xy[1]:.2f}')

        goal = self.pick_frontier_goal(robot_xy)
        if goal is None:
            with self.state_lock:
                self.no_frontier_count += 1
                count = self.no_frontier_count
                self.current_state = self.STATE_NO_FRONTIER

            if self.finish_when_no_frontiers and count >= self.no_frontier_finish_count:
                self.request_finish(True, 'No reachable frontier left')
            return

        with self.state_lock:
            self.no_frontier_count = 0

        gx, gy, yaw, size, unknown_count = goal
        self.get_logger().info(f'Chosen frontier size={size}, unknown_gain={unknown_count}')
        self.send_nav_goal(gx, gy, yaw, size, unknown_count)


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
        node.deactivate_and_stop()
        try:
            node._action_server.destroy()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
