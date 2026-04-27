#!/usr/bin/env python3
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion

import tf2_ros


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('global_frame', 'map')

        # Cartographer 的 free 不一定严格等于 0，所以这里默认设宽松一些
        self.declare_parameter('free_threshold', 60)
        self.declare_parameter('occupied_threshold', 65)

        # Frontier 过滤参数
        self.declare_parameter('min_frontier_size', 20)
        self.declare_parameter('min_goal_distance', 1.0)
        self.declare_parameter('goal_backoff_cells', 8)

        # 失败目标黑名单
        self.declare_parameter('blacklist_radius', 0.60)
        self.declare_parameter('blacklist_time', 30.0)

        # 规划周期
        self.declare_parameter('plan_period', 3.0)

        # 目标评分参数：更偏向未知区域，而不是只选最近目标
        self.declare_parameter('unknown_gain_radius_cells', 10)
        self.declare_parameter('unknown_gain_weight', 0.025)
        self.declare_parameter('frontier_size_weight', 0.03)
        self.declare_parameter('distance_weight', 0.80)

        self.map_topic = self.get_parameter('map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value

        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)

        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.min_goal_distance = float(self.get_parameter('min_goal_distance').value)
        self.goal_backoff_cells = int(self.get_parameter('goal_backoff_cells').value)

        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.blacklist_time = float(self.get_parameter('blacklist_time').value)
        self.plan_period = float(self.get_parameter('plan_period').value)

        self.unknown_gain_radius_cells = int(self.get_parameter('unknown_gain_radius_cells').value)
        self.unknown_gain_weight = float(self.get_parameter('unknown_gain_weight').value)
        self.frontier_size_weight = float(self.get_parameter('frontier_size_weight').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)

        # ------------------------------------------------------------
        # Map subscription
        # ------------------------------------------------------------
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        # ------------------------------------------------------------
        # TF and Nav2 action client
        # ------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ------------------------------------------------------------
        # Runtime state
        # ------------------------------------------------------------
        self.map_msg = None
        self.goal_in_progress = False
        self.current_goal_xy = None
        self.current_goal_handle = None
        self.blacklist = []   # [(x, y, expire_time_sec)]

        self.timer = self.create_timer(self.plan_period, self.plan_once)

        self.get_logger().info('frontier_explorer started')

    # ------------------------------------------------------------
    # Time / blacklist helpers
    # ------------------------------------------------------------
    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def cleanup_blacklist(self):
        now = self.now_sec()
        self.blacklist = [b for b in self.blacklist if b[2] > now]

    def is_blacklisted(self, x, y):
        for bx, by, _ in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def add_blacklist(self, x, y):
        expire = self.now_sec() + self.blacklist_time
        self.blacklist.append((x, y, expire))

    # ------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    # ------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------
    def get_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f'Cannot get {self.global_frame}->{self.base_frame}: {e}')
            return None

    # ------------------------------------------------------------
    # Grid/world conversion
    # ------------------------------------------------------------
    def world_to_grid(self, wx, wy, info):
        gx = int((wx - info.origin.position.x) / info.resolution)
        gy = int((wy - info.origin.position.y) / info.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy, info):
        wx = info.origin.position.x + (gx + 0.5) * info.resolution
        wy = info.origin.position.y + (gy + 0.5) * info.resolution
        return wx, wy

    # ------------------------------------------------------------
    # Occupancy helpers
    # ------------------------------------------------------------
    def is_unknown(self, v):
        return v == -1

    def is_free(self, v):
        return 0 <= v <= self.free_threshold

    def is_occupied(self, v):
        return v >= self.occupied_threshold

    # ------------------------------------------------------------
    # Frontier extraction
    # ------------------------------------------------------------
    def extract_frontiers(self, grid):
        """
        Frontier:
        - current cell is FREE
        - at least one of its 8-neighbors is UNKNOWN
        """
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

                q = deque()
                q.append((x, y))
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

        self.get_logger().info(
            f'Map stats: free_cells={free_count}, '
            f'frontier_cells={frontier_count}, clusters={len(clusters)}'
        )

        return clusters

    # ------------------------------------------------------------
    # Goal selection helpers
    # ------------------------------------------------------------
    def pick_cluster_representative(self, cluster):
        xs = np.array([p[0] for p in cluster], dtype=float)
        ys = np.array([p[1] for p in cluster], dtype=float)

        cx = xs.mean()
        cy = ys.mean()

        rep = min(cluster, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        return rep

    def backoff_goal_from_frontier(self, grid, frontier_gx, frontier_gy, robot_gx, robot_gy):
        """
        Move a few cells from frontier back toward robot so goal is
        inside free space instead of right on unknown boundary.
        """
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
        """
        Count unknown cells around candidate goal.
        More unknown cells means this goal has higher exploration value.
        """
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
        w = info.width
        h = info.height

        grid = np.array(self.map_msg.data, dtype=np.int16).reshape((h, w))
        clusters = self.extract_frontiers(grid)

        if not clusters:
            self.get_logger().info('No frontier clusters found')
            return None

        rx, ry = robot_xy
        robot_gx, robot_gy = self.world_to_grid(rx, ry, info)

        best_goal = None
        best_score = None
        passed_count = 0

        for cluster in clusters:
            frontier_gx, frontier_gy = self.pick_cluster_representative(cluster)

            backed = self.backoff_goal_from_frontier(
                grid,
                frontier_gx,
                frontier_gy,
                robot_gx,
                robot_gy
            )

            if backed is None:
                continue

            goal_gx, goal_gy = backed
            wx, wy = self.grid_to_world(goal_gx, goal_gy, info)

            if self.is_blacklisted(wx, wy):
                continue

            dist = math.hypot(wx - rx, wy - ry)
            if dist < self.min_goal_distance:
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

        self.get_logger().info(f'Frontier candidates after filtering: {passed_count}')

        if best_goal is None:
            self.get_logger().info('Clusters exist but all filtered out')
            return None

        return best_goal

    # ------------------------------------------------------------
    # Nav2 interaction
    # ------------------------------------------------------------
    def send_nav_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.goal_in_progress = True
        self.current_goal_xy = (x, y)

        self.get_logger().info(f'Sending frontier goal: x={x:.2f}, y={y:.2f}')

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Frontier goal rejected')
            if self.current_goal_xy is not None:
                self.add_blacklist(*self.current_goal_xy)
            self.goal_in_progress = False
            self.current_goal_xy = None
            self.current_goal_handle = None
            return

        self.get_logger().info('Frontier goal accepted')
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result_wrap = future.result()
        status = result_wrap.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier goal succeeded')
        else:
            self.get_logger().warn(f'Frontier goal failed with status={status}')
            if self.current_goal_xy is not None:
                self.add_blacklist(*self.current_goal_xy)

        self.goal_in_progress = False
        self.current_goal_xy = None
        self.current_goal_handle = None

    # ------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------
    def plan_once(self):
        self.cleanup_blacklist()

        if self.map_msg is None:
            return

        if self.goal_in_progress:
            return

        if not self.nav_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warn('navigate_to_pose action server not ready')
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            return

        self.get_logger().info(
            f'Robot pose in map: x={robot_xy[0]:.2f}, y={robot_xy[1]:.2f}'
        )

        goal = self.pick_frontier_goal(robot_xy)
        if goal is None:
            return

        gx, gy, yaw, size, unknown_count = goal

        self.get_logger().info(
            f'Chosen frontier size={size}, unknown_gain={unknown_count}'
        )

        self.send_nav_goal(gx, gy, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()