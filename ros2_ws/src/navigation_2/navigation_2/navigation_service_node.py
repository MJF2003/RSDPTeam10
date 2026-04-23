"""Navigation action wrapper around Nav2 for the rover stack."""

import time

from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from rover_interface.action import NavigateToPos


class NavigationServiceNode(Node):
    """Expose the rover navigation action and forward goals into Nav2."""

    def __init__(self):
        """Initialize the wrapper node and backend readiness checks."""
        super().__init__('navigation_service_node')

        self._callback_group = ReentrantCallbackGroup()

        use_sim_time = bool(self._declare_or_get_parameter('use_sim_time', False))
        self.nav2_server_wait_timeout_sec = float(
            self._declare_or_get_parameter('nav2_server_wait_timeout_sec', 10.0)
        )
        self.map_frame = str(self._declare_or_get_parameter('map_frame', 'map'))
        self.robot_base_frame = str(
            self._declare_or_get_parameter('robot_base_frame', 'base_link')
        )
        self.tf_lookup_timeout = Duration(
            seconds=float(
                self._declare_or_get_parameter('tf_lookup_timeout_sec', 0.2)
            )
        )
        self.bt_navigator_state_timeout_sec = float(
            self._declare_or_get_parameter('bt_navigator_state_timeout_sec', 0.5)
        )

        self.navigator = BasicNavigator(node_name='navigation_service_navigator')
        self.navigator.set_parameters([Parameter('use_sim_time', value=use_sim_time)])

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._map_received = False
        self._bt_navigator_state = 'unknown'
        self._bt_navigator_state_error = None
        self._bt_navigator_state_future = None
        self._bt_navigator_state_request_started = 0.0
        self._nav2_ready = False
        self._nav2_not_ready_reason = 'Waiting for initial Nav2 status check.'
        self._last_nav2_wait_log_time = 0.0
        self._last_nav2_wait_reason = ''

        self._bt_navigator_state_client = self.create_client(
            GetState,
            'bt_navigator/get_state',
            callback_group=self._callback_group,
        )

        self._nav2_ready_timer = self.create_timer(
            0.5,
            self._refresh_nav2_readiness,
            callback_group=self._callback_group,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            10,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            NavigateToPos,
            'navigate_to_pos',
            self.execute_callback,
            callback_group=self._callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_goal_callback,
            10,
            callback_group=self._callback_group,
        )

        self.get_logger().info('Physical robot navigation node started!')
        self.get_logger().info(
            'Action server /navigate_to_pos is available immediately; goals '
            'wait for Nav2 backend readiness before execution.'
        )

    def _declare_or_get_parameter(self, name, default_value):
        """Return a parameter value without failing if launch already declared it."""
        if self.has_parameter(name):
            return self.get_parameter(name).value

        return self.declare_parameter(name, default_value).value

    def _map_callback(self, _msg):
        if self._map_received:
            return

        self._map_received = True
        self.get_logger().info('Received first /map message.')

    def _pump_bt_navigator_state(self):
        if self._bt_navigator_state_future is not None:
            if self._bt_navigator_state_future.done():
                future = self._bt_navigator_state_future
                self._bt_navigator_state_future = None
                try:
                    result = future.result()
                except Exception as exc:  # pragma: no cover - ROS future errors
                    self._bt_navigator_state = 'unknown'
                    self._bt_navigator_state_error = (
                        'Failed to query bt_navigator/get_state: '
                        f'{type(exc).__name__}: {exc}'
                    )
                    return

                if result is None:
                    self._bt_navigator_state = 'unknown'
                    self._bt_navigator_state_error = (
                        'bt_navigator/get_state returned no result.'
                    )
                    return

                self._bt_navigator_state = result.current_state.label or 'unknown'
                self._bt_navigator_state_error = None
                return

            if (
                time.monotonic() - self._bt_navigator_state_request_started
                >= self.bt_navigator_state_timeout_sec
            ):
                self._bt_navigator_state = 'unknown'
                self._bt_navigator_state_error = (
                    'Timed out querying bt_navigator/get_state.'
                )
                self._bt_navigator_state_future = None
            return

        if not self._bt_navigator_state_client.wait_for_service(timeout_sec=0.0):
            self._bt_navigator_state = 'unknown'
            self._bt_navigator_state_error = (
                'Lifecycle service bt_navigator/get_state is unavailable.'
            )
            return

        self._bt_navigator_state_request_started = time.monotonic()
        self._bt_navigator_state_future = self._bt_navigator_state_client.call_async(
            GetState.Request()
        )

    def _get_nav2_not_ready_reason(self):
        self._pump_bt_navigator_state()

        if self._bt_navigator_state_error is not None:
            return self._bt_navigator_state_error

        if self._bt_navigator_state != 'active':
            return (
                'bt_navigator lifecycle state is '
                f"'{self._bt_navigator_state}', not 'active'."
            )

        if not self.navigator.nav_to_pose_client.server_is_ready():
            return "Action server 'navigate_to_pose' is unavailable."

        if not self.navigator.compute_path_to_pose_client.server_is_ready():
            return "Action server 'compute_path_to_pose' is unavailable."

        if not self.navigator.follow_path_client.server_is_ready():
            return "Action server 'follow_path' is unavailable."

        if self.count_publishers('/map') == 0:
            return 'No publishers detected on /map yet.'

        if not self._map_received:
            return 'Waiting for first /map message.'

        try:
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                Time(),
                timeout=self.tf_lookup_timeout,
            )
        except TransformException as exc:
            return (
                f"Transform '{self.map_frame}' -> "
                f"'{self.robot_base_frame}' unavailable: {exc}"
            )

        return None

    def _refresh_nav2_readiness(self):
        reason = self._get_nav2_not_ready_reason()
        nav2_ready = reason is None

        if nav2_ready and not self._nav2_ready:
            self._nav2_ready = True
            self._nav2_not_ready_reason = ''
            self.get_logger().info('Nav2 backend is ready to accept goals.')
            return

        if not nav2_ready and self._nav2_ready:
            self._nav2_ready = False
            self._nav2_not_ready_reason = reason
            self.get_logger().warning(
                f'Nav2 backend became unavailable: {reason}'
            )
            return

        self._nav2_not_ready_reason = reason or ''

    def _wait_for_nav2_backend(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._refresh_nav2_readiness()
            if self._nav2_ready:
                return True

            now = time.monotonic()
            reason = self._nav2_not_ready_reason or 'Nav2 backend is not ready yet.'
            if (
                reason != self._last_nav2_wait_reason
                or now - self._last_nav2_wait_log_time >= 5.0
            ):
                self.get_logger().warning(f'Waiting for Nav2 backend: {reason}')
                self._last_nav2_wait_log_time = now
                self._last_nav2_wait_reason = reason
            time.sleep(0.1)

        return self._nav2_ready

    def goal_callback(self, _goal_request):
        """Accept goals and gate execution inside execute_callback."""
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        """Accept goal cancellation requests."""
        return CancelResponse.ACCEPT

    def rviz_goal_callback(self, msg):
        """Forward RViz click goals once the Nav2 backend is ready."""
        if not self._wait_for_nav2_backend(self.nav2_server_wait_timeout_sec):
            self.get_logger().warning(
                'Ignoring RViz goal because Nav2 backend is unavailable: '
                f'{self._nav2_not_ready_reason}'
            )
            return

        self.get_logger().info(
            f'Received RViz click goal: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )
        if not self.navigator.goToPose(msg):
            self.get_logger().warning('Nav2 rejected the RViz goal request.')

    async def execute_callback(self, goal_handle):
        """Execute a NavigateToPos goal through Nav2 once ready."""
        request_point = goal_handle.request.target_pos
        goal_frame = request_point.header.frame_id.strip() or self.map_frame
        self.get_logger().info(
            'Received navigation action request '
            f"in frame '{goal_frame}'."
        )

        try:
            if not self._wait_for_nav2_backend(self.nav2_server_wait_timeout_sec):
                result = NavigateToPos.Result()
                result.success = False
                result.final_distance = float('inf')
                result.message = (
                    'Nav2 backend did not become ready before timeout. '
                    f'Last issue: {self._nav2_not_ready_reason}'
                )
                goal_handle.abort()
                return result

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = goal_frame
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = request_point.point.x
            goal_pose.pose.position.y = request_point.point.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            if not self.navigator.goToPose(goal_pose):
                result = NavigateToPos.Result()
                result.success = False
                result.final_distance = float('inf')
                result.message = 'Nav2 rejected the requested goal.'
                goal_handle.abort()
                return result

            while not self.navigator.isTaskComplete():
                feedback = NavigateToPos.Feedback()
                nav_feedback = self.navigator.getFeedback()
                if nav_feedback:
                    feedback.distance_remaining = nav_feedback.distance_remaining
                    goal_handle.publish_feedback(feedback)

                time.sleep(0.1)

            nav_result = self.navigator.getResult()
            result = NavigateToPos.Result()

            final_feedback = self.navigator.getFeedback()
            if final_feedback:
                result.final_distance = float(final_feedback.distance_remaining)
            else:
                result.final_distance = 0.0

            if nav_result == TaskResult.SUCCEEDED:
                result.success = True
                result.message = 'Goal reached!'
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f'Nav failed with status: {nav_result}'
                goal_handle.abort()
            return result

        except Exception as exc:
            self.get_logger().error(f'Exception during navigation: {exc}')
            goal_handle.abort()
            return NavigateToPos.Result(success=False, message=str(exc))

    def destroy_node(self):
        """Destroy the action server and embedded navigator node."""
        self._action_server.destroy()
        self.navigator.destroy_node()
        super().destroy_node()


def main(args=None):
    """Run the navigation wrapper node."""
    rclpy.init(args=args)
    node = NavigationServiceNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.navigator)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node.navigator)
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
