import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

from rover_interface.action import NavigateToPos

class NavigationServiceNode(Node):
    def __init__(self):
        super().__init__('navigation_service_node')

        use_sim_time = bool(self.declare_parameter('use_sim_time', False).value)
        self.nav2_server_wait_timeout_sec = float(
            self.declare_parameter('nav2_server_wait_timeout_sec', 10.0).value
        )

        # Initialize Nav2 controller
        # 1. 初始化 Nav2 控制器
        self.navigator = BasicNavigator(node_name='navigation_service_navigator')
        self.navigator.set_parameters(
            [Parameter('use_sim_time', value=use_sim_time)]
        )
        self._nav2_ready = False
        self._last_nav2_wait_log_time = 0.0
        self._nav2_ready_timer = self.create_timer(1.0, self._refresh_nav2_readiness)

        # Initialize Action Server (waiting for goals sent by code)
        # 2. 初始化 Action Server (等待代码发送的目标)
        self._action_server = ActionServer(
            self,
            NavigateToPos,
            'navigate_to_pos',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Add testing tool: Listen to target points clicked in the RViz interface
        # 3. 增加实体测试利器：监听 RViz 界面鼠标点击的目标
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.rviz_goal_callback,
            10,
        )

        self.get_logger().info('Physical robot navigation node started!')
        self.get_logger().info(
            'Action server /navigate_to_pos is available; goals will be accepted '
            'once Nav2 backend servers are ready.'
        )

    def _nav2_servers_ready(self):
        return all(
            [
                self.navigator.nav_to_pose_client.server_is_ready(),
                self.navigator.compute_path_to_pose_client.server_is_ready(),
                self.navigator.follow_path_client.server_is_ready(),
            ]
        )

    def _refresh_nav2_readiness(self):
        nav2_ready = self._nav2_servers_ready()
        if nav2_ready and not self._nav2_ready:
            self._nav2_ready = True
            self.get_logger().info('Nav2 backend is ready to accept goals.')
        elif not nav2_ready and self._nav2_ready:
            self._nav2_ready = False
            self.get_logger().warning('Nav2 backend became unavailable.')

    def _wait_for_nav2_backend(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self._refresh_nav2_readiness()
            if self._nav2_ready:
                return True

            now = time.monotonic()
            if now - self._last_nav2_wait_log_time >= 5.0:
                self.get_logger().warning(
                    'Waiting for Nav2 backend action servers '
                    '(navigate_to_pose / compute_path_to_pose / follow_path)...'
                )
                self._last_nav2_wait_log_time = now
            time.sleep(0.1)

        return self._nav2_ready

    def goal_callback(self, _goal_request):
        if not self._nav2_ready:
            self.get_logger().warning(
                'Rejecting navigate_to_pos goal because Nav2 backend is not ready yet.'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel_callback(_goal_handle):
        return CancelResponse.ACCEPT

    def rviz_goal_callback(self, msg):
        # Process target points from RViz mouse clicks
        # 处理来自 RViz 鼠标点击的目标
        if not self._wait_for_nav2_backend(self.nav2_server_wait_timeout_sec):
            self.get_logger().warning(
                'Ignoring RViz goal because Nav2 backend is unavailable.'
            )
            return
        self.get_logger().info(f'📍 Received RViz click goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.navigator.goToPose(msg)

    async def execute_callback(self, goal_handle):
        request_point = goal_handle.request.target_pos
        self.get_logger().info(f'📍 Received Action navigation request, frame_id: {request_point.header.frame_id}')

        try:
            if not self._wait_for_nav2_backend(self.nav2_server_wait_timeout_sec):
                result = NavigateToPos.Result()
                result.success = False
                result.final_distance = float('inf')
                result.message = (
                    'Nav2 backend did not become ready before timeout.'
                )
                goal_handle.abort()
                return result

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = request_point.point.x
            goal_pose.pose.position.y = request_point.point.y
            goal_pose.pose.position.z = 0.0
            goal_pose.pose.orientation.w = 1.0

            self.navigator.goToPose(goal_pose)
            
            while not self.navigator.isTaskComplete():
                feedback = NavigateToPos.Feedback()
                nav_feedback = self.navigator.getFeedback()
                if nav_feedback:
                    feedback.distance_remaining = nav_feedback.distance_remaining
                    goal_handle.publish_feedback(feedback)

                time.sleep(0.1) 

            nav_result = self.navigator.getResult()
            result = NavigateToPos.Result()

            final_fb = self.navigator.getFeedback()
            result.final_distance = float(final_fb.distance_remaining) if final_fb else 0.0

            if nav_result == TaskResult.SUCCEEDED:
                result.success = True
                result.message = "Goal Reached!"
                goal_handle.succeed()
            else:
                result.success = False
                result.message = f"Nav Failed with status: {nav_result}"
                goal_handle.abort()
            return result

        except Exception as e:
            self.get_logger().error(f'Exception during navigation: {str(e)}')
            goal_handle.abort()
            return NavigateToPos.Result(success=False, message=str(e))

    def destroy_node(self):
        self._action_server.destroy()
        self.navigator.destroy_node()
        super().destroy_node()

def main(args=None):
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
        rclpy.shutdown()
