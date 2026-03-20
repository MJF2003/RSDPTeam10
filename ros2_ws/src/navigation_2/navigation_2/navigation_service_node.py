import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from rover_interface.action import NavigateToPos

class NavigationServiceNode(Node):
    def __init__(self):
        super().__init__('navigation_service_node')
        
        # 1. Initialize Nav2 controller
        # 1. 初始化 Nav2 控制器 
        self.navigator = BasicNavigator()
        
        # 2. Initialize TF2 listener for handling camera-to-map transformations
        # 2. 初始化 TF2 监听器，用于处理相机到地图的转换 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 3. Initialize Action Server
        # 3. 初始化 Action Server
        # 名称必须为 'navigation_stub' 以匹配 rover_controller.py 的请求
        self._action_server = ActionServer(
            self,
            NavigateToPos,
            'navigate_to_block', 
            self.execute_callback)
        
        self.get_logger().info('Navigation service node started, supporting real-time coordinate transformation, waiting for goals...')

    async def execute_callback(self, goal_handle):
        """
        Action 执行主逻辑
        """
        # 获取 Simulation 组发送的 PointStamped 目标
        request_msg = goal_handle.request.target_pos 
        source_frame = request_msg.header.frame_id
        target_frame = 'map'
        
        self.get_logger().info(f'Received coordinates, source frame: {source_frame}')

        # 将 PointStamped 转换为 Pose 以便进行 TF 转换
        target_pose = Pose()
        target_pose.position = request_msg.point
        target_pose.orientation.w = 1.0 # 默认朝向
        
        try:
            # A. Wait for and retrieve the latest coordinate transformation
            # A. 等待并获取最新的坐标变换关系 
            # lookup_transform automatically finds the transform chain: source_frame -> ... -> base_link -> odom -> map
            # lookup_transform 会自动查找变换链：source_frame -> ... -> base_link -> odom -> map
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0))
            
            # B. Execute coordinate transformation
            # B. 执行坐标转换 
            map_pose = do_transform_pose(target_pose, transform)

            # C. Construct the final goal point to be sent to Nav2
            # C. 构建发送给 Nav2 的最终目标点 
            nav_goal = PoseStamped()
            nav_goal.header.frame_id = target_frame
            nav_goal.header.stamp = self.get_clock().now().to_msg()
            nav_goal.pose = map_pose

            self.get_logger().info(f'Transformation successful: x={source_frame} -> map')

            # D. Send navigation command
            # D. 发送导航指令 
            self.navigator.goToPose(nav_goal)

            # E. Loop to monitor progress
            # E. 循环监控进度 
            feedback_msg = NavigateToPos.Feedback()
            while not self.navigator.isTaskComplete():
                nav_feedback = self.navigator.getFeedback()
                if nav_feedback:
                    feedback_msg.distance_remaining = nav_feedback.distance_remaining
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().info(f'Distance remaining: {feedback_msg.distance_remaining:.2f} meters', throttle_duration_sec=1.0)
                
                rclpy.spin_once(self, timeout_sec=0.1)

            # F. Check final result
            # F. 检查最终结果 
            nav_result = self.navigator.getResult()
            result = NavigateToPos.Result()

            final_feedback = self.navigator.getFeedback()
            result.final_distance = final_feedback.distance_remaining if final_feedback else 0.0
            if nav_result == TaskResult.SUCCEEDED:
                result.success = True
                result.message = "Navigation Succeeded"
                self.get_logger().info('✅ Successfully arrived at the goal area!')
                goal_handle.succeed()
            else:
                result.success = False
                result.message = "Navigation Failed or Canceled"
                self.get_logger().error('❌ Navigation failed')
                goal_handle.abort()
            return result

        except Exception as e:
            self.get_logger().error(f'Unable to complete coordinate transformation: {str(e)}')
            self.get_logger().warn('Please confirm if SLAM is running (check for map frame) and if camera TF is being published')
            goal_handle.abort()
            return NavigateToPos.Result(success=False, message=str(e))

def main(args=None):
    rclpy.init(args=args)
    node = NavigationServiceNode()
    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()