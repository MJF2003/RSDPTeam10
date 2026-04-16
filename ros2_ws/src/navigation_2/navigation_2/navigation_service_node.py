import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import asyncio
import time

from rover_interface.action import NavigateToPos

class NavigationServiceNode(Node):
    def __init__(self):
        super().__init__('navigation_service_node')
        
        # Initialize Nav2 controller
        # 1. 初始化 Nav2 控制器 
        self.navigator = BasicNavigator()
        
        # Initialize Action Server (waiting for goals sent by code)
        # 2. 初始化 Action Server (等待代码发送的目标)
        self._action_server = ActionServer(
            self,
            NavigateToPos,
            'navigate_to_block', 
            self.execute_callback)
            
        # Add testing tool: Listen to target points clicked in the RViz interface
        # 3. 增加实体测试利器：监听 RViz 界面鼠标点击的目标
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.rviz_goal_callback, 
            10)
        
        self.get_logger().info('Physical robot navigation node started!')
        self.get_logger().info('You can send targets via Action or use "2D Goal Pose" in RViz!')

    def rviz_goal_callback(self, msg):
        # Process target points from RViz mouse clicks
        # 处理来自 RViz 鼠标点击的目标
        self.get_logger().info(f'📍 Received RViz click goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.navigator.goToPose(msg)

    async def execute_callback(self, goal_handle):
        request_point = goal_handle.request.target_pos
        self.get_logger().info(f'📍 Received Action navigation request, frame_id: {request_point.header.frame_id}')

        try:
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
