#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布从base_link到camera_link的静态变换
        # 假设相机安装在机器人前方0.1米，高度0.2米，正向前方
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        
        # 相机位置：机器人前方0.1米，高度0.2米
        transform.transform.translation.x = 0.1  # 前方0.1米
        transform.transform.translation.y = 0.0  # 中心
        transform.transform.translation.z = 0.2  # 高度0.2米
        
        # 没有旋转（相机正向前方）
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Publish Static TF Transform: base_link -> camera_link")
        self.get_logger().info("Camera Position: 前方0.1米, 高度0.2米, 正向前方")

def main():
    rclpy.init()
    node = StaticTFPublisher()
    
    print("Static TF Publisher Running...")
    print("TF L: base_link → camera_link → camera_color_frame → camera_color_optical_frame")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStop Static TF Publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()