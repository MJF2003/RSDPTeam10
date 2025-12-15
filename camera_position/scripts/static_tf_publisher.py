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
        # Publish static transformation from base_link to camera_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'camera_link'
        
        # 相机位置：机器人前方0.1米，高度0.2米
        # Camera position: 0.1 metres in front of the robot, at a height of 0.2 metres
        transform.transform.translation.x = 0.1  # front 0.1m
        transform.transform.translation.y = 0.0  # center
        transform.transform.translation.z = 0.2  # height 0.2m
        
        # 没有旋转（相机正向前方）
        # No rotation (camera facing forward)
        transform.transform.rotation.w = 1.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Publish Static TF Transform: base_link -> camera_link")
        self.get_logger().info("Camera Position: 0.1 metres ahead, 0.2 metres high, moving directly forward")

def main(args=None):
    try:
        rclpy.init(args=args)

        node = StaticTFPublisher()
        
        print("Static TF Publisher Running...")
        print("TF L: base_link → camera_link → camera_color_frame → camera_color_optical_frame")
    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()