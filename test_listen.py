#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TestListener(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vision/target_pose',
            self.listener_callback,
            10)
        print("測試程式啟動！我沒有連接 MoveIt，我專心聽訊號...")

    def listener_callback(self, msg):
        print(f"收到訊號了！X={msg.pose.position.x}")

def main(args=None):
    rclpy.init(args=args)
    node = TestListener()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

