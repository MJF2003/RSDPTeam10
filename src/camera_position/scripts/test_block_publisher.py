#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from leo_navigation.msg import BlockList, DetectedBlock
from std_msgs.msg import Header
import time

class TestBlockPublisher(Node):
    def __init__(self):
        super().__init__('test_block_publisher')
        self.publisher_ = self.create_publisher(BlockList, '/detected_blocks', 10)
        self.timer = self.create_timer(3.0, self.publish_test_blocks)
        self.counter = 0
        
    def publish_test_blocks(self):
        self.counter += 1
        block_list = BlockList()
        
        # 创建测试方块1 - 在图像中心，深度1.5米
        block1 = DetectedBlock()
        block1.block_id = f"block_center_{self.counter}"
        block1.bbox = [646.91, 371.37, 50.0, 50.0]  # 图像中心位置
        block1.depth = 1.0  # 1.0米远
        block1.confidence = 0.95
        
        # 创建测试方块2 - 在图像左侧，深度1.5米
        block2 = DetectedBlock()
        block2.block_id = f"block_corner_{self.counter}"
        block2.bbox = [400.0, 371.37, 50.0, 50.0]  # 左侧位置
        block2.depth = 1.5  # 1.5米远
        block2.confidence = 0.85
        
        block_list.blocks = [block1, block2]
        block_list.header = Header()
        block_list.header.stamp = self.get_clock().now().to_msg()
        block_list.header.frame_id = "camera_frame"
        
        self.publisher_.publish(block_list)
        self.get_logger().info(f'Send {len(block_list.blocks)} test block')

def main():
    rclpy.init()
    node = TestBlockPublisher()
    print("Test publisher activated, publishing test data every 3 seconds....")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()