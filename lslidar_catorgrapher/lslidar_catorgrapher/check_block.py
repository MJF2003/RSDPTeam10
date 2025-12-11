#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        # 订阅激光雷达话题
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, scan_data: LaserScan):
        ranges = scan_data.ranges
        angle_min = scan_data.angle_min
        angle_increment = scan_data.angle_increment

        # 定义前方范围：-30° ~ +30°
        front_min_deg = -30
        front_max_deg = 30
        obstacle_detected = False

        for i, distance in enumerate(ranges):
            angle_rad = angle_min + i * angle_increment
            angle_deg = angle_rad * 180.0 / 3.14159

            if front_min_deg <= angle_deg <= front_max_deg:
                if distance < 1.0:  # 阈值：小于 1 米认为有障碍物
                    obstacle_detected = True
                    break

        if obstacle_detected:
            self.get_logger().info("前方有障碍物！")
        else:
            self.get_logger().info("前方畅通无阻")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

