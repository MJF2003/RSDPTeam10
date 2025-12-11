#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_pose(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            
            # 小车 WIFI
            # angle speed
            
            # 位置信息
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
            
            # 姿态信息（四元数）
            q = trans.transform.rotation
            
            return [x, y, z, q.x, q.y, q.z, q.w]

        except Exception as e:
            self.get_logger().warn(f"No Map: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    tfNode = TfListener()
    time.sleep(1.0)  # 等待 tf 缓冲填充

    while rclpy.ok():
        rclpy.spin_once(tfNode, timeout_sec=0.1)  # 更新 tf buffer
        pose = tfNode.get_pose()
        if pose:
            tfNode.get_logger().info(
                f"位置: x={pose[0]:.2f}, y={pose[1]:.2f}, z={pose[2]:.2f} | "
                f"姿态四元数: ({pose[3]:.2f}, {pose[4]:.2f}, {pose[5]:.2f}, {pose[6]:.2f})"
            )
        time.sleep(0.1)
        
    tfNode.destroy_node()
    rclpy.shutdown()