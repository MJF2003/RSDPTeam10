#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from leo_navigation.msg import BlockList, DetectedBlock

import numpy as np

class BlockLocalizer(Node):
    def __init__(self):
        super().__init__('block_localizer')
        
        self.get_logger().info("Block Localizer Node Started!")
        
        # TF2 相关初始化 - 增加缓存时间
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        #  D435i相机实际内参
        self.camera_matrix = np.array([
            [909.724, 0.0,     646.908],
            [0.0,     909.881, 371.367],
            [0.0,     0.0,     1.0]
        ])
        
        # 相机坐标系名称
        self.camera_frame_id = "camera_color_optical_frame"
        
        # 订阅检测到的方块
        self.detection_sub = self.create_subscription(
            BlockList,
            '/detected_blocks',
            self.detection_callback,
            10
        )
        
        # 发布转换后的方块位置
        self.block_pub = self.create_publisher(BlockList, '/navigable_blocks', 10)
        
        # 发布可视化标记
        self.marker_pub = self.create_publisher(MarkerArray, '/block_markers', 10)
        
        self.get_logger().info("Waiting for block detection and TF transformation...")
        self.get_logger().info("Using the camera coordinate system: camera_color_optical_frame")

    def pixel_to_camera_frame(self, u, v, depth):
        """将像素坐标和深度转换到相机坐标系"""
        try:
            pixel_point = np.array([u, v, 1.0])
            inv_camera_matrix = np.linalg.inv(self.camera_matrix)
            camera_point = inv_camera_matrix.dot(pixel_point)
            camera_point_scaled = camera_point * depth
            
            self.get_logger().debug(
                f"Conversion: Pixel({u:.1f}, {v:.1f}) Depth{depth:.2f}m -> "
                f"Camera Coordinates({camera_point_scaled[0]:.3f}, {camera_point_scaled[1]:.3f}, {camera_point_scaled[2]:.3f})"
            )
            
            return camera_point_scaled
            
        except Exception as e:
            self.get_logger().error(f"Coordinate transformation error: {e}")
            return np.array([0.0, 0.0, 0.0])

    def detection_callback(self, msg):
        """处理接收到的方块检测结果"""
        self.get_logger().info(f"Receive {len(msg.blocks)} results for each block")
        
        transformed_blocks = []
        marker_array = MarkerArray()
        
        for i, block in enumerate(msg.blocks):
            try:
                # 提取数据
                bbox = block.bbox
                u_center = bbox[0]
                v_center = bbox[1]
                depth = block.depth
                
                self.get_logger().info(
                    f"Processing Blocks {block.block_id}: "
                    f"Pixel position=({u_center:.1f}, {v_center:.1f}), Depth={depth:.2f}m"
                )
                
                # 转换到相机坐标系
                point_camera = self.pixel_to_camera_frame(u_center, v_center, depth)
                
                # 创建PointStamped消息
                point_in_camera_frame = PointStamped()
                point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
                point_in_camera_frame.header.frame_id = self.camera_frame_id
                point_in_camera_frame.point.x = point_camera[0]
                point_in_camera_frame.point.y = point_camera[1] 
                point_in_camera_frame.point.z = point_camera[2]
                
                # 转换到机器人基座坐标系
                try:
                    # 等待TF变换可用
                    self.tf_buffer.can_transform('base_link', self.camera_frame_id, rclpy.time.Time())
                    
                    point_in_base_frame = self.tf_buffer.transform(
                        point_in_camera_frame, 
                        'base_link',
                        timeout=rclpy.duration.Duration(seconds=2.0)
                    )
                    
                    self.get_logger().info(
                        f"Successfully converted {block.block_id} to base_link: "
                        f"({point_in_base_frame.point.x:.2f}, {point_in_base_frame.point.y:.2f}, {point_in_base_frame.point.z:.2f})"
                    )
                    
                    # 保存变换后的方块信息
                    transformed_block = DetectedBlock()
                    transformed_block.block_id = block.block_id
                    transformed_block.bbox = block.bbox
                    transformed_block.depth = block.depth
                    transformed_block.confidence = block.confidence
                    transformed_blocks.append(transformed_block)
                    
                    # 创建RViz可视化标记
                    marker = self.create_block_marker(point_in_base_frame.point, block.block_id, i)
                    marker_array.markers.append(marker)
                    
                except Exception as tf_error:
                    self.get_logger().error(f"TF transformation failed: {tf_error}")
                    self.get_logger().info("Please ensure the static TF publisher is running.")
                    continue
                    
            except Exception as e:
                self.get_logger().error(f"Processing Block {block.block_id} Error: {e}")
                continue
        
        # 发布变换后的方块列表
        if transformed_blocks:
            output_msg = BlockList()
            output_msg.header.stamp = self.get_clock().now().to_msg()
            output_msg.header.frame_id = 'base_link'
            output_msg.blocks = transformed_blocks
            self.block_pub.publish(output_msg)
            self.get_logger().info(f"Send {len(transformed_blocks)} converted block to /navigable_blocks")
        
        # 发布可视化标记
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
            self.get_logger().info(f"Send {len(marker_array.markers)} visual marker to /block_markers")

    def create_block_marker(self, point, block_id, marker_id):
        """创建RViz可视化标记 - 放大版本"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_blocks"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 位置 - 确保在地面以上
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = max(point.z, 0.1) + 0.1  # 确保至少在地面上0.2米
        
        # 明确的四元数
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # 🎯 放大尺寸！更容易看到
        marker.scale.x = 0.15  # 15厘米
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        
        # 🎯 更鲜艳的颜色
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 完全不透明
        
        return marker

def main():
    rclpy.init()
    
    try:
        node = BlockLocalizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nThe node was interrupted by the user.")
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("Node has been closed")

if __name__ == '__main__':
    main()