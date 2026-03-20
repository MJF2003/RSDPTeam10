#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from rover_interface.msg import (
    BlockPoseObservation,
    BinPoseObservation,
    PlatformPoseObservation,
    BinOpeningPoseObservation,
)


class TracksMonitorTyped(Node):
    def __init__(self):
        super().__init__("rsdp_tracks_monitor_typed")

        self.create_subscription(BlockPoseObservation, "/cv/block_poses", self.cb_blocks, 10)
        self.create_subscription(BinPoseObservation, "/cv/bin_poses", self.cb_bins, 10)
        self.create_subscription(PlatformPoseObservation, "/cv/platform_poses", self.cb_platforms, 10)
        self.create_subscription(BinOpeningPoseObservation, "/cv/bin_opening_poses", self.cb_openings, 10)

        self.get_logger().info("Listening to /cv/block_poses /cv/bin_poses /cv/platform_poses /cv/bin_opening_poses ...")

    def cb_blocks(self, msg: BlockPoseObservation):
        lines = [f"BLOCKS frame={msg.header.frame_id} fps={msg.yolo_fps:.2f} n={len(msg.observations)}"]
        for o in msg.observations:
            lines.append(
                f"  id={o.id:>2} conf={o.confidence:.3f} "
                f"xyz=({o.pose.position.x:.4f},{o.pose.position.y:.4f},{o.pose.position.z:.4f}) "
                f"color={o.color_label}({o.color_confidence:.2f},v={o.color_votes}) "
                f"shape={o.shape_label}({o.shape_confidence:.2f},v={o.shape_votes}) "
                f"missed={o.missed}"
            )
        self.get_logger().info("\n" + "\n".join(lines))

    def cb_bins(self, msg: BinPoseObservation):
        lines = [f"BINS frame={msg.header.frame_id} fps={msg.yolo_fps:.2f} n={len(msg.observations)}"]
        for o in msg.observations:
            lines.append(
                f"  id={o.id:>2} conf={o.confidence:.3f} "
                f"xyz=({o.pose.position.x:.4f},{o.pose.position.y:.4f},{o.pose.position.z:.4f}) "
                f"color={o.color_label}({o.color_confidence:.2f},v={o.color_votes}) "
                f"missed={o.missed}"
            )
        self.get_logger().info("\n" + "\n".join(lines))

    def cb_platforms(self, msg: PlatformPoseObservation):
        lines = [f"PLATFORMS frame={msg.header.frame_id} fps={msg.yolo_fps:.2f} n={len(msg.observations)}"]
        for o in msg.observations:
            lines.append(
                f"  id={o.id:>2} conf={o.confidence:.3f} "
                f"xyz=({o.pose.position.x:.4f},{o.pose.position.y:.4f},{o.pose.position.z:.4f}) "
                f"missed={o.missed}"
            )
        self.get_logger().info("\n" + "\n".join(lines))

    def cb_openings(self, msg: BinOpeningPoseObservation):
        lines = [f"BIN_OPENINGS frame={msg.header.frame_id} fps={msg.yolo_fps:.2f} n={len(msg.observations)}"]
        for o in msg.observations:
            lines.append(
                f"  id={o.id:>2} conf={o.confidence:.3f} "
                f"xyz=({o.pose.position.x:.4f},{o.pose.position.y:.4f},{o.pose.position.z:.4f}) "
                f"missed={o.missed}"
            )
        self.get_logger().info("\n" + "\n".join(lines))


def main():
    rclpy.init()
    node = TracksMonitorTyped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()