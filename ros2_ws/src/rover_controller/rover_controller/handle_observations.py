import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node

from rover_interface.msg import (
    BlockBinColor,
    BlockPoseObservation,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
    BlockShape,
)


class AggregateObservations(Node):
    def __init__(self):
        super().__init__("aggregate_observations")
        self.get_logger().info("Launching observation smoothing node")

        self.block_dist_threshold_m = self.declare_parameter(
            "block_dist_threshold_m", 0.5
        ).value
        self.get_logger().info(
            f"Setting disambiguation distance threshold to {self.block_dist_threshold_m:.2f}m"
        )

        # TODO: set debug to false by default
        self.debug = self.declare_parameter("debug", True).value

        self.sub = self.create_subscription(
            BlockPoseObservation,
            topic="/cv/block_poses",
            callback=self.observe_block,
            qos_profile=1,
        )

        self.block_poses: np.ndarray | None = None
        self.block_colors: list[BlockBinColor] = []

        self.pub_topic = "/controller/block_poses"
        self.pub = self.create_publisher(
            msg_type=BlockPoseSmoothedArray,
            topic=self.pub_topic,
            qos_profile=1,
        )

        self.create_timer(0.1, self.publish_smoothed_array)

    def compare_blocks(self, pos: np.ndarray) -> tuple[int, bool]:
        dists = np.linalg.norm(pos - self.block_poses, axis=-1)
        if dists.min() < self.block_dist_threshold_m:
            # Fix: Return argmin (closest), not argmax (furthest)
            return dists.argmin(), False
        return len(dists), True

    def publish_smoothed_array(self):
        if self.block_poses is None:
            self.get_logger().info("No known block positions to publish")
            return

        msg = BlockPoseSmoothedArray()
        for i in range(len(self.block_poses)):
            block = BlockPoseSmoothed()
            block.id = i

            pose = self.block_poses[i]
            block.position = PointStamped()
            block.position.header.frame_id = "odom"
            block.position.header.stamp = self.get_clock().now().to_msg()
            block.position.point = Point(x=pose[0], y=pose[1], z=pose[2])

            block.shape = BlockShape()
            block.shape.shape = BlockShape.CUBE

            block.color = BlockBinColor()
            # Note: You likely want to map the actual stored colors here
            block.color.color = BlockBinColor.RED
            block.collected = False

            msg.blocks.append(block)  # type:ignore

        self.pub.publish(msg=msg)

    def update_pose_estimate(self, current: np.ndarray, new: np.ndarray):
        # TODO: test different smoothing approaches
        return 0.95 * current + 0.05 * new

    def observe_block(self, msg: BlockPoseObservation):
        pos = np.array(
            [msg.position.point.x, msg.position.point.y, msg.position.point.z]
        )

        if self.block_poses is None:
            self.block_poses = pos[np.newaxis, :]  # Shape (1, 3)
            self.block_colors.append(msg.color)
            return

        idx, is_distinct = self.compare_blocks(pos)

        if is_distinct:
            self.block_poses = np.vstack([self.block_poses, pos])
            self.block_colors.append(msg.color)
        else:
            self.block_poses[idx] = self.update_pose_estimate(
                self.block_poses[idx], pos
            )


def main():
    try:
        rclpy.init()
        rclpy.spin(AggregateObservations())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
