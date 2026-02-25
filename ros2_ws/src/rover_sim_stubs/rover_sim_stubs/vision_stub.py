from functools import partial
from random import gauss

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray
from rclpy.node import Node

from rover_interface.msg import (
    BinPoseObservation,
    BlockBinColor,
    BlockPoseObservation,
    BlockShape,
)

# TODO: add publish function for observations
# TODO: add subs for each block type
# TODO: also need to see where the rover is relative to the blocks to compute the true orientation. Question - what frame are BlockPoseObservations made in?


class VisionStub(Node):
    def __init__(self):
        super().__init__("vision_stub")
        self.get_logger().info("Launching vision stub")
        self.add_gaussian_noise = bool(
            self.declare_parameter("add_gaussian_noise", True).value
        )
        self.position_noise_stddev_m = float(
            self.declare_parameter("position_noise_stddev_m", 0.02).value
        )
        if self.position_noise_stddev_m < 0.0:
            self.get_logger().warning(
                "position_noise_stddev_m was negative; using absolute value."
            )
            self.position_noise_stddev_m = abs(self.position_noise_stddev_m)

        blockbin_colors = {
            "red": BlockBinColor.RED,
            "blue": BlockBinColor.BLUE,
            "yellow": BlockBinColor.YELLOW,
        }

        self.subs = []
        # Build all the pubs and subs
        for block_str, color in blockbin_colors.items():
            self.get_logger().info(
                f"Creating block subscription to /model/block_{block_str}_1/pose"
            )
            self.subs.append(
                self.create_subscription(
                    msg_type=PoseArray,
                    topic=f"/model/block_{block_str}_1/pose",
                    callback=partial(self.block_obs_callback, color=color),
                    qos_profile=1,
                )
            )
        self.block_publisher = self.create_publisher(
            BlockPoseObservation,
            topic="cv/block_poses",
            qos_profile=1,
        )

        for bin_str, color in blockbin_colors.items():
            self.get_logger().info(
                f"Creating bin subscription to /model/bin_{bin_str}_1/pose"
            )
            self.subs.append(
                self.create_subscription(
                    msg_type=PoseArray,
                    topic=f"/model/bin_{bin_str}_1/pose",
                    callback=partial(self.bin_obs_callback, color=color),
                    qos_profile=1,
                )
            )
        self.bin_publisher = self.create_publisher(
            BinPoseObservation,
            topic="cv/bin_poses",
            qos_profile=1,
        )

    def fill_obs_features[T: BlockPoseObservation | BinPoseObservation](
        self,
        obs: T,
        msg: PoseArray,
    ) -> T:
        "Add the header and pose information to an observation message"
        obs.position.header.stamp = msg.header.stamp
        obs.position.header.frame_id = msg.header.frame_id

        # All the objects have their own topics, so
        # just index into the first element in the array
        obs.position.point.x = self.apply_noise(msg.poses[0].position.x)  # type: ignore
        obs.position.point.y = self.apply_noise(msg.poses[0].position.y)  # type: ignore
        obs.position.point.z = self.apply_noise(msg.poses[0].position.z)  # type: ignore

        return obs

    def apply_noise(self, value: float) -> float:
        if not self.add_gaussian_noise or self.position_noise_stddev_m == 0.0:
            return value
        return value + gauss(0.0, self.position_noise_stddev_m)

    def block_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Converts a pose message from gazebo into a
        rover_interface-compatible message"""
        obs = BlockPoseObservation()
        obs = self.fill_obs_features(obs, msg)
        obs.shape.shape = BlockShape.CUBE
        obs.color.color = color

        self.block_publisher.publish(obs)

    def bin_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Converts a pose message from gazebo into a
        rover_interface-compatible message"""
        obs = BinPoseObservation()
        obs = self.fill_obs_features(obs, msg)
        obs.color.color = color

        self.bin_publisher.publish(obs)


def main():
    try:
        rclpy.init()
        node = VisionStub()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
# Pose topics
# /model/bin_blue_1/pose
# /model/bin_red_1/pose
# /model/bin_yellow_1/pose
# /model/block_blue_1/pose
# /model/block_red_1/pose
# /model/block_yellow_1/pose
# /model/platform_blue/pose
# /model/platform_red/pose
# /model/platform_yellow/pose
