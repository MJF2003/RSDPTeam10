from functools import partial
from random import gauss

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node

from rover_interface.msg import (
    BinPose,
    BinPoseObservation,
    BlockPose,
    BlockBinColor,
    BlockPoseObservation,
)

# TODO: add publish function for observations
# TODO: add subs for each block type
# TODO: also need to see where the rover is relative to the blocks to compute the true orientation. Question - what frame are BlockPoseObservations made in?


class VisionStub(Node):
    def __init__(self):
        super().__init__("vision_stub")
        self.get_logger().info("Launching vision stub")
        self.observation_frame_id = str(
            self.declare_parameter("observation_frame_id", "map").value
        )
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

    def fill_obs_header(
        self,
        obs: BlockPoseObservation | BinPoseObservation,
        msg: PoseArray,
    ) -> None:
        "Add the shared header information to an observation envelope."
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = self.observation_frame_id
        obs.yolo_fps = 0.0

    def build_pose(self, msg: PoseArray) -> Pose | None:
        "Build a noisy pose from the first Gazebo pose array entry."
        if not msg.poses:
            self.get_logger().warning("Received empty PoseArray from simulator.")
            return None

        source_pose = msg.poses[0]
        pose = Pose()
        pose.position.x = self.apply_noise(source_pose.position.x)
        pose.position.y = self.apply_noise(source_pose.position.y)
        pose.position.z = self.apply_noise(source_pose.position.z)
        pose.orientation.x = source_pose.orientation.x
        pose.orientation.y = source_pose.orientation.y
        pose.orientation.z = source_pose.orientation.z
        pose.orientation.w = source_pose.orientation.w
        return pose

    def apply_noise(self, value: float) -> float:
        if not self.add_gaussian_noise or self.position_noise_stddev_m == 0.0:
            return value
        return value + gauss(0.0, self.position_noise_stddev_m)

    def block_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Converts a pose message from gazebo into a
        rover_interface-compatible message"""
        pose = self.build_pose(msg)
        if pose is None:
            return

        obs = BlockPoseObservation()
        self.fill_obs_header(obs, msg)

        detection = BlockPose()
        detection.id = 0
        detection.pose = pose
        detection.confidence = 0.0
        detection.age = 0
        detection.missed = 0
        detection.center_u = 0
        detection.center_v = 0
        detection.color = color
        detection.color_confidence = 0.0
        detection.color_votes = 0
        detection.shape_label = "cube"
        detection.shape_confidence = 0.0
        detection.shape_votes = 0

        obs.observations.append(detection)  # type: ignore

        self.block_publisher.publish(obs)

    def bin_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Converts a pose message from gazebo into a
        rover_interface-compatible message"""
        pose = self.build_pose(msg)
        if pose is None:
            return

        obs = BinPoseObservation()
        self.fill_obs_header(obs, msg)

        detection = BinPose()
        detection.id = 0
        detection.pose = pose
        detection.confidence = 0.0
        detection.age = 0
        detection.missed = 0
        detection.center_u = 0
        detection.center_v = 0
        detection.color = color
        detection.color_confidence = 0.0
        detection.color_votes = 0

        obs.observations.append(detection)  # type: ignore

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
