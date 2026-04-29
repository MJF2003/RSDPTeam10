import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from rover_interface.msg import (
    BinPose,
    BinPoseObservation,
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
    BlockPose,
    BlockPoseObservation,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
    BlockShape,
)

UNKNOWN_COLOR = 255


class AggregateObservations(Node):
    def __init__(self):
        super().__init__("aggregate_observations")
        self.get_logger().info("Launching observation smoothing node")

        self.target_frame = "map"
        self.tf_lookup_timeout = Duration(seconds=0.2)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._transform_warn_period_ns = int(5e9)
        self._last_transform_warn_ns: dict[str, int] = {}

        self.block_dist_threshold_m = self.declare_parameter(
            "block_dist_threshold_m", 0.5
        ).value
        self.get_logger().info(
            f"Setting block disambiguation distance threshold to "
            f"{self.block_dist_threshold_m:.2f}m"
        )
        self.bin_dist_threshold_m = self.declare_parameter(
            "bin_dist_threshold_m", 0.2
        ).value
        self.get_logger().info(
            f"Setting bin disambiguation distance threshold to "
            f"{self.bin_dist_threshold_m:.2f}m"
        )

        self.debug = self.declare_parameter("debug", False).value

        # -- BLOCK SETUP ------------------------------------------------------
        self.block_sub = self.create_subscription(
            BlockPoseObservation,
            topic="/cv/block_poses",
            callback=self.observe_block,
            qos_profile=1,
        )

        self.block_poses: np.ndarray | None = None
        self.block_colors: list[int] = []
        self.block_frame_id = self.target_frame

        self.block_pub_topic = "/controller/block_poses"
        self.block_pub = self.create_publisher(
            msg_type=BlockPoseSmoothedArray,
            topic=self.block_pub_topic,
            qos_profile=1,
        )

        self.create_timer(0.1, self.publish_smoothed_block_array)

        # -- BIN SETUP --------------------------------------------------------
        self.bin_sub = self.create_subscription(
            msg_type=BinPoseObservation,
            topic="/cv/bin_poses",
            callback=self.observe_bin,
            qos_profile=1,
        )
        self.bin_poses: np.ndarray | None = None
        self.bin_colors: list[int] = []
        self.bin_frame_id = self.target_frame

        self.bin_pub_topic = "/controller/bin_poses"
        self.bin_pub = self.create_publisher(
            msg_type=BinPoseSmoothedArray,
            topic=self.bin_pub_topic,
            qos_profile=1,
        )

        self.create_timer(0.1, self.publish_smoothed_bin_array)

    def _maybe_warn_transform_failure(self, key: str, message: str):
        now_ns = self.get_clock().now().nanoseconds
        last_warn_ns = self._last_transform_warn_ns.get(key, 0)
        if now_ns - last_warn_ns < self._transform_warn_period_ns:
            return

        self._last_transform_warn_ns[key] = now_ns
        self.get_logger().warning(message)

    def _lookup_observation_transform(self, source_frame: str, stamp, topic: str):
        source_frame = source_frame.strip()
        if not source_frame:
            self._maybe_warn_transform_failure(
                f"{topic}:empty_frame",
                f"Dropping observations from {topic}: empty source frame_id.",
            )
            return False, None

        if source_frame == self.target_frame:
            return True, None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                timeout=self.tf_lookup_timeout,
            )
        except TransformException as exc:
            self._maybe_warn_transform_failure(
                f"{topic}:{source_frame}",
                f"Dropping observations from {topic}: failed to transform "
                f"'{source_frame}' -> '{self.target_frame}': {exc}",
            )
            return False, None

        return True, transform

    def _position_from_pose(
        self, pose, source_frame: str, stamp, transform
    ) -> np.ndarray:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = source_frame
        pose_stamped.header.stamp = stamp
        pose_stamped.pose = pose

        if transform is not None:
            pose_stamped = do_transform_pose_stamped(pose_stamped, transform)

        return np.array(
            [
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z,
            ]
        )

    # --------------------- BLOCKS -------------------------------------------------

    def compare_blocks(self, pos: np.ndarray) -> tuple[int, bool]:
        dists = np.linalg.norm(pos - self.block_poses, axis=-1)
        if dists.min() < self.block_dist_threshold_m:
            return dists.argmin(), False
        return len(dists), True

    def publish_smoothed_block_array(self):
        if self.block_poses is None:
            # self.get_logger().info("No known block positions to publish")
            return

        msg = BlockPoseSmoothedArray()
        for i in range(len(self.block_poses)):
            block = BlockPoseSmoothed()
            block.id = i

            pose = self.block_poses[i]
            block.position = PointStamped()
            block.position.header.frame_id = self.block_frame_id
            block.position.header.stamp = self.get_clock().now().to_msg()
            block.position.point = Point(x=pose[0], y=pose[1], z=pose[2])

            block.shape = BlockShape()
            block.shape.shape = BlockShape.CUBE

            block.color = BlockBinColor()
            block.color.color = self.block_colors[i]

            msg.blocks.append(block)  # type:ignore

        self.block_pub.publish(msg=msg)

    def update_pose_estimate(self, current: np.ndarray, new: np.ndarray):
        # TODO: test different smoothing approaches
        return 0.95 * current + 0.05 * new

    def observe_block(self, msg: BlockPoseObservation):
        source_frame = msg.header.frame_id.strip()
        ok, transform = self._lookup_observation_transform(
            source_frame=source_frame,
            stamp=msg.header.stamp,
            topic="/cv/block_poses",
        )
        if not ok:
            return

        self.block_frame_id = self.target_frame
        for observation in msg.observations:
            if int(observation.color) == UNKNOWN_COLOR:
                continue  # ignore blocks of unknown colour
            pos = self._position_from_pose(
                observation.pose, source_frame, msg.header.stamp, transform
            )
            self.observe_block_detection(observation, pos)

    def observe_block_detection(self, observation: BlockPose, pos: np.ndarray):
        if self.block_poses is None:
            self.block_poses = pos[np.newaxis, :]  # Shape (1, 3)
            self.block_colors.append(int(observation.color))
            return

        idx, is_distinct = self.compare_blocks(pos)

        if is_distinct:
            self.block_poses = np.vstack([self.block_poses, pos])
            self.block_colors.append(int(observation.color))
        else:
            self.block_poses[idx] = self.update_pose_estimate(
                self.block_poses[idx], pos
            )

    # --------------------- BINS -------------------------------------------------
    def compare_bins(self, pos: np.ndarray) -> tuple[int, bool]:
        dists = np.linalg.norm(pos - self.bin_poses, axis=-1)
        if dists.min() < self.bin_dist_threshold_m:
            return dists.argmin(), False
        return len(dists), True

    def publish_smoothed_bin_array(self):
        if self.bin_poses is None:
            # self.get_logger().info("No known bin positions to publish")
            return

        msg = BinPoseSmoothedArray()
        for i in range(len(self.bin_poses)):
            bin_pose = BinPoseSmoothed()

            pose = self.bin_poses[i]
            bin_pose.position = PointStamped()
            bin_pose.position.header.frame_id = self.bin_frame_id
            bin_pose.position.header.stamp = self.get_clock().now().to_msg()
            bin_pose.position.point = Point(x=pose[0], y=pose[1], z=pose[2])

            bin_pose.color = BlockBinColor()
            bin_pose.color.color = self.bin_colors[i]

            msg.bins.append(bin_pose)  # type:ignore

        self.bin_pub.publish(msg=msg)

    def observe_bin(self, msg: BinPoseObservation):
        source_frame = msg.header.frame_id.strip()
        ok, transform = self._lookup_observation_transform(
            source_frame=source_frame,
            stamp=msg.header.stamp,
            topic="/cv/bin_poses",
        )
        if not ok:
            return

        self.bin_frame_id = self.target_frame
        for observation in msg.observations:
            if int(observation.color) == UNKNOWN_COLOR:
                continue  # ignore bins of unknown colour
            pos = self._position_from_pose(
                observation.pose, source_frame, msg.header.stamp, transform
            )
            self.observe_bin_detection(observation, pos)

    def observe_bin_detection(self, observation: BinPose, pos: np.ndarray):
        if self.bin_poses is None:
            self.bin_poses = pos[np.newaxis, :]  # Shape (1, 3)
            self.bin_colors.append(int(observation.color))
            return

        idx, is_distinct = self.compare_bins(pos)

        if is_distinct:
            self.bin_poses = np.vstack([self.bin_poses, pos])
            self.bin_colors.append(int(observation.color))
        else:
            self.bin_poses[idx] = self.update_pose_estimate(self.bin_poses[idx], pos)


def main():
    try:
        rclpy.init()
        rclpy.spin(AggregateObservations())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
