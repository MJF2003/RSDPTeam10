import math
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from rover_interface.msg import (
    BinPoseObservation,
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
    BlockPoseObservation,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
)


@dataclass
class RawObservation:
    color_value: int
    detection_id: int
    frame_id: str
    point: Point
    timestamp: float


class NavDebugOverlay(Node):
    def __init__(self):
        super().__init__("nav_debug_overlay")

        self.marker_frame = str(self.declare_parameter("marker_frame", "map").value)
        self.publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.1).value
        )
        self.summary_period_sec = float(
            self.declare_parameter("summary_period_sec", 1.0).value
        )
        self.rover_trail_length = int(
            self.declare_parameter("rover_trail_length", 200).value
        )

        self.observation_timeout_sec = int(
            self.declare_parameter("observation_timeout_sec", 5.0).value
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/debug/nav_markers",
            10,
        )

        self.current_pose: Optional[tuple[float, float, float]] = None
        self.controller_blocks: list[BlockPoseSmoothed] = []
        self.controller_bins: list[BinPoseSmoothed] = []
        self.raw_block_obs: Dict[Tuple[int, int, int], RawObservation] = {}
        self.raw_bin_obs: Dict[Tuple[int, int, int], RawObservation] = {}
        self.rover_trail: list[Point] = []

        self._warned_frame_sources: set[Tuple[str, str]] = set()
        self._last_summary_time = self.get_clock().now()
        self._pose_topic_logged = False
        self._empty_pose_frame_warned = False
        self._unexpected_pose_frame_warned = False

        self.create_subscription(
            PoseArray,
            "/sim/rover_pose",
            self.global_pose_callback,
            10,
        )
        self.create_subscription(
            BlockPoseSmoothedArray,
            "/controller/block_poses",
            self.controller_block_callback,
            10,
        )
        self.create_subscription(
            BinPoseSmoothedArray,
            "/controller/bin_poses",
            self.controller_bin_callback,
            10,
        )
        self.create_subscription(
            BlockPoseObservation,
            "/cv/block_poses",
            self.raw_block_callback,
            10,
        )
        self.create_subscription(
            BinPoseObservation,
            "/cv/bin_poses",
            self.raw_bin_callback,
            10,
        )

        self.create_timer(self.publish_period_sec, self.publish_markers)
        self.get_logger().info(
            "Publishing navigation debug overlay on /debug/nav_markers"
        )

    def global_pose_callback(self, msg: PoseArray):
        if not msg.poses:
            return

        pose = msg.poses[0]
        frame_id = msg.header.frame_id.strip()
        yaw = self._yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        x = float(pose.position.x)
        y = float(pose.position.y)
        self.current_pose = (x, y, yaw)
        point = Point(
            x=x,
            y=y,
            z=0.05,
        )
        self.rover_trail.append(point)
        if len(self.rover_trail) > self.rover_trail_length:
            self.rover_trail = self.rover_trail[-self.rover_trail_length :]
        if not self._pose_topic_logged:
            self.get_logger().info("Overlay rover pose is using /sim/rover_pose")
            self._pose_topic_logged = True
        if not frame_id and not self._empty_pose_frame_warned:
            self.get_logger().warning(
                f"/sim/rover_pose arrived with an empty frame_id; "
                f"visualizing it in '{self.marker_frame}'."
            )
            self._empty_pose_frame_warned = True
        elif frame_id and frame_id != self.marker_frame:
            if not self._unexpected_pose_frame_warned:
                self.get_logger().warning(
                    f"/sim/rover_pose arrived in frame '{frame_id}', "
                    f"but overlay is visualizing it in '{self.marker_frame}'."
                )
                self._unexpected_pose_frame_warned = True

    def controller_block_callback(self, msg: BlockPoseSmoothedArray):
        self.controller_blocks = list(msg.blocks)
        for block in self.controller_blocks:
            self._warn_on_frame_mismatch(
                source="controller/block_poses",
                frame_id=block.position.header.frame_id,
            )

    def controller_bin_callback(self, msg: BinPoseSmoothedArray):
        self.controller_bins = list(msg.bins)
        for bin_pose in self.controller_bins:
            self._warn_on_frame_mismatch(
                source="controller/bin_poses",
                frame_id=bin_pose.position.header.frame_id,
            )

    def raw_block_callback(self, msg: BlockPoseObservation):
        self._warn_on_frame_mismatch(
            source="cv/block_poses",
            frame_id=msg.header.frame_id,
        )
        now = self.get_clock().now().nanoseconds / 1e9
        for index, observation in enumerate(msg.observations):
            marker_key = (int(observation.color), int(observation.id), index)
            self.raw_block_obs[marker_key] = RawObservation(
                color_value=int(observation.color),
                detection_id=int(observation.id),
                frame_id=msg.header.frame_id,
                point=self._point(
                    observation.pose.position.x,
                    observation.pose.position.y,
                    observation.pose.position.z,
                ),
                timestamp=now,
            )

    def raw_bin_callback(self, msg: BinPoseObservation):
        self._warn_on_frame_mismatch(
            source="cv/bin_poses",
            frame_id=msg.header.frame_id,
        )
        now = self.get_clock().now().nanoseconds / 1e9
        for index, observation in enumerate(msg.observations):
            marker_key = (int(observation.color), int(observation.id), index)
            self.raw_bin_obs[marker_key] = RawObservation(
                color_value=int(observation.color),
                detection_id=int(observation.id),
                frame_id=msg.header.frame_id,
                point=self._point(
                    observation.pose.position.x,
                    observation.pose.position.y,
                    observation.pose.position.z,
                ),
                timestamp=now,
            )

    def _prune_stale_observations(self):
        now = self.get_clock().now().nanoseconds / 1e9
        cutoff = now - self.observation_timeout_sec
        self.raw_block_obs = {
            k: v for k, v in self.raw_block_obs.items() if v.timestamp >= cutoff
        }
        self.raw_bin_obs = {
            k: v for k, v in self.raw_bin_obs.items() if v.timestamp >= cutoff
        }

    def publish_markers(self):
        self._prune_stale_observations()
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker())

        if self.current_pose is not None:
            markers.markers.append(self._rover_marker())
            trail_marker = self._trail_marker()
            if trail_marker is not None:
                markers.markers.append(trail_marker)

        rover_xy = self._current_rover_xy()

        markers.markers.extend(
            self._controller_markers_for_bins(rover_xy=rover_xy),
        )
        markers.markers.extend(
            self._controller_markers_for_blocks(rover_xy=rover_xy),
        )
        markers.markers.extend(self._raw_markers_for_bins())
        markers.markers.extend(self._raw_markers_for_blocks())

        self.marker_pub.publish(markers)
        self._log_summary(rover_xy)

    def _delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _rover_marker(self) -> Marker:
        assert self.current_pose is not None
        x, y, yaw = self.current_pose
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rover"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.5
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.3
        return marker

    def _trail_marker(self) -> Optional[Marker]:
        if len(self.rover_trail) < 2:
            return None

        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rover_trail"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.85
        marker.color.g = 0.85
        marker.color.b = 0.85
        marker.color.a = 0.8
        marker.points = list(self.rover_trail)
        return marker

    def _controller_markers_for_bins(
        self, rover_xy: Optional[Tuple[float, float]]
    ) -> list[Marker]:
        markers: list[Marker] = []
        for index, bin_pose in enumerate(self.controller_bins):
            point = bin_pose.position.point
            rgba = self._color_rgba(int(bin_pose.color.color), alpha=0.9)
            markers.append(
                self._shape_marker(
                    namespace="controller_bin",
                    marker_id=1000 + index,
                    marker_type=Marker.CYLINDER,
                    point=point,
                    scale_xyz=(0.35, 0.35, 0.25),
                    rgba=rgba,
                )
            )
            markers.append(
                self._text_marker(
                    namespace="controller_bin_text",
                    marker_id=1100 + index,
                    point=point,
                    rgba=rgba,
                    text=(
                        f"ctrl bin {self._color_name(int(bin_pose.color.color))}\n"
                        f"frame='{bin_pose.position.header.frame_id}'\n"
                        f"({point.x:.2f}, {point.y:.2f})"
                    ),
                )
            )
            if rover_xy is not None:
                markers.append(
                    self._line_marker(
                        namespace="controller_bin_line",
                        marker_id=1200 + index,
                        start=self._point(rover_xy[0], rover_xy[1], 0.15),
                        end=self._point(point.x, point.y, 0.15),
                        rgba=rgba,
                    )
                )
        return markers

    def _controller_markers_for_blocks(
        self, rover_xy: Optional[Tuple[float, float]]
    ) -> list[Marker]:
        markers: list[Marker] = []
        for index, block in enumerate(self.controller_blocks):
            point = block.position.point
            rgba = self._color_rgba(int(block.color.color), alpha=0.9)
            markers.append(
                self._shape_marker(
                    namespace="controller_block",
                    marker_id=2000 + index,
                    marker_type=Marker.CUBE,
                    point=point,
                    scale_xyz=(0.18, 0.18, 0.18),
                    rgba=rgba,
                )
            )
            markers.append(
                self._text_marker(
                    namespace="controller_block_text",
                    marker_id=2100 + index,
                    point=point,
                    rgba=rgba,
                    text=(
                        f"ctrl block id={block.id}\n"
                        f"{self._color_name(int(block.color.color))}\n"
                        f"frame='{block.position.header.frame_id}'\n"
                        f"({point.x:.2f}, {point.y:.2f})"
                    ),
                )
            )
            if rover_xy is not None:
                markers.append(
                    self._line_marker(
                        namespace="controller_block_line",
                        marker_id=2200 + index,
                        start=self._point(rover_xy[0], rover_xy[1], 0.10),
                        end=self._point(point.x, point.y, 0.10),
                        rgba=rgba,
                    )
                )
        return markers

    def _raw_markers_for_bins(self) -> list[Marker]:
        markers: list[Marker] = []
        for index, (_, observation) in enumerate(sorted(self.raw_bin_obs.items())):
            point = observation.point
            rgba = self._color_rgba(observation.color_value, alpha=0.35)
            markers.append(
                self._shape_marker(
                    namespace="raw_bin",
                    marker_id=3000 + index,
                    marker_type=Marker.SPHERE,
                    point=point,
                    scale_xyz=(0.22, 0.22, 0.22),
                    rgba=rgba,
                )
            )
            markers.append(
                self._text_marker(
                    namespace="raw_bin_text",
                    marker_id=3100 + index,
                    point=point,
                    rgba=rgba,
                    text=(
                        f"cv bin {self._color_name(observation.color_value)}\n"
                        f"id={observation.detection_id}\n"
                        f"src frame='{observation.frame_id}'\n"
                        f"shown as {self.marker_frame}\n"
                        f"({point.x:.2f}, {point.y:.2f})"
                    ),
                )
            )
        return markers

    def _raw_markers_for_blocks(self) -> list[Marker]:
        markers: list[Marker] = []
        for index, (_, observation) in enumerate(sorted(self.raw_block_obs.items())):
            point = observation.point
            rgba = self._color_rgba(observation.color_value, alpha=0.35)
            markers.append(
                self._shape_marker(
                    namespace="raw_block",
                    marker_id=4000 + index,
                    marker_type=Marker.SPHERE,
                    point=point,
                    scale_xyz=(0.16, 0.16, 0.16),
                    rgba=rgba,
                )
            )
            markers.append(
                self._text_marker(
                    namespace="raw_block_text",
                    marker_id=4100 + index,
                    point=point,
                    rgba=rgba,
                    text=(
                        f"cv block {self._color_name(observation.color_value)}\n"
                        f"id={observation.detection_id}\n"
                        f"src frame='{observation.frame_id}'\n"
                        f"shown as {self.marker_frame}\n"
                        f"({point.x:.2f}, {point.y:.2f})"
                    ),
                )
            )
        return markers

    def _shape_marker(
        self,
        namespace: str,
        marker_id: int,
        marker_type: int,
        point,
        scale_xyz: Tuple[float, float, float],
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(point.x)
        marker.pose.position.y = float(point.y)
        marker.pose.position.z = float(point.z) + (scale_xyz[2] / 2.0)
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale_xyz[0]
        marker.scale.y = scale_xyz[1]
        marker.scale.z = scale_xyz[2]
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        return marker

    def _text_marker(
        self,
        namespace: str,
        marker_id: int,
        point,
        rgba: Tuple[float, float, float, float],
        text: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(point.x)
        marker.pose.position.y = float(point.y)
        marker.pose.position.z = float(point.z) + 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.17
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = min(1.0, rgba[3] + 0.3)
        marker.text = text
        return marker

    def _line_marker(
        self,
        namespace: str,
        marker_id: int,
        start: Point,
        end: Point,
        rgba: Tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = rgba[0]
        marker.color.g = rgba[1]
        marker.color.b = rgba[2]
        marker.color.a = rgba[3]
        marker.points = [start, end]
        return marker

    def _current_rover_xy(self) -> Optional[Tuple[float, float]]:
        if self.current_pose is None:
            return None
        return (self.current_pose[0], self.current_pose[1])

    def _warn_on_frame_mismatch(self, source: str, frame_id: str):
        frame_key = frame_id.strip() if frame_id else ""
        warned_key = (source, frame_key)
        if warned_key in self._warned_frame_sources:
            return

        if not frame_key:
            self.get_logger().warning(
                f"{source} arrived with an empty frame_id; "
                f"visualizing coordinates as '{self.marker_frame}'."
            )
        elif frame_key != self.marker_frame:
            self.get_logger().warning(
                f"{source} arrived in frame '{frame_key}', "
                f"but overlay is visualizing it in '{self.marker_frame}'."
            )
        self._warned_frame_sources.add(warned_key)

    def _log_summary(self, rover_xy: Optional[Tuple[float, float]]):
        now = self.get_clock().now()
        elapsed_sec = (now - self._last_summary_time).nanoseconds / 1e9
        if elapsed_sec < self.summary_period_sec:
            return

        self._last_summary_time = now
        if rover_xy is None:
            self.get_logger().info(
                "No global pose yet. controller_bins=%d controller_blocks=%d raw_bins=%d raw_blocks=%d"
                % (
                    len(self.controller_bins),
                    len(self.controller_blocks),
                    len(self.raw_bin_obs),
                    len(self.raw_block_obs),
                )
            )
            return

        yaw = self._current_yaw()
        summary_parts = [f"map=({rover_xy[0]:.2f}, {rover_xy[1]:.2f}, yaw={yaw:.2f})"]
        summary_parts.extend(
            self._distance_summaries(
                rover_xy=rover_xy,
                prefix="bin",
                points=(
                    (
                        self._color_name(int(bin_pose.color.color)),
                        bin_pose.position.point.x,
                        bin_pose.position.point.y,
                    )
                    for bin_pose in self.controller_bins
                ),
            )
        )
        summary_parts.extend(
            self._distance_summaries(
                rover_xy=rover_xy,
                prefix="block",
                points=(
                    (
                        f"id={block.id}/{self._color_name(int(block.color.color))}",
                        block.position.point.x,
                        block.position.point.y,
                    )
                    for block in self.controller_blocks
                ),
            )
        )
        self.get_logger().info(" | ".join(summary_parts))

    def _distance_summaries(
        self,
        rover_xy: Tuple[float, float],
        prefix: str,
        points: Iterable[Tuple[str, float, float]],
    ) -> list[str]:
        summaries: list[str] = []
        for label, x, y in points:
            distance = math.hypot(x - rover_xy[0], y - rover_xy[1])
            summaries.append(f"{prefix}[{label}]_d={distance:.2f}")
        return summaries

    def _current_yaw(self) -> float:
        assert self.current_pose is not None
        return self.current_pose[2]

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        return Point(x=float(x), y=float(y), z=float(z))

    @staticmethod
    def _color_rgba(
        color_value: int, alpha: float
    ) -> Tuple[float, float, float, float]:
        color_map = {
            BlockBinColor.BLUE: (0.12, 0.47, 0.96),
            BlockBinColor.RED: (0.92, 0.20, 0.18),
            BlockBinColor.YELLOW: (0.95, 0.82, 0.16),
            BlockBinColor.PURPLE: (0.58, 0.30, 0.82),
            BlockBinColor.PINK: (0.96, 0.48, 0.71),
            BlockBinColor.GREEN: (0.20, 0.72, 0.27),
        }
        r, g, b = color_map.get(color_value, (0.70, 0.70, 0.70))
        return (r, g, b, alpha)

    @staticmethod
    def _color_name(color_value: int) -> str:
        return {
            BlockBinColor.BLUE: "BLUE",
            BlockBinColor.RED: "RED",
            BlockBinColor.YELLOW: "YELLOW",
            BlockBinColor.PURPLE: "PURPLE",
            BlockBinColor.PINK: "PINK",
            BlockBinColor.GREEN: "GREEN",
        }.get(color_value, f"UNKNOWN({color_value})")


def main():
    try:
        rclpy.init()
        rclpy.spin(NavDebugOverlay())
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(exc)


if __name__ == "__main__":
    main()
