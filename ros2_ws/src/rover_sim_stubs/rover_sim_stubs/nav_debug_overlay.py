import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from rover_interface.msg import (
    BinPoseSmoothed,
    BinPoseSmoothedArray,
    BlockBinColor,
    BlockPoseSmoothed,
    BlockPoseSmoothedArray,
)


class NavDebugOverlay(Node):
    def __init__(self):
        super().__init__("nav_debug_overlay")

        self.marker_frame = str(self.declare_parameter("marker_frame", "map").value)
        self.publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.1).value
        )
        self.summary_period_sec = float(
            self.declare_parameter("summary_period_sec", 10.0).value
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/debug/nav_markers",
            10,
        )

        self.controller_blocks: list[BlockPoseSmoothed] = []
        self.controller_bins: list[BinPoseSmoothed] = []

        self._warned_frame_sources: set[tuple[str, str]] = set()
        self._last_summary_time = self.get_clock().now()

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

        self.create_timer(self.publish_period_sec, self.publish_markers)
        self.get_logger().info(
            "Publishing navigation debug overlay on /debug/nav_markers"
        )

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

    def publish_markers(self):
        markers = MarkerArray()
        markers.markers.append(self._delete_all_marker())

        markers.markers.extend(self._controller_markers_for_bins())
        markers.markers.extend(self._controller_markers_for_blocks())

        self.marker_pub.publish(markers)
        self._log_summary()

    def _delete_all_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        return marker

    def _controller_markers_for_bins(self) -> list[Marker]:
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
        return markers

    def _controller_markers_for_blocks(self) -> list[Marker]:
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
        return markers

    def _shape_marker(
        self,
        namespace: str,
        marker_id: int,
        marker_type: int,
        point,
        scale_xyz: tuple[float, float, float],
        rgba: tuple[float, float, float, float],
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
        rgba: tuple[float, float, float, float],
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

    def _log_summary(self):
        now = self.get_clock().now()
        elapsed_sec = (now - self._last_summary_time).nanoseconds / 1e9
        if elapsed_sec < self.summary_period_sec:
            return

        self._last_summary_time = now
        self.get_logger().info(
            "controller_bins=%d controller_blocks=%d"
            % (
                len(self.controller_bins),
                len(self.controller_blocks),
            )
        )

    @staticmethod
    def _color_rgba(
        color_value: int, alpha: float
    ) -> tuple[float, float, float, float]:
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
