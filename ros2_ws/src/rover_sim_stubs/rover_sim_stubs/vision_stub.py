"""Simulation vision stub with camera-FOV and wall-occlusion filtering."""

from dataclasses import dataclass
from functools import partial
import math
from random import gauss

from geometry_msgs.msg import Pose, PoseArray

import numpy as np

import rclpy
from rclpy.node import Node

from rover_interface.msg import (
    BinPose,
    BinPoseObservation,
    BlockBinColor,
    BlockPose,
    BlockPoseObservation,
)

from sensor_msgs.msg import CameraInfo

CAMERA_TRANSLATION_BASE_M = np.array([0.1746, 0.0, 0.0906], dtype=float)
CAMERA_RPY_BASE_RAD = (0.0, 0.4712, 0.0)
OPTICAL_RPY_CAMERA_RAD = (-1.5708, 0.0, -1.5708)
DEFAULT_IMAGE_WIDTH = 1280
DEFAULT_IMAGE_HEIGHT = 720
DEFAULT_HORIZONTAL_FOV_RAD = 1.204
DEFAULT_NEAR_CLIP_M = 0.28
DEFAULT_FAR_CLIP_M = 10.0


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera model extracted from a CameraInfo message."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class Projection:
    """Projected target centre and optical-frame depth."""

    u: int
    v: int
    depth_m: float


@dataclass(frozen=True)
class BoxOccluder:
    """Axis-aligned box that can block line of sight."""

    name: str
    minimum: np.ndarray
    maximum: np.ndarray


def rotation_matrix_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    """Return ROS fixed-axis roll/pitch/yaw rotation matrix."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array(
        [
            [cy * cp, (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr)],
            [sy * cp, (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr)],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


BASE_TO_CAMERA_OPTICAL_ROTATION = (
    rotation_matrix_from_rpy(*CAMERA_RPY_BASE_RAD)
    @ rotation_matrix_from_rpy(*OPTICAL_RPY_CAMERA_RAD)
)


def box_from_center_size(
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> BoxOccluder:
    """Build an axis-aligned box occluder from SDF centre and size."""
    center_array = np.array(center, dtype=float)
    half_size = np.array(size, dtype=float) / 2.0
    return BoxOccluder(
        name=name,
        minimum=center_array - half_size,
        maximum=center_array + half_size,
    )


DEFAULT_OCCLUDERS = (
    box_from_center_size('wall_north', (0.0, 5.0, 0.25), (10.0, 0.2, 0.5)),
    box_from_center_size('wall_south', (0.0, -5.0, 0.25), (10.0, 0.2, 0.5)),
    box_from_center_size('wall_east', (5.0, 0.0, 0.25), (0.2, 10.0, 0.5)),
    box_from_center_size('wall_west', (-5.0, 0.0, 0.25), (0.2, 10.0, 0.5)),
    box_from_center_size(
        'wall_block_screen',
        (1.35, 0.0, 0.4),
        (0.2, 3.8, 0.8),
    ),
)


def rotation_matrix_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    """Return a normalized quaternion rotation matrix."""
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm == 0.0 or not math.isfinite(norm):
        return np.eye(3, dtype=float)

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - (2.0 * (yy + zz)), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - (2.0 * (xx + zz)), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - (2.0 * (xx + yy))],
        ],
        dtype=float,
    )


def pose_position(pose: Pose) -> np.ndarray:
    """Return a pose position as a 3D array."""
    return np.array(
        [pose.position.x, pose.position.y, pose.position.z],
        dtype=float,
    )


def pose_rotation(pose: Pose) -> np.ndarray:
    """Return a pose orientation as a rotation matrix."""
    q = pose.orientation
    return rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)


def target_centre_from_pose(pose: Pose, z_offset_m: float) -> np.ndarray:
    """Return the object centre point in the pose's parent frame."""
    offset = np.array([0.0, 0.0, z_offset_m], dtype=float)
    return pose_position(pose) + (pose_rotation(pose) @ offset)


def camera_intrinsics_from_info(msg: CameraInfo) -> CameraIntrinsics | None:
    """Build camera intrinsics from a CameraInfo message."""
    width = int(msg.width)
    height = int(msg.height)
    fx = float(msg.k[0])
    fy = float(msg.k[4])
    cx = float(msg.k[2])
    cy = float(msg.k[5])

    values = (fx, fy, cx, cy)
    if width <= 0 or height <= 0:
        return None
    if fx <= 0.0 or fy <= 0.0:
        return None
    if not all(math.isfinite(value) for value in values):
        return None

    return CameraIntrinsics(
        width=width,
        height=height,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
    )


def camera_intrinsics_from_fov(
    *,
    width: int,
    height: int,
    horizontal_fov_rad: float,
) -> CameraIntrinsics:
    """Build camera intrinsics from an image size and horizontal FOV."""
    fx = float(width) / (2.0 * math.tan(horizontal_fov_rad / 2.0))
    return CameraIntrinsics(
        width=width,
        height=height,
        fx=fx,
        fy=fx,
        cx=float(width) / 2.0,
        cy=float(height) / 2.0,
    )


def camera_pose_from_rover(rover_pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    """Return camera optical-frame origin and world rotation."""
    rover_position = pose_position(rover_pose)
    world_from_base = pose_rotation(rover_pose)
    camera_origin_world = (
        rover_position + (world_from_base @ CAMERA_TRANSLATION_BASE_M)
    )
    world_from_optical = world_from_base @ BASE_TO_CAMERA_OPTICAL_ROTATION
    return camera_origin_world, world_from_optical


def project_world_point_to_camera(
    *,
    point_world: np.ndarray,
    rover_pose: Pose,
    intrinsics: CameraIntrinsics,
    near_clip_m: float,
    far_clip_m: float,
) -> Projection | None:
    """Project a world point into the rover depth camera image."""
    camera_origin_world, world_from_optical = camera_pose_from_rover(rover_pose)
    point_optical = world_from_optical.T @ (point_world - camera_origin_world)

    depth_m = float(point_optical[2])
    if not math.isfinite(depth_m):
        return None
    if depth_m < near_clip_m or depth_m > far_clip_m:
        return None

    u_float = (
        (intrinsics.fx * float(point_optical[0]) / depth_m)
        + intrinsics.cx
    )
    v_float = (
        (intrinsics.fy * float(point_optical[1]) / depth_m)
        + intrinsics.cy
    )
    if not math.isfinite(u_float) or not math.isfinite(v_float):
        return None

    u = int(round(u_float))
    v = int(round(v_float))
    if u < 0 or v < 0 or u >= intrinsics.width or v >= intrinsics.height:
        return None

    return Projection(u=u, v=v, depth_m=depth_m)


def project_world_point_to_camera_planar(
    *,
    point_world: np.ndarray,
    rover_pose: Pose,
    intrinsics: CameraIntrinsics,
    near_clip_m: float,
    far_clip_m: float,
) -> Projection | None:
    """Project a world point using only planar forward/lateral offsets."""
    camera_origin_world, _ = camera_pose_from_rover(rover_pose)
    world_from_base = pose_rotation(rover_pose)
    point_base = world_from_base.T @ (point_world - camera_origin_world)

    forward_m = float(point_base[0])
    lateral_m = float(-point_base[1])
    planar_range_m = math.hypot(forward_m, lateral_m)

    if not math.isfinite(forward_m) or not math.isfinite(lateral_m):
        return None
    if forward_m <= 0.0:
        return None
    if planar_range_m < near_clip_m or planar_range_m > far_clip_m:
        return None

    u_float = (intrinsics.fx * lateral_m / forward_m) + intrinsics.cx
    if not math.isfinite(u_float):
        return None

    u = int(round(u_float))
    if u < 0 or u >= intrinsics.width:
        return None

    return Projection(
        u=u,
        v=int(round(intrinsics.cy)),
        depth_m=planar_range_m,
    )


def segment_intersection_t_for_aabb(
    *,
    start: np.ndarray,
    end: np.ndarray,
    box: BoxOccluder,
) -> float | None:
    """Return the first segment fraction intersecting an AABB."""
    direction = end - start
    t_min = 0.0
    t_max = 1.0

    for axis in range(3):
        if abs(direction[axis]) < 1e-12:
            if start[axis] < box.minimum[axis] or start[axis] > box.maximum[axis]:
                return None
            continue

        inv_direction = 1.0 / direction[axis]
        t1 = (box.minimum[axis] - start[axis]) * inv_direction
        t2 = (box.maximum[axis] - start[axis]) * inv_direction
        if t1 > t2:
            t1, t2 = t2, t1

        t_min = max(t_min, float(t1))
        t_max = min(t_max, float(t2))
        if t_min > t_max:
            return None

    return t_min


def is_occluded_by_boxes(
    *,
    camera_origin_world: np.ndarray,
    target_world: np.ndarray,
    boxes: tuple[BoxOccluder, ...],
    target_tolerance: float = 1e-3,
) -> bool:
    """Return true if a wall box intersects the sightline before the target."""
    for box in boxes:
        t_hit = segment_intersection_t_for_aabb(
            start=camera_origin_world,
            end=target_world,
            box=box,
        )
        if t_hit is not None and 0.0 <= t_hit < 1.0 - target_tolerance:
            return True

    return False


class VisionStub(Node):
    """Publish visible fake observations."""

    def __init__(self):
        """Initialize subscriptions and visibility state."""
        super().__init__('vision_stub')
        self.get_logger().info('Launching vision stub')
        self.observation_frame_id = str(
            self.declare_parameter('observation_frame_id', 'map').value
        )
        self.add_gaussian_noise = bool(
            self.declare_parameter('add_gaussian_noise', True).value
        )
        self.position_noise_stddev_m = float(
            self.declare_parameter('position_noise_stddev_m', 0.02).value
        )
        if self.position_noise_stddev_m < 0.0:
            self.get_logger().warning(
                'position_noise_stddev_m was negative; using absolute value.'
            )
            self.position_noise_stddev_m = abs(self.position_noise_stddev_m)

        self.rover_pose_topic = str(
            self.declare_parameter('rover_pose_topic', '/sim/rover_pose').value
        )
        self.camera_info_topic = str(
            self.declare_parameter(
                'camera_info_topic',
                '/depth_camera/camera_info',
            ).value
        )
        self.camera_data_timeout_sec = max(
            0.0,
            float(
                self.declare_parameter('camera_data_timeout_sec', 0.5).value
            ),
        )
        self.enable_wall_occlusion = bool(
            self.declare_parameter('enable_wall_occlusion', True).value
        )
        self.horizontal_fov_rad = float(
            self.declare_parameter(
                'horizontal_fov_rad',
                DEFAULT_HORIZONTAL_FOV_RAD,
            ).value
        )
        self.image_width = int(
            self.declare_parameter('image_width', DEFAULT_IMAGE_WIDTH).value
        )
        self.image_height = int(
            self.declare_parameter('image_height', DEFAULT_IMAGE_HEIGHT).value
        )
        self.near_clip_m = float(
            self.declare_parameter('camera_near_clip_m', DEFAULT_NEAR_CLIP_M).value
        )
        self.far_clip_m = float(
            self.declare_parameter('camera_far_clip_m', DEFAULT_FAR_CLIP_M).value
        )
        self.block_center_z_offset_m = float(
            self.declare_parameter('block_center_z_offset_m', 0.03).value
        )
        self.bin_center_z_offset_m = float(
            self.declare_parameter('bin_center_z_offset_m', 0.08).value
        )
        if self.near_clip_m < 0.0:
            self.get_logger().warning(
                'camera_near_clip_m was negative; using absolute value.'
            )
            self.near_clip_m = abs(self.near_clip_m)
        if self.far_clip_m < self.near_clip_m:
            self.get_logger().warning(
                'camera_far_clip_m was less than near clip; using near clip.'
            )
            self.far_clip_m = self.near_clip_m

        self._latest_rover_pose: Pose | None = None
        self._latest_rover_pose_time = None
        self._latest_intrinsics: CameraIntrinsics = camera_intrinsics_from_fov(
            width=self.image_width,
            height=self.image_height,
            horizontal_fov_rad=self.horizontal_fov_rad,
        )
        self._missing_inputs_warned = False
        self._invalid_camera_info_warned = False
        self._occluders = DEFAULT_OCCLUDERS

        blockbin_colors = {
            'red': BlockBinColor.RED,
            'blue': BlockBinColor.BLUE,
            'yellow': BlockBinColor.YELLOW,
        }

        self.subs = []
        self.subs.append(
            self.create_subscription(
                msg_type=PoseArray,
                topic=self.rover_pose_topic,
                callback=self.rover_pose_callback,
                qos_profile=10,
            )
        )
        self.subs.append(
            self.create_subscription(
                msg_type=CameraInfo,
                topic=self.camera_info_topic,
                callback=self.camera_info_callback,
                qos_profile=10,
            )
        )

        # Build all the pubs and subs
        for block_str, color in blockbin_colors.items():
            self.get_logger().info(
                f'Creating block subscription to '
                f'/model/block_{block_str}_1/pose'
            )
            self.subs.append(
                self.create_subscription(
                    msg_type=PoseArray,
                    topic=f'/model/block_{block_str}_1/pose',
                    callback=partial(self.block_obs_callback, color=color),
                    qos_profile=1,
                )
            )
        self.block_publisher = self.create_publisher(
            BlockPoseObservation,
            topic='cv/block_poses',
            qos_profile=1,
        )

        for bin_str, color in blockbin_colors.items():
            self.get_logger().info(
                f'Creating bin subscription to /model/bin_{bin_str}_1/pose'
            )
            self.subs.append(
                self.create_subscription(
                    msg_type=PoseArray,
                    topic=f'/model/bin_{bin_str}_1/pose',
                    callback=partial(self.bin_obs_callback, color=color),
                    qos_profile=1,
                )
            )
        self.bin_publisher = self.create_publisher(
            BinPoseObservation,
            topic='cv/bin_poses',
            qos_profile=1,
        )

    def rover_pose_callback(self, msg: PoseArray) -> None:
        """Store the latest rover world pose."""
        if not msg.poses:
            return

        self._latest_rover_pose = msg.poses[0]
        self._latest_rover_pose_time = self.get_clock().now()

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Store the latest usable camera intrinsics."""
        intrinsics = camera_intrinsics_from_info(msg)
        if intrinsics is None:
            if not self._invalid_camera_info_warned:
                self.get_logger().warning(
                    'Ignoring invalid depth camera info; observations will '
                    'remain suppressed until valid camera info arrives.'
                )
                self._invalid_camera_info_warned = True
            return

        self._latest_intrinsics = intrinsics

    def fill_obs_header(
        self,
        obs: BlockPoseObservation | BinPoseObservation,
        msg: PoseArray,
    ) -> None:
        """Add the shared header information to an observation envelope."""
        obs.header.stamp = msg.header.stamp
        obs.header.frame_id = self.observation_frame_id
        obs.yolo_fps = 0.0

    def source_pose(self, msg: PoseArray) -> Pose | None:
        """Return the first Gazebo pose array entry."""
        if not msg.poses:
            self.get_logger().warning(
                'Received empty PoseArray from simulator.'
            )
            return None

        return msg.poses[0]

    def build_pose(self, source_pose: Pose) -> Pose:
        """Build a noisy pose from a Gazebo pose."""
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
        """Apply configured Gaussian noise to a scalar value."""
        if not self.add_gaussian_noise or self.position_noise_stddev_m == 0.0:
            return value
        return value + gauss(0.0, self.position_noise_stddev_m)

    def visible_projection(
        self,
        source_pose: Pose,
        center_z_offset_m: float,
    ) -> Projection | None:
        """Return camera projection if the source pose centre is planarly visible."""
        if not self.visibility_inputs_ready():
            return None

        assert self._latest_rover_pose is not None
        assert self._latest_intrinsics is not None

        point_world = target_centre_from_pose(source_pose, center_z_offset_m)
        camera_origin_world, _ = camera_pose_from_rover(self._latest_rover_pose)
        projection = project_world_point_to_camera_planar(
            point_world=point_world,
            rover_pose=self._latest_rover_pose,
            intrinsics=self._latest_intrinsics,
            near_clip_m=self.near_clip_m,
            far_clip_m=self.far_clip_m,
        )
        if projection is None:
            return None

        if self.enable_wall_occlusion and is_occluded_by_boxes(
            camera_origin_world=camera_origin_world,
            target_world=point_world,
            boxes=self._occluders,
        ):
            return None

        return projection

    def visibility_inputs_ready(self) -> bool:
        """Return true when all required visibility inputs are fresh."""
        ready = (
            self._latest_rover_pose is not None
            and self._latest_rover_pose_time is not None
        )
        if not ready:
            if not self._missing_inputs_warned:
                self.get_logger().warning(
                    'Suppressing vision observations until rover pose data '
                    'has arrived.'
                )
                self._missing_inputs_warned = True
            return False

        now = self.get_clock().now()
        rover_pose_age_sec = (
            now - self._latest_rover_pose_time
        ).nanoseconds / 1e9

        return rover_pose_age_sec <= self.camera_data_timeout_sec

    def block_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Convert a Gazebo block pose message into an observation."""
        source_pose = self.source_pose(msg)
        if source_pose is None:
            return

        projection = self.visible_projection(
            source_pose,
            self.block_center_z_offset_m,
        )
        if projection is None:
            return

        pose = self.build_pose(source_pose)
        obs = BlockPoseObservation()
        self.fill_obs_header(obs, msg)

        detection = BlockPose()
        detection.id = 0
        detection.pose = pose
        detection.confidence = 0.0
        detection.age = 0
        detection.missed = 0
        detection.center_u = projection.u
        detection.center_v = projection.v
        detection.color = color
        detection.color_confidence = 0.0
        detection.color_votes = 0
        detection.shape_label = 'cube'
        detection.shape_confidence = 0.0
        detection.shape_votes = 0

        obs.observations.append(detection)  # type: ignore

        self.block_publisher.publish(obs)

    def bin_obs_callback(self, msg: PoseArray, color: int) -> None:
        """Convert a Gazebo bin pose message into an observation."""
        source_pose = self.source_pose(msg)
        if source_pose is None:
            return

        projection = self.visible_projection(
            source_pose,
            self.bin_center_z_offset_m,
        )
        if projection is None:
            return

        pose = self.build_pose(source_pose)
        obs = BinPoseObservation()
        self.fill_obs_header(obs, msg)

        detection = BinPose()
        detection.id = 0
        detection.pose = pose
        detection.confidence = 0.0
        detection.age = 0
        detection.missed = 0
        detection.center_u = projection.u
        detection.center_v = projection.v
        detection.color = color
        detection.color_confidence = 0.0
        detection.color_votes = 0

        obs.observations.append(detection)  # type: ignore

        self.bin_publisher.publish(obs)


def main():
    """Run the vision stub."""
    try:
        rclpy.init()
        node = VisionStub()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
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
