import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ApproachPose2D:
    x: float
    y: float
    yaw: float


def compute_standoff_pose(
    *,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    target_x: float,
    target_y: float,
    stand_off_distance: float,
    min_goal_vector_length: float,
) -> ApproachPose2D:
    if stand_off_distance < 0.0:
        raise ValueError("stand_off_distance must be non-negative")
    if min_goal_vector_length <= 0.0:
        raise ValueError("min_goal_vector_length must be positive")

    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)

    if distance < min_goal_vector_length:
        return ApproachPose2D(x=robot_x, y=robot_y, yaw=robot_yaw)

    target_yaw = math.atan2(dy, dx)
    if distance <= stand_off_distance:
        return ApproachPose2D(x=robot_x, y=robot_y, yaw=target_yaw)

    scale = stand_off_distance / distance
    return ApproachPose2D(
        x=target_x - (dx * scale),
        y=target_y - (dy * scale),
        yaw=target_yaw,
    )


def quaternion_to_yaw(*, x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    half_yaw = yaw / 2.0
    return math.sin(half_yaw), math.cos(half_yaw)
