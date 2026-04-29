import math
from dataclasses import dataclass
from typing import Iterable

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


@dataclass(frozen=True)
class SemanticObstacle:
    x: float
    y: float
    radius: float


def generate_disk_points(
    obstacles: Iterable[SemanticObstacle],
    point_spacing: float,
    point_z: float,
) -> list[tuple[float, float, float]]:
    """Return xyz points that fill each obstacle footprint as a 2D disk."""
    if point_spacing <= 0.0:
        raise ValueError("point_spacing must be positive")

    points: list[tuple[float, float, float]] = []
    for obstacle in obstacles:
        if obstacle.radius <= 0.0:
            continue

        steps = max(0, math.ceil(obstacle.radius / point_spacing))
        for x_step in range(-steps, steps + 1):
            x_offset = x_step * point_spacing
            for y_step in range(-steps, steps + 1):
                y_offset = y_step * point_spacing
                if math.hypot(x_offset, y_offset) > obstacle.radius:
                    continue

                points.append(
                    (
                        float(obstacle.x + x_offset),
                        float(obstacle.y + y_offset),
                        float(point_z),
                    )
                )

    return points


def build_obstacle_cloud(
    header: Header,
    obstacles: Iterable[SemanticObstacle],
    point_spacing: float,
    point_z: float,
) -> PointCloud2:
    points = generate_disk_points(
        obstacles=obstacles,
        point_spacing=point_spacing,
        point_z=point_z,
    )
    return point_cloud2.create_cloud_xyz32(header, points)
