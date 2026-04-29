import math
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped
import pytest
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from rover_controller.rover_controller import ControllerNode
from rover_controller.semantic_obstacles import (
    SemanticObstacle,
    build_obstacle_cloud,
    generate_disk_points,
)


def test_generate_disk_points_returns_empty_for_no_obstacles():
    assert generate_disk_points([], point_spacing=0.05, point_z=0.25) == []


def test_generate_disk_points_fill_disk_with_configured_height():
    obstacle = SemanticObstacle(x=1.0, y=2.0, radius=0.1)

    points = generate_disk_points([obstacle], point_spacing=0.05, point_z=0.25)

    assert (1.0, 2.0, 0.25) in points
    assert len(points) > 1
    for x, y, z in points:
        assert math.hypot(x - obstacle.x, y - obstacle.y) <= obstacle.radius + 1e-9
        assert z == pytest.approx(0.25)


def test_generate_disk_points_rejects_non_positive_spacing():
    with pytest.raises(ValueError, match="point_spacing"):
        generate_disk_points(
            [SemanticObstacle(x=0.0, y=0.0, radius=0.1)],
            point_spacing=0.0,
            point_z=0.25,
        )


def test_build_obstacle_cloud_uses_header_and_points():
    header = Header()
    header.frame_id = "map"
    obstacle = SemanticObstacle(x=1.0, y=2.0, radius=0.05)

    cloud = build_obstacle_cloud(
        header=header,
        obstacles=[obstacle],
        point_spacing=0.05,
        point_z=0.25,
    )

    assert cloud.header.frame_id == "map"
    points = list(
        point_cloud2.read_points(
            cloud,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
    )
    assert len(points) > 0
    assert any(
        x == pytest.approx(1.0)
        and y == pytest.approx(2.0)
        and z == pytest.approx(0.25)
        for x, y, z in points
    )


def make_point(x, y, frame_id="map"):
    point = PointStamped()
    point.header.frame_id = frame_id
    point.point.x = x
    point.point.y = y
    return point


def make_controller_for_obstacle_tests():
    controller = ControllerNode.__new__(ControllerNode)
    controller.map_frame = "map"
    controller.semantic_block_radius_m = 0.25
    controller.semantic_bin_radius_m = 0.35
    controller.semantic_fixed_keepout_radius_m = 0.45
    return controller


def test_controller_obstacles_exclude_collected_blocks_but_keep_fixed_positions():
    controller = make_controller_for_obstacle_tests()
    controller.blocks = [
        SimpleNamespace(id=1, position=make_point(1.0, 2.0)),
        SimpleNamespace(id=2, position=make_point(3.0, 4.0)),
    ]
    controller.collected = {1}
    controller.bins = []
    controller.fixed_keepout_positions = {1: make_point(1.0, 2.0)}

    obstacles = controller._build_semantic_obstacles()

    assert SemanticObstacle(x=3.0, y=4.0, radius=0.25) in obstacles
    assert SemanticObstacle(x=1.0, y=2.0, radius=0.25) not in obstacles
    assert SemanticObstacle(x=1.0, y=2.0, radius=0.45) in obstacles


def test_controller_obstacles_include_bins():
    controller = make_controller_for_obstacle_tests()
    controller.blocks = []
    controller.collected = set()
    controller.bins = [SimpleNamespace(position=make_point(5.0, 6.0))]
    controller.fixed_keepout_positions = {}

    assert controller._build_semantic_obstacles() == [
        SemanticObstacle(x=5.0, y=6.0, radius=0.35)
    ]
