import math

import pytest

from rover_controller.navigation_goals import (
    compute_standoff_pose,
    quaternion_to_yaw,
    yaw_to_quaternion,
)


def test_compute_standoff_pose_offsets_along_robot_to_target_line():
    pose = compute_standoff_pose(
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        target_x=1.0,
        target_y=0.0,
        stand_off_distance=0.6,
        min_goal_vector_length=1e-3,
    )

    assert pose.x == pytest.approx(0.4)
    assert pose.y == pytest.approx(0.0)
    assert pose.yaw == pytest.approx(0.0)


def test_compute_standoff_pose_keeps_current_position_when_already_close():
    pose = compute_standoff_pose(
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        target_x=0.3,
        target_y=0.4,
        stand_off_distance=0.6,
        min_goal_vector_length=1e-3,
    )

    assert pose.x == pytest.approx(0.0)
    assert pose.y == pytest.approx(0.0)
    assert pose.yaw == pytest.approx(math.atan2(0.4, 0.3))


def test_compute_standoff_pose_uses_current_yaw_when_target_is_degenerate():
    pose = compute_standoff_pose(
        robot_x=2.0,
        robot_y=-1.5,
        robot_yaw=1.2,
        target_x=2.0,
        target_y=-1.5,
        stand_off_distance=0.6,
        min_goal_vector_length=1e-3,
    )

    assert pose.x == pytest.approx(2.0)
    assert pose.y == pytest.approx(-1.5)
    assert pose.yaw == pytest.approx(1.2)


def test_yaw_quaternion_round_trip():
    yaw = -0.75
    qz, qw = yaw_to_quaternion(yaw)

    assert quaternion_to_yaw(x=0.0, y=0.0, z=qz, w=qw) == pytest.approx(yaw)
