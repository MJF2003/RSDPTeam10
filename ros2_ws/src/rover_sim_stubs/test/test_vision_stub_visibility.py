"""Tests for vision stub depth-camera visibility helpers."""

from types import SimpleNamespace

from geometry_msgs.msg import Pose

import numpy as np

import pytest

from rover_sim_stubs.vision_stub import (
    BASE_TO_CAMERA_OPTICAL_ROTATION,
    camera_intrinsics_from_info,
    camera_intrinsics_from_fov,
    CAMERA_TRANSLATION_BASE_M,
    CameraIntrinsics,
    DEFAULT_OCCLUDERS,
    is_occluded_by_boxes,
    segment_intersection_t_for_aabb,
    box_from_center_size,
    project_world_point_to_camera,
    Projection,
    target_centre_from_pose,
)


def make_pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    """Build a simple map-frame pose."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.z = np.sin(yaw / 2.0)
    pose.orientation.w = np.cos(yaw / 2.0)
    return pose


def test_project_world_point_at_optical_axis_centre():
    """A point on the optical axis projects to the camera centre."""
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        fx=900.0,
        fy=900.0,
        cx=640.0,
        cy=360.0,
    )
    depth_m = 2.0
    point_world = (
        CAMERA_TRANSLATION_BASE_M
        + (BASE_TO_CAMERA_OPTICAL_ROTATION @ np.array([0.0, 0.0, depth_m]))
    )

    projection = project_world_point_to_camera(
        point_world=point_world,
        rover_pose=make_pose(),
        intrinsics=intrinsics,
        near_clip_m=0.28,
        far_clip_m=10.0,
    )

    assert projection == Projection(
        u=640,
        v=360,
        depth_m=pytest.approx(depth_m),
    )


def test_project_world_point_rejects_behind_camera():
    """A point behind the optical frame is not visible."""
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        fx=900.0,
        fy=900.0,
        cx=640.0,
        cy=360.0,
    )
    point_world = (
        CAMERA_TRANSLATION_BASE_M
        + (BASE_TO_CAMERA_OPTICAL_ROTATION @ np.array([0.0, 0.0, -1.0]))
    )

    projection = project_world_point_to_camera(
        point_world=point_world,
        rover_pose=make_pose(),
        intrinsics=intrinsics,
        near_clip_m=0.28,
        far_clip_m=10.0,
    )

    assert projection is None


def test_intrinsics_from_fov_uses_image_centre():
    """FOV-derived intrinsics use the configured image centre."""
    intrinsics = camera_intrinsics_from_fov(
        width=1280,
        height=720,
        horizontal_fov_rad=1.204,
    )

    assert intrinsics.cx == pytest.approx(640.0)
    assert intrinsics.cy == pytest.approx(360.0)
    assert intrinsics.fx == pytest.approx(intrinsics.fy)


def test_segment_intersection_returns_first_wall_hit():
    """A sightline through an AABB returns the first hit fraction."""
    box = box_from_center_size(
        'test_wall',
        center=(1.0, 0.0, 0.2),
        size=(0.2, 2.0, 0.4),
    )

    t_hit = segment_intersection_t_for_aabb(
        start=np.array([0.0, 0.0, 0.2]),
        end=np.array([2.0, 0.0, 0.2]),
        box=box,
    )

    assert t_hit == pytest.approx(0.45)


def test_wall_between_camera_and_target_occludes():
    """A wall box between camera and target blocks visibility."""
    box = box_from_center_size(
        'test_wall',
        center=(1.0, 0.0, 0.2),
        size=(0.2, 2.0, 0.4),
    )

    assert is_occluded_by_boxes(
        camera_origin_world=np.array([0.0, 0.0, 0.2]),
        target_world=np.array([2.0, 0.0, 0.2]),
        boxes=(box,),
    )


def test_wall_beyond_target_does_not_occlude():
    """A wall behind the target does not block visibility."""
    box = box_from_center_size(
        'test_wall',
        center=(1.0, 0.0, 0.2),
        size=(0.2, 2.0, 0.4),
    )

    assert not is_occluded_by_boxes(
        camera_origin_world=np.array([0.0, 0.0, 0.2]),
        target_world=np.array([0.5, 0.0, 0.2]),
        boxes=(box,),
    )


def test_pick_place_screen_wall_occludes_central_sightline():
    """The arena screen wall blocks targets behind it."""
    assert is_occluded_by_boxes(
        camera_origin_world=np.array([0.0, 0.0, 0.2]),
        target_world=np.array([2.4, 0.0, 0.2]),
        boxes=DEFAULT_OCCLUDERS,
    )

def test_target_centre_applies_local_z_offset():
    """The configured object-centre offset is applied in pose coordinates."""
    centre = target_centre_from_pose(make_pose(x=1.0, y=2.0, z=3.0), 0.08)

    np.testing.assert_allclose(centre, np.array([1.0, 2.0, 3.08]))


def test_camera_intrinsics_rejects_zero_focal_length():
    """Camera info without focal length is unusable."""
    msg = SimpleNamespace(
        width=1280,
        height=720,
        k=[0.0, 0.0, 640.0, 0.0, 0.0, 360.0, 0.0, 0.0, 1.0],
    )

    assert camera_intrinsics_from_info(msg) is None
