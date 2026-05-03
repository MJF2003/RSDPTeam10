import sys
from pathlib import Path

import numpy as np


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from leo_explore.explore_action_server import ExploreActionServer  # noqa: E402


def make_explorer():
    explorer = object.__new__(ExploreActionServer)
    explorer.free_threshold = 60
    explorer.goal_backoff_cells = 2
    explorer.max_goal_backoff_cells = 6
    explorer.min_goal_clearance_m = 0.1
    explorer.occupied_threshold = 65
    explorer.frontier_direction_radius_cells = 2
    explorer.frontier_min_unknown_depth_m = 0.2
    explorer.frontier_corridor_half_width_m = 0.0
    explorer._clearance_offsets_cache = {}
    return explorer


def test_safe_goal_mask_requires_clearance_from_non_free_cells():
    explorer = make_explorer()
    grid = np.zeros((7, 7), dtype=np.int16)
    grid[3, 3] = 100
    grid[1, 5] = -1

    safe = explorer.build_safe_goal_mask(grid, clearance_cells=1)

    assert not safe[3, 3]
    assert not safe[3, 2]
    assert not safe[1, 5]
    assert not safe[1, 4]
    assert safe[5, 1]


def test_reachable_distances_do_not_cross_unsafe_wall():
    explorer = make_explorer()
    safe = np.ones((5, 5), dtype=bool)
    safe[:, 2] = False

    distances = explorer.compute_reachable_distances(safe, 0, 2)

    assert distances[2, 1] >= 0
    assert distances[2, 3] == -1


def test_frontier_unknown_open_rejects_wall_beyond_frontier():
    explorer = make_explorer()
    grid = np.zeros((7, 11), dtype=np.int16)
    grid[:, 5] = 100

    assert explorer.frontier_unknown_is_open(
        grid,
        frontier_gx=4,
        frontier_gy=3,
        ux=1.0,
        uy=0.0,
        info=type('Info', (), {'resolution': 0.1})(),
    ) is None


def test_backoff_uses_frontier_normal_to_place_goal_on_known_side():
    explorer = make_explorer()
    grid = np.zeros((7, 11), dtype=np.int16)
    safe = explorer.is_free_grid(grid)
    distances = explorer.compute_reachable_distances(safe, 2, 3)

    goal = explorer.backoff_goal_from_frontier(
        grid,
        frontier_gx=8,
        frontier_gy=3,
        unknown_ux=1.0,
        unknown_uy=0.0,
        safe_goal_mask=safe,
        reachable_distances=distances,
    )

    assert goal == (6, 3)


def test_cluster_traversal_goal_targets_open_unknown_side():
    explorer = make_explorer()
    grid = np.zeros((7, 12), dtype=np.int16)
    grid[:, 8:] = -1
    cluster = [(7, 2), (7, 3), (7, 4)]
    safe = explorer.is_free_grid(grid)
    distances = explorer.compute_reachable_distances(safe, 2, 3)

    goal = explorer.pick_cluster_traversal_goal(
        grid,
        cluster,
        info=type('Info', (), {'resolution': 0.1})(),
        safe_goal_mask=safe,
        reachable_distances=distances,
    )

    assert goal is not None
    goal_gx, _, frontier_gx, _, unknown_ux, _, _ = goal
    assert goal_gx < frontier_gx
    assert unknown_ux > 0.0
