"""
tests/test_corridor.py — step05_forward_path_filter.md Part A, §A4.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from pyquaternion import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf.geometry.corridor import (
    candidate_gate,
    corridor_contains,
    footprint_intersects_corridor,
    nearest_point_on_footprint,
)
from ttcf.geometry.transforms import FrameContext, ego_to_global, global_to_ego

HALF_WIDTH = 1.2
MAX_RANGE = 60.0
MARGIN = 1.5


# ── Test 1: points just inside/outside each edge ──────────────────────────

def test_edges_front_rear_left_right_max_range():
    # Front edge (max_range): boundary inclusive.
    assert corridor_contains((MAX_RANGE, 0.0), HALF_WIDTH, MAX_RANGE)
    assert not corridor_contains((MAX_RANGE + 0.01, 0.0), HALF_WIDTH, MAX_RANGE)

    # Rear edge (x=0): strictly excluded, even though it's "at" the ego.
    assert not corridor_contains((0.0, 0.0), HALF_WIDTH, MAX_RANGE)
    assert corridor_contains((0.01, 0.0), HALF_WIDTH, MAX_RANGE)

    # Left/right edges (|y| = half_width): boundary inclusive.
    assert corridor_contains((10.0, HALF_WIDTH), HALF_WIDTH, MAX_RANGE)
    assert corridor_contains((10.0, -HALF_WIDTH), HALF_WIDTH, MAX_RANGE)
    assert not corridor_contains((10.0, HALF_WIDTH + 0.001), HALF_WIDTH, MAX_RANGE)
    assert not corridor_contains((10.0, -HALF_WIDTH - 0.001), HALF_WIDTH, MAX_RANGE)


# ── Test 2: candidate gate is a strict superset of the tight corridor ────

def test_candidate_gate_is_superset_of_tight_corridor():
    rng = np.random.default_rng(0)
    any_extra = False
    for _ in range(2000):
        x = rng.uniform(-5, MAX_RANGE + 5)
        y = rng.uniform(-(HALF_WIDTH + MARGIN + 1), HALF_WIDTH + MARGIN + 1)
        tight = corridor_contains((x, y), HALF_WIDTH, MAX_RANGE)
        wide = corridor_contains((x, y), HALF_WIDTH + MARGIN, MAX_RANGE)
        if tight:
            assert wide, f"point ({x},{y}) in tight corridor but not the wider gate"
        if wide and not tight:
            any_extra = True
    assert any_extra, "expected at least some points only the wider gate admits"


# ── Test 3: a point behind the ego is never in the corridor ──────────────

def test_point_behind_ego_never_in_corridor():
    rng = np.random.default_rng(1)
    for _ in range(500):
        x = rng.uniform(-100, 0)  # behind or level with ego
        y = rng.uniform(-50, 50)
        assert not corridor_contains((x, y), HALF_WIDTH, MAX_RANGE)
        assert not corridor_contains((x, y), HALF_WIDTH + MARGIN, MAX_RANGE)


# ── Test 4: transform round trip -- membership stable under round trip ───

def test_membership_stable_under_transform_round_trip():
    cs = {"translation": [1.2, 0.0, 1.8], "rotation": [1.0, 0.0, 0.0, 0.0]}
    ep = {
        "translation": [500.0, -200.0, 0.0],
        "rotation": Quaternion(axis=[0, 0, 1], angle=1.13).elements.tolist(),
    }
    ctx = FrameContext(calibrated_sensor=cs, ego_pose=ep)

    point_global = np.array([[520.0, -185.0, 0.0]])
    ref_xy_ego_1 = global_to_ego(point_global, ctx)[0][:2]
    membership_1 = corridor_contains(ref_xy_ego_1, HALF_WIDTH, MAX_RANGE)

    round_tripped_global = ego_to_global(np.array([ref_xy_ego_1]), ctx)
    ref_xy_ego_2 = global_to_ego(round_tripped_global, ctx)[0][:2]
    membership_2 = corridor_contains(ref_xy_ego_2, HALF_WIDTH, MAX_RANGE)

    np.testing.assert_allclose(ref_xy_ego_1, ref_xy_ego_2, atol=1e-9)
    assert membership_1 == membership_2


# ── Test 5: GT footprint partly overlapping the corridor edge ────────────

def _box_footprint(cx, cy, length, width, yaw=0.0) -> np.ndarray:
    """4 corners of a box centred at (cx,cy), ego frame, given yaw (rad)."""
    hl, hw = length / 2.0, width / 2.0
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return (local @ R.T) + np.array([cx, cy])


def test_footprint_intersects_corridor_per_nearest_point_rule():
    # Entirely inside the tight corridor.
    inside = _box_footprint(cx=10.0, cy=0.0, length=4.6, width=1.9)
    assert footprint_intersects_corridor(inside, HALF_WIDTH, MAX_RANGE)

    # Entirely outside, well clear of the corridor (parked well off to the side).
    outside = _box_footprint(cx=10.0, cy=5.0, length=4.6, width=1.9)
    assert not footprint_intersects_corridor(outside, HALF_WIDTH, MAX_RANGE)

    # Straddling the right edge (half_width=1.2): box centred at y=1.5 with
    # width 1.9 spans y in [0.55, 2.45] -- its nearest point to the ego
    # origin has y ~ 0.55, inside the corridor's |y|<=1.2 band, so it
    # counts as intersecting under the nearest-point rule even though most
    # of the box's footprint (and its centroid) sits outside the corridor.
    straddling = _box_footprint(cx=10.0, cy=1.5, length=4.6, width=1.9)
    nearest = nearest_point_on_footprint(straddling)
    assert abs(nearest[1]) < HALF_WIDTH + 1e-6  # nearest point IS inside the band
    assert footprint_intersects_corridor(straddling, HALF_WIDTH, MAX_RANGE)

    # Push it further out so even the nearest point clears the corridor.
    clearly_outside = _box_footprint(cx=10.0, cy=3.5, length=4.6, width=1.9)
    assert not footprint_intersects_corridor(clearly_outside, HALF_WIDTH, MAX_RANGE)
