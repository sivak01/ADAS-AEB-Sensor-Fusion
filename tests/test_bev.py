"""
tests/test_bev.py — step12_bev_visualization.md §6 (synthetic only).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.geometry.corridor import candidate_gate, corridor_contains
from ttcf.viz.bev import (
    corridor_polygon_ego,
    display_xlim,
    display_ylim,
    gt_category_allowed,
    to_display,
    ttc_color,
)


# ── Test 1: ego always renders at the display origin (DEC-11) ────────────

def test_ego_always_at_display_origin():
    assert to_display([0.0, 0.0]) == (0.0, 0.0)


# ── Test 2: heading-up mapping on a known point ───────────────────────────

def test_to_display_known_point_forward_and_left():
    # 10m forward, 5m left (ego frame) -> screen: left of center, further up.
    disp_x, disp_y = to_display([10.0, 5.0])
    assert disp_x == -5.0  # left (+y_ego) appears on-screen left (negative display_x)
    assert disp_y == 10.0  # forward (+x_ego) appears "up" (positive display_y)


def test_to_display_right_side_point():
    disp_x, disp_y = to_display([10.0, -5.0])
    assert disp_x == 5.0  # right (-y_ego) appears on-screen right


# ── Test 3: TTC color bins match config thresholds exactly at the boundary ─

def test_ttc_color_boundaries_match_config():
    aeb = config.TTC_AEB_S.value
    gradual = config.TTC_GRADUAL_S.value
    assert ttc_color(aeb) == "red"  # at or below AEB threshold -> red
    assert ttc_color(aeb - 0.01) == "red"
    assert ttc_color(aeb + 0.01) == "orange"  # just above AEB, at/below GRADUAL -> orange
    assert ttc_color(gradual) == "orange"
    assert ttc_color(gradual + 0.01) == "green"  # above both -> green


def test_ttc_color_non_finite_is_gray():
    assert ttc_color(float("inf")) == "gray"
    assert ttc_color(float("nan")) == "gray"


# ── Test 4: corridor polygon matches candidate_gate's own geometry ───────

def test_corridor_polygon_matches_candidate_gate_geometry():
    corners = corridor_polygon_ego()
    half_width = config.CORRIDOR_HALF_WIDTH_M.value + config.CANDIDATE_EXTRA_MARGIN_M.value
    max_range = config.CORRIDOR_MAX_RANGE_M.value

    # A point just inside the polygon's lateral edge should pass candidate_gate;
    # just outside should fail -- the SAME half_width/max_range candidate_gate uses.
    just_inside = np.array([max_range / 2, half_width - 0.05])
    just_outside = np.array([max_range / 2, half_width + 0.05])
    assert candidate_gate(just_inside)
    assert not candidate_gate(just_outside)

    # Polygon's own lateral extent (max |y| among corners) must equal half_width exactly.
    assert np.max(np.abs(corners[:, 1])) == half_width
    assert np.max(corners[:, 0]) == max_range

    # And corridor_contains with these exact numbers agrees with the polygon's own bounds.
    assert corridor_contains(just_inside, half_width, max_range)
    assert not corridor_contains(just_outside, half_width, max_range)


# ── Test 5: GT category filter matches DEC-2's GT_CATEGORIES exactly ─────

def test_gt_category_filter_matches_config_list():
    allowed = set(config.GT_CATEGORIES.value)
    assert "vehicle.car" in allowed  # sanity: this project's own list really has it
    assert gt_category_allowed("vehicle.car")
    assert not gt_category_allowed("movable_object.barrier")  # DEC-2: never a GT event
    assert not gt_category_allowed("static_object.bicycle_rack")
    # Every category GT_CATEGORIES itself lists must be "allowed" -- no drift.
    for cat in allowed:
        assert gt_category_allowed(cat)


# ── Test 6: viewport limits are internally consistent with to_display ────

def test_display_limits_consistent_with_to_display_of_view_range():
    from ttcf.viz.bev import VIEW_X_RANGE, VIEW_Y_RANGE

    x_lo, x_hi = display_xlim()
    y_lo, y_hi = display_ylim()
    assert (y_lo, y_hi) == VIEW_X_RANGE  # forward range becomes the display Y range

    # The four corners of the ego-frame view box must land inside [x_lo,x_hi]x[y_lo,y_hi].
    for ex in VIEW_X_RANGE:
        for ey in VIEW_Y_RANGE:
            dx, dy = to_display([ex, ey])
            assert x_lo <= dx <= x_hi
            assert y_lo <= dy <= y_hi
