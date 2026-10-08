"""
src/ttcf/geometry/corridor.py — forward-path relevance, Part A (step05 §A2).

All computations are in the EGO FRAME at the detection's own time (x
forward, y left) -- convert via step02's transforms first (G1, G2). This
is entirely new to this project; no precedent exists in the predecessor
repo (checked -- V1/v2 have no corridor/forward-path code at all, since
both were MOT-scoped and tracked everything in the scene, not just a
forward corridor: lessons-from-v1-v2.md).

DEC-1 (nearest-surface-to-ego) governs footprint_intersects_corridor()
below: membership is decided on the NEAREST POINT of the GT footprint to
the ego origin, not the footprint's centroid -- computed exactly (closest
point on the rectangle's boundary/interior to the origin, not just its 4
corners), per "state precisely which rule you implemented" (step05 §A2).

The corridor is straight in the ego frame. Curved paths (turns,
intersections) are a KNOWN LIMITATION, not solved here -- see step05 §A2
("do not build path prediction").
"""
from __future__ import annotations

import numpy as np

from ttcf import config


def corridor_contains(ref_xy_ego, half_width: float, max_range: float) -> bool:
    """0 < x <= max_range and |y| <= half_width, ego frame (x forward, y
    left). Strictly ahead of the ego origin -- x=0 (level with the ego)
    is not "in the corridor"."""
    x, y = float(ref_xy_ego[0]), float(ref_xy_ego[1])
    return 0.0 < x <= max_range and abs(y) <= half_width


def candidate_gate(ref_xy_ego, cfg=config) -> bool:
    """The generous, position-only pre-tracker filter: half_width widened
    by CANDIDATE_EXTRA_MARGIN_M so an object about to enter the tight
    corridor isn't missed before it even reaches the tracker. Position
    only -- velocity-based "about to enter" is Part B's job, once a track
    has a velocity."""
    half_width = cfg.CORRIDOR_HALF_WIDTH_M.value + cfg.CANDIDATE_EXTRA_MARGIN_M.value
    return corridor_contains(ref_xy_ego, half_width, cfg.CORRIDOR_MAX_RANGE_M.value)


def _closest_point_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ab = b - a
    denom = float(ab @ ab)
    if denom == 0.0:
        return a.copy()
    t = float((p - a) @ ab) / denom
    t = min(1.0, max(0.0, t))
    return a + t * ab


def _point_in_convex_polygon(p: np.ndarray, corners: np.ndarray) -> bool:
    """True if p is inside (or on the boundary of) the convex polygon
    given by corners (N,2), in either winding order."""
    n = len(corners)
    signs = []
    for i in range(n):
        a, b = corners[i], corners[(i + 1) % n]
        cross = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        signs.append(cross)
    return all(s >= -1e-9 for s in signs) or all(s <= 1e-9 for s in signs)


def nearest_point_on_footprint(box_corners_ego) -> np.ndarray:
    """The point on the GT box footprint (a convex quadrilateral, ego
    frame) nearest the ego origin (0, 0) -- DEC-1's nearest-surface-to-ego
    rule. If the ego origin is inside the footprint, returns the origin
    itself (distance 0)."""
    corners = np.asarray(box_corners_ego, dtype=float)
    origin = np.zeros(2)
    if _point_in_convex_polygon(origin, corners):
        return origin.copy()

    n = len(corners)
    best_pt, best_d2 = None, np.inf
    for i in range(n):
        a, b = corners[i], corners[(i + 1) % n]
        cp = _closest_point_on_segment(origin, a, b)
        d2 = float((origin - cp) @ (origin - cp))
        if d2 < best_d2:
            best_d2, best_pt = d2, cp
    return best_pt


def footprint_intersects_corridor(box_corners_ego, half_width: float, max_range: float) -> bool:
    """GT footprint membership per DEC-1 (nearest-surface-to-ego): find
    the nearest point of the footprint to the ego origin, then test THAT
    point against the corridor -- not the footprint's centroid, and not a
    full polygon-rectangle overlap test. This means a large object whose
    nearest edge is inside the corridor counts as intersecting even if
    most of its footprint lies outside; a large object that merely
    surrounds the corridor without any part being the "near side" is not
    the case this rule is built for (an object that large enveloping the
    ego is out of scope for this project's vehicle-scale GT)."""
    nearest = nearest_point_on_footprint(box_corners_ego)
    return corridor_contains(nearest, half_width, max_range)
