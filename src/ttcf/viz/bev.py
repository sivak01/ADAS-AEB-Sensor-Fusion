"""
src/ttcf/viz/bev.py — STEP 12: bird's-eye-view drawing primitives.

Pure drawing/geometry functions only — no NuScenes access, no pipeline-
running logic. `scripts/render_bev.py` runs the real pipeline and calls
these with already-computed data (ego-frame coordinates, TTCResult,
category strings). Kept separate so the drawing math itself is testable
with plain synthetic numbers (tests/test_bev.py), matching this
project's existing split between "math/geometry" modules and the
scripts that drive real data through them (e.g. geometry/corridor.py vs.
scripts/run_pipeline.py).

Rebuilt fresh — no precedent is ported from the predecessor project's own
BEV code (`Step_8_BEV_Fusion_Video.ipynb`, V1 only, V2 never built one):
it's built on V1's file-based per-step handoff and per-sensor trackers
merged after the fact, neither of which this project has. Three of its
*ideas* are kept (see step12_bev_visualization.md §1): per-scene output,
fps measured from real deltas, and filtering GT to vehicle/pedestrian
classes — none of its code.

Coordinate convention: callers pass EGO-FRAME coordinates (x forward, y
left — corridor.py's own convention, already rotated into the ego's own
heading by whatever produced them, typically `global_to_ego`). This
module's only job is the final ego-frame -> screen-axis relabeling
(DEC-11, heading-up: ego's own forward direction always points "up" on
screen) — never a second rotation.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ttcf import config

# DEC-12: fixed viewport every frame, derived from this project's own
# corridor config (not auto-fit -- a fixed scale is what makes TTC-
# urgency color meaningful frame to frame). Forward range matches
# CORRIDOR_MAX_RANGE_M (+5m margin beyond it); lateral/rear extent
# matches this project's own existing adapter-evidence script
# convention (scripts/*_adapter_evidence.py's fixed +-5/60m x +-15m
# window), not re-derived per frame.
VIEW_X_RANGE = (-5.0, config.CORRIDOR_MAX_RANGE_M.value + 5.0)  # ego-frame forward (m)
VIEW_Y_RANGE = (-15.0, 15.0)  # ego-frame lateral (m)

# Purely cosmetic drawing sizes (do not affect any computed result, so
# not G12 config params -- same precedent as the existing adapter-
# evidence scripts' own hardcoded plot extents).
EGO_MARKER_LENGTH_M = 4.0
EGO_MARKER_WIDTH_M = 1.8


def to_display(xy_ego) -> tuple:
    """DEC-11 (heading-up): ego frame (x forward, y left) -> screen axes
    (x right, y up). display_x = -y_ego (ego's left appears on-screen
    left), display_y = x_ego (forward is up). Ego is always at (0, 0)
    in its own frame by construction, so this alone puts the ego marker
    at a fixed screen position every frame -- no separate rotation step."""
    x, y = float(xy_ego[0]), float(xy_ego[1])
    return -y, x


def display_xlim() -> tuple:
    y_lo, y_hi = VIEW_Y_RANGE
    return -y_hi, -y_lo


def display_ylim() -> tuple:
    return VIEW_X_RANGE


def ttc_color(ttc_s: float) -> str:
    """DEC-13: bins derived from this project's own tuned thresholds
    (config.TTC_AEB_S, config.TTC_GRADUAL_S) -- never a second, separate
    hardcoded set of numbers (G12). inf/NaN (NOT_ELIGIBLE or no real
    closing-speed signal) is gray, not green -- "no estimate" must stay
    visually distinct from "confirmed safe" (same principle ttc/estimator.py's
    own TTCReason.NOT_ELIGIBLE path already enforces)."""
    if not np.isfinite(ttc_s):
        return "gray"
    if ttc_s <= config.TTC_AEB_S.value:
        return "red"
    if ttc_s <= config.TTC_GRADUAL_S.value:
        return "orange"
    return "green"


def gt_category_allowed(category: str, cfg=config) -> bool:
    """The exact same class filter DEC-2/GT_CATEGORIES already uses for
    GT event scoping (gt/gt_events.py) -- one shared list, never a second
    hardcoded vehicle/pedestrian filter for this visualization."""
    return category in set(cfg.GT_CATEGORIES.value)


def corridor_polygon_ego(cfg=config) -> np.ndarray:
    """(4,2) ego-frame corners of the CANDIDATE-GATE corridor (the
    widened one `geometry.corridor.candidate_gate` actually tests
    against: CORRIDOR_HALF_WIDTH_M + CANDIDATE_EXTRA_MARGIN_M), traced
    around its perimeter once. Same two config values candidate_gate
    itself reads -- no second, drifted copy of the geometry."""
    half_width = cfg.CORRIDOR_HALF_WIDTH_M.value + cfg.CANDIDATE_EXTRA_MARGIN_M.value
    max_range = cfg.CORRIDOR_MAX_RANGE_M.value
    return np.array(
        [
            [0.0, -half_width],
            [max_range, -half_width],
            [max_range, half_width],
            [0.0, half_width],
        ]
    )


def new_bev_axes(ax, title: Optional[str] = None):
    """Sets up one BEV panel's fixed viewport/aspect/labels (DEC-12).
    Caller supplies `ax` (one matplotlib Axes) and draws content on it
    afterward via draw_ego/draw_corridor/draw_detections/draw_tracks/draw_gt_boxes."""
    ax.set_xlim(*display_xlim())
    ax.set_ylim(*display_ylim())
    ax.set_aspect("equal")
    ax.set_xlabel("lateral (m) — left is screen-left")
    ax.set_ylabel("forward (m) — heading-up")
    if title:
        ax.set_title(title)
    ax.grid(True, alpha=0.2)
    return ax


def draw_ego(ax) -> None:
    """Fixed triangle at the display origin, pointing up (DEC-11)."""
    tip = to_display([EGO_MARKER_LENGTH_M / 2, 0.0])
    left = to_display([-EGO_MARKER_LENGTH_M / 2, EGO_MARKER_WIDTH_M / 2])
    right = to_display([-EGO_MARKER_LENGTH_M / 2, -EGO_MARKER_WIDTH_M / 2])
    ax.fill(
        [tip[0], left[0], right[0]], [tip[1], left[1], right[1]],
        color="black", zorder=5, label="ego",
    )


def draw_corridor(ax, cfg=config) -> None:
    corners_ego = corridor_polygon_ego(cfg)
    disp = np.array([to_display(c) for c in corners_ego])
    disp = np.vstack([disp, disp[0]])  # close the polygon
    ax.plot(disp[:, 0], disp[:, 1], color="steelblue", linestyle="--", alpha=0.6, label="candidate corridor")


def draw_detections(ax, detections_ego, label: Optional[str] = None, color: str = "dodgerblue") -> None:
    """detections_ego: iterable of (2,) ego-frame xy. Drawn small/faint --
    these are raw, pre-gate, pre-track points (step12 §3), not meant to
    dominate the panel the way tracks/GT should."""
    pts = [to_display(d) for d in detections_ego]
    if not pts:
        return
    pts = np.asarray(pts)
    ax.scatter(pts[:, 0], pts[:, 1], s=14, marker="x", color=color, alpha=0.5, label=label, zorder=3)


def draw_tracks(ax, tracks: list) -> None:
    """tracks: list of dicts with keys xy_ego (2,), ttc_s (float),
    track_id (str), vxy_ego (optional (2,), for a small velocity arrow).
    Colored by ttc_color (DEC-13)."""
    for t in tracks:
        disp_x, disp_y = to_display(t["xy_ego"])
        color = ttc_color(t["ttc_s"])
        ax.scatter([disp_x], [disp_y], s=60, color=color, edgecolor="black", zorder=6)
        ax.annotate(t["track_id"], (disp_x, disp_y), fontsize=7, xytext=(3, 3), textcoords="offset points")
        v = t.get("vxy_ego")
        if v is not None:
            dvx, dvy = to_display([t["xy_ego"][0] + v[0], t["xy_ego"][1] + v[1]])
            ax.annotate(
                "", xy=(dvx, dvy), xytext=(disp_x, disp_y),
                arrowprops=dict(arrowstyle="->", color=color, alpha=0.7), zorder=6,
            )


def draw_gt_boxes(ax, boxes_ego: list, color: str = "purple", label: Optional[str] = "GT box (vehicle/ped)") -> None:
    """boxes_ego: list of (4,2) ego-frame footprint corners (gt_events.py's
    own corner convention/order — perimeter once, see its own
    _FOOTPRINT_CORNER_IDX comment). `label` is applied only to the first
    box drawn, so N boxes produce one legend entry, not N."""
    for i, corners in enumerate(boxes_ego):
        disp = np.array([to_display(c) for c in corners])
        disp = np.vstack([disp, disp[0]])
        ax.plot(disp[:, 0], disp[:, 1], color=color, linewidth=1.3, alpha=0.8, zorder=4,
                 label=label if i == 0 else None)


def draw_track_legend_entry(ax) -> None:
    """A single proxy legend entry summarizing the TTC color scheme —
    draw_tracks itself never labels individual points (one per track
    would flood the legend with track_ids, not colors)."""
    for color, text in [("red", "AEB"), ("orange", "GRADUAL"), ("green", "clear"), ("gray", "no estimate")]:
        ax.scatter([], [], color=color, edgecolor="black", s=60, label=f"track ({text})")
