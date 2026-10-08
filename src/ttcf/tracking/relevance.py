"""
src/ttcf/tracking/relevance.py — STEP 05 Part B: track-level path relevance.

At detection time (Part A, corridor.py) only a position exists, so
relevance there is a generous, position-only gate. Once a track has a
real velocity, a stricter, motion-aware test applies: is it already in
the corridor, or clearly about to enter it. No precedent exists anywhere
in V1/v2 for this (confirmed at step05 Part A) -- entirely new.

is_velocity_eligible() is written once here and meant to be reused by
step07 Part B's TTCEstimator later (same n_updates/velocity-std check,
one implementation -- not duplicated).
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from pyquaternion import Quaternion

from ttcf import config
from ttcf.filtering.ego_state import EgoState
from ttcf.geometry.corridor import candidate_gate, corridor_contains
from ttcf.geometry.transforms import FrameContext, global_to_ego
from ttcf.types import TrackSnapshot


class Relevance(Enum):
    IN_PATH = "IN_PATH"
    ENTERING = "ENTERING"
    OUT = "OUT"


@dataclass(frozen=True)
class RelevanceResult:
    verdict: Relevance
    provisional: bool  # True only when the eligibility fallback was used


def is_velocity_eligible(snapshot: TrackSnapshot, cfg=config) -> bool:
    """Same eligibility step07 Part B's TTCEstimator will use: enough real
    updates, and velocity uncertainty tight enough to trust. Velocity std
    is the WORSE of the two axis stds (sqrt(P[2,2]), sqrt(P[3,3])) -- a
    track isn't eligible unless BOTH velocity components are individually
    well-determined, not just their combined magnitude."""
    if snapshot.n_updates < cfg.MIN_UPDATES_FOR_TTC.value:
        return False
    vel_std = max(float(np.sqrt(snapshot.P[2, 2])), float(np.sqrt(snapshot.P[3, 3])))
    return vel_std <= cfg.MAX_VEL_STD_MPS.value


def _frame_context_from_ego(ego: EgoState) -> FrameContext:
    rotation = Quaternion(axis=[0, 0, 1], angle=ego.heading_rad).elements.tolist()
    return FrameContext(
        calibrated_sensor={"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]},
        ego_pose={"translation": [ego.x, ego.y, 0.0], "rotation": rotation},
    )


def _rotate_2d(v: np.ndarray, angle: float) -> np.ndarray:
    """Rotate a free 2D vector (e.g. velocity) by `angle` -- no
    translation applies to a vector under a rigid transform, so this is
    deliberately NOT routed through global_to_ego (which is for points)."""
    c, s = np.cos(angle), np.sin(angle)
    R = np.array([[c, -s], [s, c]])
    return R @ np.asarray(v, dtype=float)


def path_relevance(snapshot: TrackSnapshot, ego: EgoState, cfg=config) -> RelevanceResult:
    """IN_PATH / ENTERING / OUT for one track at one instant, ego frame.
    Never uses absolute (global) velocity for the lateral test (B3) --
    always relative velocity (object minus ego), rotated to the ego
    frame. Never extrapolates further than ENTERING_HORIZON_S. No lane
    maps or path prediction (B3) -- the corridor stays straight."""
    ctx = _frame_context_from_ego(ego)
    ref_xy_global = snapshot.x[:2]
    ref_xy_ego = global_to_ego(np.array([ref_xy_global]), ctx)[0][:2]

    half_width = cfg.CORRIDOR_HALF_WIDTH_M.value
    max_range = cfg.CORRIDOR_MAX_RANGE_M.value

    # Already in the strict corridor right now -- a confident, velocity-
    # independent fact; no eligibility needed for this branch.
    if corridor_contains(ref_xy_ego, half_width, max_range):
        return RelevanceResult(Relevance.IN_PATH, provisional=False)

    if not is_velocity_eligible(snapshot, cfg):
        # Can't compute the motion-aware ENTERING test without a trusted
        # velocity -- fall back to Part A's generous position-only gate,
        # and mark the WHOLE verdict provisional either way (B2, B4 test 6:
        # an ineligible track can never be reported as ENTERING).
        fallback_in = candidate_gate(ref_xy_ego, cfg)
        return RelevanceResult(
            Relevance.IN_PATH if fallback_in else Relevance.OUT, provisional=True
        )

    ego_v_global = np.array([ego.vx, ego.vy])
    rel_v_global = snapshot.x[2:4] - ego_v_global  # relative velocity (G8's ego term)
    rel_v_ego = _rotate_2d(rel_v_global, -ego.heading_rad)

    horizon_s = cfg.ENTERING_HORIZON_S.value
    extrapolated_xy_ego = ref_xy_ego + rel_v_ego * horizon_s
    approaching_longitudinally = rel_v_ego[0] < 0  # closing in ego-forward (x) distance

    if corridor_contains(extrapolated_xy_ego, half_width, max_range) and approaching_longitudinally:
        return RelevanceResult(Relevance.ENTERING, provisional=False)

    return RelevanceResult(Relevance.OUT, provisional=False)
