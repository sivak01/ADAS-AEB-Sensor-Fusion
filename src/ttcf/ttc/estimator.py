"""
src/ttcf/ttc/estimator.py — STEP 07 Part B: track-level TTC estimator.

Wires the pure TTC math (Part A, ttc_math.py) to real tracks: eligibility
(reused from step05 Part B's is_velocity_eligible -- one implementation,
never duplicated), and picking the single most urgent path-relevant
object to act on.

No precedent exists anywhere in V1/v2 for "critical object selection"
(picking one track to act on among several) -- checked directly (grep for
critical/driving_track across v2's source turned up only an unrelated
"chi-square critical value" docstring). V1/v2 were MOT-scoped and
evaluated every track against GT; neither ever needed to choose one
object to act on. Built fresh.
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

from ttcf import config
from ttcf.filtering.ego_state import EgoState
from ttcf.tracking.relevance import Relevance, RelevanceResult, is_velocity_eligible
from ttcf.ttc.ttc_math import TTCMathConfig, ttc_from_state
from ttcf.types import TTCReason, TTCResult, TrackSnapshot


class TTCEstimator:
    def __init__(self, cfg=config):
        self._cfg = cfg
        self._ttc_cfg = TTCMathConfig(
            min_trusted_speed_mps=cfg.MIN_TRUSTED_SPEED_MPS.value,
            min_closing_speed_mps=cfg.MIN_CLOSING_SPEED_MPS.value,
        )

    def estimate(self, snapshot: TrackSnapshot, ego: EgoState) -> TTCResult:
        """`ego` must be the ego state AT snapshot.t_us (causal -- G15).
        The caller (typically via EgoStateEstimator.state_at(snapshot.t_us))
        is responsible for that; this method doesn't re-derive it, so the
        causality obligation stays visible at the call site rather than
        hidden inside this class."""
        if not is_velocity_eligible(snapshot, self._cfg):
            # NOT_ELIGIBLE must be visible downstream, never silently
            # treated as "safe" (B3) -- ttc=inf here means "no usable
            # estimate", not "no danger".
            return TTCResult(
                t_us=snapshot.t_us,
                track_id=snapshot.track_id,
                distance_m=float("nan"),
                closing_speed_mps=float("nan"),
                ttc_s=float("inf"),
                reason=TTCReason.NOT_ELIGIBLE,
                closing_std_mps=float("nan"),
            )

        obj_v_cov = snapshot.P[2:4, 2:4]  # velocity sub-block of the KF covariance
        math_result = ttc_from_state(
            obj_ref_xy=snapshot.x[:2],
            obj_v_xy=snapshot.x[2:4],  # velocity from the KF state ONLY (G5)
            ego_xy=np.array([ego.x, ego.y]),
            ego_v_xy=np.array([ego.vx, ego.vy]),
            cfg=self._ttc_cfg,
            obj_v_cov_xy=obj_v_cov,
        )
        return TTCResult(
            t_us=snapshot.t_us,
            track_id=snapshot.track_id,
            distance_m=math_result.distance_m,
            closing_speed_mps=math_result.closing_speed_mps,
            ttc_s=math_result.ttc_s,
            reason=math_result.reason,
            closing_std_mps=math_result.closing_std_mps,
        )

    def conservative_ttc(self, result: TTCResult, k: float = 2.0) -> float:
        """DIAGNOSTIC ONLY (DEC-10, approved 2026-09-23) -- never used to
        drive the action decision in v1 (that stays the point estimate,
        result.ttc_s). distance / (closing + k*closing_std); inf if the
        point estimate itself isn't finite."""
        if not math.isfinite(result.ttc_s):
            return float("inf")
        denom = result.closing_speed_mps + k * result.closing_std_mps
        if not math.isfinite(denom) or denom <= 0:
            return float("inf")
        return result.distance_m / denom


def critical_object(
    results: dict[str, TTCResult], relevance: dict[str, RelevanceResult]
) -> Optional[tuple[str, TTCResult]]:
    """Among tracks `relevance` marks path-relevant (IN_PATH or ENTERING
    -- never gated by class, DEC-2), pick the smallest finite TTC.
    Returns (track_id, result), or None if nothing qualifies."""
    best: Optional[tuple[str, TTCResult]] = None
    for tid, result in results.items():
        rel = relevance.get(tid)
        if rel is None or rel.verdict not in (Relevance.IN_PATH, Relevance.ENTERING):
            continue
        if not math.isfinite(result.ttc_s):
            continue
        if best is None or result.ttc_s < best[1].ttc_s:
            best = (tid, result)
    return best
