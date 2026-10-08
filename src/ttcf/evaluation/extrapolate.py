"""
src/ttcf/evaluation/extrapolate.py — STEP 09: bounded, causal extrapolation
to ground-truth timestamps (G15).

Ground truth exists only at 2Hz keyframe instants; the pipeline updates at
native rate. To compare fairly: "what did the SYSTEM believe at exactly
the GT instant?" -- using only information available up to that instant,
extrapolated a bounded distance, never backward before a track existed.

Ported from the predecessor project's v2/evaluate.py: BOTH bounding
guards this step's spec calls out as regression targets already exist,
fixed, in v2 -- an annotation before a track's first snapshot is a clean
no-match (never backward extrapolation), and one past
MAX_MISSED_SECONDS (here: EVICTION_GAP_S) is treated as no-match too,
since the track would already be evicted by then. Both guards are
carried over with the same reasoning, using this project's own (much
shorter) EVICTION_GAP_S rather than v2's constant.

NOT ported: v2's position_at() is plain arithmetic (pos + v*dt) with NO
covariance growth, deliberately -- its own docstring notes this is
"exactly what re-predicting a fresh KF... would produce for the mean
state," so it skips the KF entirely. This project's eligibility check
(is_velocity_eligible, step05 Part B) depends on P, and a track that's
been extrapolated a long time without a real update SHOULD become
progressively less trusted -- v2 never needed this since it only ever
compared position for MAE/RMSE, never gated eligibility on P. This
module reconstructs a throwaway KF from the snapshot and calls its own,
already-tested state_at() instead of reimplementing the arithmetic by
hand -- one implementation of "predict forward," not two.

Also NOT ported: v2's association is a LOCKED identity (matches once,
reused for a track's whole life) -- explicitly a MOT-style concept. This
step's §3.3 wants a FRESH per-instant Hungarian match every time,
consistent with this project's no-persistent-identity design (G11).
Built fresh.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

from ttcf import config
from ttcf.filtering.ego_state import EgoStateEstimator
from ttcf.filtering.kalman import ConstantVelocityKF
from ttcf.ttc.action import ActionRecord, ActionTierDebouncer
from ttcf.ttc.estimator import TTCEstimator, critical_object
from ttcf.tracking.relevance import RelevanceResult, path_relevance
from ttcf.types import TTCReason, TTCResult, TrackSnapshot

# Reasons a per-track extrapolation has nothing usable, per step09 §3.2.
NO_TRACK = "NO_TRACK"
STALE = "STALE"
NOT_ELIGIBLE = "NOT_ELIGIBLE"
NONE_IN_PATH = "NONE_IN_PATH"


@dataclass(frozen=True)
class TrackExtrapolation:
    track_id: str
    ttc_result: Optional[TTCResult]
    relevance: Optional[RelevanceResult]
    no_estimate_reason: Optional[str]  # None if usable


@dataclass(frozen=True)
class SystemView:
    t_gt_us: int
    per_track: dict  # track_id -> TrackExtrapolation
    critical_track_id: Optional[str]
    critical_result: Optional[TTCResult]
    action: Optional[ActionRecord]
    no_estimate_reason: Optional[str]  # set only when critical_track_id is None


def _extrapolate_snapshot(snapshot: TrackSnapshot, t_gt_us: int, sigma_a: float) -> TrackSnapshot:
    """A throwaway KF seeded from the snapshot, predicted forward to
    t_gt_us via its own (already-tested) state_at() -- reuses the exact
    same constant-velocity math (x_new = F @ x) the live tracker uses,
    rather than a parallel 'pos + v*dt' implementation, and grows P
    correctly along the way (unlike v2's plain-arithmetic position_at())."""
    kf = ConstantVelocityKF(sigma_a=sigma_a)
    kf.x = snapshot.x.copy()
    kf.P = snapshot.P.copy()
    kf._t_us = int(snapshot.t_us)
    predicted = kf.state_at(t_gt_us)
    return TrackSnapshot(
        track_id=snapshot.track_id,
        t_us=int(t_gt_us),
        x=predicted.x,
        P=predicted.P,
        n_updates=snapshot.n_updates,
        sensors_in_estimate=snapshot.sensors_in_estimate,
        first_t_us=snapshot.first_t_us,
    )


def extrapolate_track(
    snapshot_history: list,
    t_gt_us: int,
    ego_estimator: EgoStateEstimator,
    estimator: TTCEstimator,
    cfg=config,
):
    """Returns (ttc_result, relevance, no_estimate_reason) for ONE track's
    causal history at t_gt_us. Never looks at snapshots with t_us > t_gt_us
    (G15) -- filtering happens here, not by trusting the caller to have
    already truncated the history."""
    t_gt_us = int(t_gt_us)
    causal = [s for s in snapshot_history if s.t_us <= t_gt_us]
    if not causal:
        return None, None, NO_TRACK

    snapshot = max(causal, key=lambda s: s.t_us)
    delta_s = (t_gt_us - snapshot.t_us) / 1e6
    if delta_s > cfg.EVICTION_GAP_S.value:
        return None, None, STALE

    extrapolated = _extrapolate_snapshot(snapshot, t_gt_us, cfg.SIGMA_A.value)
    ego_at_t_gt = ego_estimator.state_at(t_gt_us)  # causal -- G15, step09 §3.2

    ttc_result = estimator.estimate(extrapolated, ego_at_t_gt)
    relevance = path_relevance(extrapolated, ego_at_t_gt, cfg)
    reason = NOT_ELIGIBLE if ttc_result.reason == TTCReason.NOT_ELIGIBLE else None
    return ttc_result, relevance, reason


def _derive_top_level_reason(per_track: dict) -> str:
    """Priority order for the SystemView-level no_estimate_reason when no
    critical object was found -- not fully pinned down by the spec text
    (§3.2 lists the four possible values but not a precedence among
    them), so documented explicitly here rather than left implicit."""
    if not per_track:
        return NO_TRACK
    reasons = {te.no_estimate_reason for te in per_track.values()}
    if reasons <= {NO_TRACK}:
        return NO_TRACK
    if reasons <= {NO_TRACK, STALE}:
        return STALE
    if reasons <= {NO_TRACK, STALE, NOT_ELIGIBLE}:
        return NOT_ELIGIBLE
    return NONE_IN_PATH


def system_view_at(
    snapshot_histories: dict,
    ego_estimator: EgoStateEstimator,
    t_gt_us: int,
    action_debouncer: Optional[ActionTierDebouncer] = None,
    estimator: Optional[TTCEstimator] = None,
    cfg=config,
) -> SystemView:
    """snapshot_histories: track_id -> list[TrackSnapshot] (each track's
    own causal history; need not be pre-truncated to t_gt_us -- see
    extrapolate_track). Returns what the system believed at exactly
    t_gt_us: per-track extrapolated state, the single critical object (if
    any), and the system's action at that instant (step08's action_at)."""
    estimator = estimator or TTCEstimator(cfg)
    per_track = {}
    for track_id, history in snapshot_histories.items():
        ttc_result, relevance, reason = extrapolate_track(history, t_gt_us, ego_estimator, estimator, cfg)
        per_track[track_id] = TrackExtrapolation(track_id, ttc_result, relevance, reason)

    results = {tid: te.ttc_result for tid, te in per_track.items() if te.ttc_result is not None}
    relevances = {tid: te.relevance for tid, te in per_track.items() if te.relevance is not None}
    winner = critical_object(results, relevances)

    if winner is not None:
        critical_track_id, critical_result = winner
        top_reason = None
    else:
        critical_track_id, critical_result = None, None
        top_reason = _derive_top_level_reason(per_track)

    action = action_debouncer.action_at(t_gt_us) if action_debouncer is not None else None

    return SystemView(
        t_gt_us=int(t_gt_us),
        per_track=per_track,
        critical_track_id=critical_track_id,
        critical_result=critical_result,
        action=action,
        no_estimate_reason=top_reason,
    )


def associate_tracks_to_gt(
    track_positions: dict, gt_positions: dict, max_assoc_dist_m: float
) -> dict:
    """STEP 09 §3.3: diagnostic association only, for false-brake
    attribution -- NOT used by the primary metric (step10). Fresh
    Hungarian assignment at this one instant (track_id -> position),
    (gt_instance_token -> position), gated at max_assoc_dist_m. Returns
    track_id -> matched gt_instance_token, or None (a "ghost" -- a track
    with no real object nearby)."""
    track_ids = list(track_positions.keys())
    gt_ids = list(gt_positions.keys())
    result = {tid: None for tid in track_ids}
    if not track_ids or not gt_ids:
        return result

    infeasible = 1e6
    cost = np.full((len(track_ids), len(gt_ids)), infeasible)
    for i, tid in enumerate(track_ids):
        for j, gid in enumerate(gt_ids):
            d = float(np.linalg.norm(np.asarray(track_positions[tid]) - np.asarray(gt_positions[gid])))
            if d <= max_assoc_dist_m:
                cost[i, j] = d

    row_idx, col_idx = linear_sum_assignment(cost)
    for r, c in zip(row_idx, col_idx):
        if cost[r, c] < infeasible:
            result[track_ids[r]] = gt_ids[c]
    return result
