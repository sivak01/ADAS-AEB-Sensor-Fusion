"""
tests/test_extrapolate.py — step09_extrapolation.md §5.

Uses a lightweight fake ego estimator (any object exposing
.state_at(t_us) -> EgoState) rather than a real EgoStateEstimator/
NuScenes -- decouples these tests from the real dataset, consistent with
every other synthetic-scope step so far.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.evaluation.extrapolate import (
    NO_TRACK,
    NOT_ELIGIBLE,
    STALE,
    associate_tracks_to_gt,
    extrapolate_track,
    system_view_at,
)
from ttcf.filtering.ego_state import EgoState
from ttcf.ttc.estimator import TTCEstimator
from ttcf.types import TrackSnapshot

TIGHT_P = np.diag([0.05, 0.05, 0.05, 0.05])
GAP_US = int(config.EVICTION_GAP_S.value * 1e6)  # 300_000


class _FakeEgoEstimator:
    """Constant-velocity fake -- .state_at(t_us) gives genuinely different
    positions at different t_us, so causality tests can tell the right
    ego state from a wrong one."""

    def __init__(self, x0=0.0, y0=0.0, vx=0.0, vy=0.0, heading=0.0, t0_us=0):
        self.x0, self.y0, self.vx, self.vy, self.heading, self.t0_us = x0, y0, vx, vy, heading, t0_us

    def state_at(self, t_us):
        dt = (int(t_us) - self.t0_us) / 1e6
        return EgoState(
            t_us=int(t_us), x=self.x0 + self.vx * dt, y=self.y0 + self.vy * dt,
            vx=self.vx, vy=self.vy, speed_mps=float(np.hypot(self.vx, self.vy)),
            heading_rad=self.heading,
        )


def _snapshot(x, y, vx, vy, t_us, n_updates=3, P=TIGHT_P, first_t_us=None, track_id="trk_A"):
    return TrackSnapshot(
        track_id=track_id, t_us=t_us, x=np.array([x, y, vx, vy]), P=P,
        n_updates=n_updates, sensors_in_estimate=frozenset({"SENSOR_A"}),
        first_t_us=first_t_us if first_t_us is not None else t_us,
    )


# ── Test 1: exact extrapolation matches the analytic value ───────────────

def test_exact_extrapolation_matches_analytic_value():
    history = [_snapshot(10.0, 0.0, 5.0, 0.0, t_us=0, first_t_us=0)]
    ego = _FakeEgoEstimator(x0=0.0, y0=0.0, vx=0.0, vy=0.0)
    est = TTCEstimator()

    t_gt = 200_000  # 0.2s later, within the eviction gap
    result, relevance, reason = extrapolate_track(history, t_gt, ego, est)

    assert reason is None
    expected_x = 10.0 + 5.0 * 0.2  # pos + v*dt
    assert result.distance_m == pytest.approx(expected_x, abs=1e-6)


# ── Test 2: Delta > EVICTION_GAP_S -> STALE ───────────────────────────────

def test_delta_beyond_eviction_gap_is_stale():
    history = [_snapshot(10.0, 0.0, 5.0, 0.0, t_us=0, first_t_us=0)]
    ego = _FakeEgoEstimator()
    est = TTCEstimator()

    t_gt = GAP_US + 100_000  # comfortably past the eviction gap
    result, relevance, reason = extrapolate_track(history, t_gt, ego, est)

    assert reason == STALE
    assert result is None
    assert relevance is None


# ── Test 3: t_gt before first snapshot -> NO_TRACK ────────────────────────

def test_t_gt_before_first_snapshot_is_no_track():
    history = [_snapshot(10.0, 0.0, 5.0, 0.0, t_us=500_000, first_t_us=500_000)]
    ego = _FakeEgoEstimator()
    est = TTCEstimator()

    result, relevance, reason = extrapolate_track(history, 100_000, ego, est)

    assert reason == NO_TRACK
    assert result is None
    assert relevance is None


# ── Test 4: no-lookahead -- identical results if future entries are removed ─

def test_no_lookahead_future_entries_do_not_affect_result():
    full_history = [
        _snapshot(10.0, 0.0, 5.0, 0.0, t_us=0, first_t_us=0),
        _snapshot(10.05, 0.0, 5.0, 0.0, t_us=100_000, first_t_us=0),
        _snapshot(20.0, 0.0, 5.0, 0.0, t_us=200_000, first_t_us=0),  # "future" relative to t_gt below
        _snapshot(25.0, 0.0, 5.0, 0.0, t_us=300_000, first_t_us=0),
    ]
    truncated_history = [s for s in full_history if s.t_us <= 150_000]
    ego = _FakeEgoEstimator()
    est = TTCEstimator()
    t_gt = 150_000

    result_full, rel_full, reason_full = extrapolate_track(full_history, t_gt, ego, est)
    result_trunc, rel_trunc, reason_trunc = extrapolate_track(truncated_history, t_gt, ego, est)

    assert reason_full == reason_trunc
    assert result_full.distance_m == pytest.approx(result_trunc.distance_m)
    assert result_full.ttc_s == pytest.approx(result_trunc.ttc_s)


# ── Test 5: NOT_ELIGIBLE propagates ───────────────────────────────────────

def test_not_eligible_propagates():
    history = [_snapshot(10.0, 0.0, 5.0, 0.0, t_us=0, first_t_us=0, n_updates=1)]  # too few updates
    ego = _FakeEgoEstimator()
    est = TTCEstimator()

    result, relevance, reason = extrapolate_track(history, 100_000, ego, est)

    assert reason == NOT_ELIGIBLE
    assert result is not None  # NOT_ELIGIBLE is visible, not silently dropped
    assert result.reason.name == "NOT_ELIGIBLE"


# ── Test 6: ego state used is at t_gt, not at the snapshot time ──────────

def test_ego_state_used_is_at_t_gt():
    history = [_snapshot(30.0, 0.0, 0.0, 0.0, t_us=0, first_t_us=0)]  # stationary object
    # Ego starts at x=0, moving at 10 m/s -- at t_gt=200_000 (0.2s) it's at x=2.0.
    ego = _FakeEgoEstimator(x0=0.0, vx=10.0, t0_us=0)
    est = TTCEstimator()

    t_gt = 200_000
    result, _, _ = extrapolate_track(history, t_gt, ego, est)

    # If the snapshot-time ego state (x=0) had been used instead of the
    # t_gt ego state (x=2.0), distance would be 30.0, not 28.0.
    assert result.distance_m == pytest.approx(28.0, abs=1e-6)


# ── Test 7: association threshold behaviour on toy cases ─────────────────

def test_association_threshold_toy_cases():
    track_positions = {
        "trk_near": np.array([10.0, 0.0]),   # 0.5m from gt_A -> should match
        "trk_far": np.array([10.0, 5.0]),    # 5m from everything -> ghost
    }
    gt_positions = {
        "gt_A": np.array([10.5, 0.0]),
        "gt_B": np.array([50.0, 50.0]),
    }
    result = associate_tracks_to_gt(track_positions, gt_positions, max_assoc_dist_m=2.0)
    assert result["trk_near"] == "gt_A"
    assert result["trk_far"] is None  # a ghost -- no real object nearby


def test_association_empty_inputs():
    assert associate_tracks_to_gt({}, {"gt_A": np.array([0.0, 0.0])}, 2.0) == {}
    assert associate_tracks_to_gt({"trk_A": np.array([0.0, 0.0])}, {}, 2.0) == {"trk_A": None}


# ── Test 8: proportion check -- mid-eviction/not-yet-started cases ───────

def test_proportion_of_no_track_stale_and_valid_with_short_memory():
    """With only 2-4 updates of memory, a track's alive window is short
    relative to a scene -- mid-eviction and not-yet-started GT queries
    are relatively common, not rare edge cases. This test samples across
    a track's short life and confirms all three outcomes actually occur,
    per step09 §5 item 8's explicit instruction."""
    history = [
        _snapshot(10.0, 0.0, 5.0, 0.0, t_us=1_000_000, first_t_us=1_000_000),
        _snapshot(10.5, 0.0, 5.0, 0.0, t_us=1_100_000, first_t_us=1_000_000),
        _snapshot(11.0, 0.0, 5.0, 0.0, t_us=1_200_000, first_t_us=1_000_000),
    ]
    ego = _FakeEgoEstimator()
    est = TTCEstimator()

    # Sample widely: well before the track starts, through its life, to
    # well after its last update (beyond the eviction gap).
    t_gt_values = list(range(800_000, 1_700_000, 50_000))
    outcomes = {"OK": 0, NO_TRACK: 0, STALE: 0}
    for t_gt in t_gt_values:
        _, _, reason = extrapolate_track(history, t_gt, ego, est)
        outcomes["OK" if reason is None else reason] = outcomes.get("OK" if reason is None else reason, 0) + 1

    print(f"\nProportion check over {len(t_gt_values)} GT-instant queries: {outcomes}")
    assert outcomes[NO_TRACK] > 0, "expected some queries before the track existed"
    assert outcomes[STALE] > 0, "expected some queries after the track's eviction gap"
    assert outcomes["OK"] > 0, "expected some queries genuinely within the track's life"


# ── system_view_at: a small integration check across multiple tracks ─────

def test_system_view_at_picks_critical_and_reports_reason_when_none():
    history_near = [_snapshot(15.0, 0.0, 0.0, 0.0, t_us=0, first_t_us=0, track_id="trk_near")]
    history_far = [_snapshot(50.0, 0.0, 0.0, 0.0, t_us=0, first_t_us=0, track_id="trk_far")]
    ego = _FakeEgoEstimator(vx=10.0)

    view = system_view_at(
        {"trk_near": history_near, "trk_far": history_far}, ego, t_gt_us=100_000
    )
    assert view.critical_track_id == "trk_near"
    assert view.no_estimate_reason is None

    # Nothing exists at all before either track started.
    view_early = system_view_at(
        {"trk_near": history_near, "trk_far": history_far}, ego, t_gt_us=-500_000
    )
    assert view_early.critical_track_id is None
    assert view_early.no_estimate_reason == NO_TRACK
