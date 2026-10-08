"""
tests/test_ttc_estimator.py — step07_ttc.md Part B, §B4.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from ttcf.filtering.ego_state import EgoState
from ttcf.tracking.relevance import Relevance, RelevanceResult, path_relevance
from ttcf.tracking.tracker import ShortMemoryTracker
from ttcf.ttc.estimator import TTCEstimator, critical_object
from ttcf.types import TTCReason, TrackSnapshot

from synthetic_scenarios import single_object_scans

TIGHT_P = np.diag([0.05, 0.05, 0.05, 0.05])
LOOSE_VEL_P = np.diag([0.05, 0.05, 20.0, 20.0])  # velocity std ~4.47 > MAX_VEL_STD_MPS=3.90


def _snapshot(x, y, vx, vy, n_updates=3, P=TIGHT_P, t_us=0, first_t_us=0, track_id="trk_test"):
    return TrackSnapshot(
        track_id=track_id, t_us=t_us, x=np.array([x, y, vx, vy]), P=P,
        n_updates=n_updates, sensors_in_estimate=frozenset({"SENSOR_A"}), first_t_us=first_t_us,
    )


def _ego(x=0.0, y=0.0, vx=10.0, vy=0.0, heading=0.0, t_us=0):
    return EgoState(t_us=t_us, x=x, y=y, vx=vx, vy=vy, speed_mps=float(np.hypot(vx, vy)), heading_rad=heading)


# ── Test 1: track with 1 update -> NOT_ELIGIBLE ───────────────────────────

def test_one_update_is_not_eligible():
    est = TTCEstimator()
    snap = _snapshot(30.0, 0.0, 0.0, 0.0, n_updates=1)
    result = est.estimate(snap, _ego())
    assert result.reason == TTCReason.NOT_ELIGIBLE
    assert result.ttc_s == float("inf")


# ── Test 2: wide velocity covariance -> NOT_ELIGIBLE even with many updates ─

def test_wide_velocity_covariance_is_not_eligible():
    est = TTCEstimator()
    snap = _snapshot(30.0, 0.0, 0.0, 0.0, n_updates=20, P=LOOSE_VEL_P)
    result = est.estimate(snap, _ego())
    assert result.reason == TTCReason.NOT_ELIGIBLE
    assert result.ttc_s == float("inf")


# ── Test 3: critical-object selection ─────────────────────────────────────

def test_critical_object_picks_lowest_ttc_in_path_only():
    est = TTCEstimator()
    ego = _ego()

    # In-path, far -> big TTC.
    snap_far = _snapshot(50.0, 0.0, 0.0, 0.0, track_id="trk_far")
    # In-path, near -> small TTC (the critical one).
    snap_near = _snapshot(15.0, 0.0, 0.0, 0.0, track_id="trk_near")
    # Out-of-path (far to the side), very close -- must be ignored despite tiny distance.
    snap_out = _snapshot(5.0, 20.0, 0.0, 0.0, track_id="trk_out")

    results = {}
    relevance = {}
    for snap in (snap_far, snap_near, snap_out):
        result = est.estimate(snap, ego)
        results[snap.track_id] = result
        relevance[snap.track_id] = path_relevance(snap, ego)

    # Sanity: trk_out really is judged OUT (not accidentally in-path).
    assert relevance["trk_out"].verdict == Relevance.OUT

    winner = critical_object(results, relevance)
    assert winner is not None
    assert winner[0] == "trk_near"


def test_critical_object_returns_none_when_nothing_qualifies():
    ego = _ego()
    snap_out = _snapshot(5.0, 20.0, 0.0, 0.0, track_id="trk_out")
    est = TTCEstimator()
    result = est.estimate(snap_out, ego)
    relevance = {"trk_out": path_relevance(snap_out, ego)}
    assert critical_object({"trk_out": result}, relevance) is None


# ── Test 4: ego velocity used is the value AT snapshot time, not the latest ─

def test_ego_state_used_is_causal_not_latest():
    est = TTCEstimator()
    snap = _snapshot(30.0, 0.0, 0.0, 0.0, t_us=100_000)

    ego_at_snapshot_time = _ego(x=0.0, vx=5.0, t_us=100_000)
    ego_much_later = _ego(x=20.0, vx=20.0, t_us=900_000)  # a LATER, faster, closer ego

    result_correct = est.estimate(snap, ego_at_snapshot_time)
    result_wrong = est.estimate(snap, ego_much_later)

    # Using the wrong (later/faster/closer) ego state gives a materially
    # different answer -- proving WHICH ego state is passed in actually
    # matters, not just that some ego state was supplied.
    assert result_correct.distance_m == pytest.approx(30.0, abs=0.5)
    assert result_wrong.distance_m == pytest.approx(10.0, abs=0.5)
    assert result_correct.closing_speed_mps != pytest.approx(result_wrong.closing_speed_mps, abs=0.1)


# ── Test 5: synthetic closing scenario end-to-end (tracker -> estimator) ──

def test_end_to_end_tracker_to_estimator_matches_analytic_ttc():
    # A stationary object at (30, 0), tracked via noisy alternating-sensor
    # detections (same generator step06 uses).
    scans = single_object_scans(30.0, 0.0, 0.0, 0.0, 0, 100_000, 15, rng=np.random.default_rng(5))
    tracker = ShortMemoryTracker()
    snaps = []
    for t_us, dets in scans:
        snaps = tracker.process_scan(dets, t_us)
    assert len(snaps) == 1
    final_snapshot = snaps[0]

    ego = _ego(x=0.0, y=0.0, vx=10.0, vy=0.0, t_us=final_snapshot.t_us)
    est = TTCEstimator()
    result = est.estimate(final_snapshot, ego)

    assert result.reason.name in ("OK", "SPEED_DEADBAND_ZEROED")
    analytic_ttc = 30.0 / 10.0  # distance / ego closing speed, object truly stationary
    assert result.ttc_s == pytest.approx(analytic_ttc, rel=0.15)
