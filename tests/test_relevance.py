"""
tests/test_relevance.py — step05_forward_path_filter.md Part B, §B4.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf.filtering.ego_state import EgoState
from ttcf.tracking.relevance import Relevance, path_relevance
from ttcf.types import TrackSnapshot

TIGHT_P = np.diag([0.05, 0.05, 0.05, 0.05])
LOOSE_VEL_P = np.diag([0.05, 0.05, 20.0, 20.0])  # velocity std ~4.47 > MAX_VEL_STD_MPS=3.90


def _snapshot(x, y, vx, vy, n_updates=3, P=TIGHT_P, t_us=0, first_t_us=0):
    return TrackSnapshot(
        track_id="trk_test",
        t_us=t_us,
        x=np.array([x, y, vx, vy]),
        P=P,
        n_updates=n_updates,
        sensors_in_estimate=frozenset({"SENSOR_A"}),
        first_t_us=first_t_us,
    )


def _ego(x=0.0, y=0.0, vx=10.0, vy=0.0, heading=0.0, t_us=0):
    speed = float(np.hypot(vx, vy))
    return EgoState(t_us=t_us, x=x, y=y, vx=vx, vy=vy, speed_mps=speed, heading_rad=heading)


# ── Test 1: object in the corridor, moving with ego -> IN_PATH ───────────

def test_object_in_corridor_moving_with_ego_is_in_path():
    ego = _ego()
    snap = _snapshot(5.0, 0.0, 10.0, 0.0)  # co-traveling, well inside corridor
    result = path_relevance(snap, ego)
    assert result.verdict == Relevance.IN_PATH
    assert result.provisional is False


# ── Test 2: laterally outside, crossing toward corridor fast enough -> ENTERING ─

def test_object_crossing_toward_corridor_is_entering():
    ego = _ego()
    # 3.0m laterally outside (half_width=1.2), slightly slower forward
    # than ego (closing longitudinally) and drifting inward at 2 m/s.
    snap = _snapshot(20.0, 3.0, 8.0, -2.0)
    result = path_relevance(snap, ego)
    assert result.verdict == Relevance.ENTERING
    assert result.provisional is False


# ── Test 3: outside and moving away laterally -> OUT ──────────────────────

def test_object_moving_away_laterally_is_out():
    ego = _ego()
    snap = _snapshot(20.0, 3.0, 10.0, 2.0)  # drifting further from the corridor
    result = path_relevance(snap, ego)
    assert result.verdict == Relevance.OUT


# ── Test 4: stationary roadside object, ego passing -> OUT ───────────────

def test_stationary_roadside_object_is_out():
    ego = _ego()
    snap = _snapshot(20.0, 3.0, 0.0, 0.0)  # parked, stays at the same lateral offset
    result = path_relevance(snap, ego)
    assert result.verdict == Relevance.OUT


# ── Test 5: ego-frame rotation invariance ─────────────────────────────────

def test_rotation_invariance():
    theta = 0.83  # arbitrary rotation, radians
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])

    ego = _ego()
    snap = _snapshot(20.0, 3.0, 8.0, -2.0)  # the ENTERING scenario from test 2
    baseline = path_relevance(snap, ego)

    ego_xy = R @ np.array([ego.x, ego.y])
    ego_v = R @ np.array([ego.vx, ego.vy])
    rotated_ego = _ego(x=ego_xy[0], y=ego_xy[1], vx=ego_v[0], vy=ego_v[1], heading=ego.heading_rad + theta)

    obj_xy = R @ snap.x[:2]
    obj_v = R @ snap.x[2:4]
    rotated_snap = _snapshot(obj_xy[0], obj_xy[1], obj_v[0], obj_v[1])

    rotated = path_relevance(rotated_snap, rotated_ego)
    assert rotated.verdict == baseline.verdict
    assert rotated.provisional == baseline.provisional


# ── Test 6: ineligible-velocity track -> PROVISIONAL, never ENTERING ─────

def test_ineligible_track_is_provisional_never_entering():
    ego = _ego()

    # Too few updates, but within the WIDER candidate gate (half_width +
    # margin = 1.2 + 1.5 = 2.7) -- falls back to IN_PATH, provisional.
    snap_few_updates = _snapshot(20.0, 2.0, 8.0, -5.0, n_updates=1)
    result = path_relevance(snap_few_updates, ego)
    assert result.provisional is True
    assert result.verdict != Relevance.ENTERING
    assert result.verdict == Relevance.IN_PATH  # within the candidate-gate fallback

    # Too few updates AND outside even the wide gate -> OUT, provisional.
    snap_far = _snapshot(20.0, 5.0, 8.0, -5.0, n_updates=1)
    result_far = path_relevance(snap_far, ego)
    assert result_far.provisional is True
    assert result_far.verdict == Relevance.OUT

    # Enough updates but velocity uncertainty too high -> also provisional.
    snap_loose = _snapshot(20.0, 2.0, 8.0, -5.0, n_updates=5, P=LOOSE_VEL_P)
    result_loose = path_relevance(snap_loose, ego)
    assert result_loose.provisional is True
    assert result_loose.verdict != Relevance.ENTERING
