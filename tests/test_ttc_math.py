"""
tests/test_ttc_math.py — step07_ttc.md Part A, §A4, items 1-9.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf.ttc.ttc_math import TTCMathConfig, ttc_from_state
from ttcf.types import TTCReason

CFG = TTCMathConfig(min_trusted_speed_mps=0.5, min_closing_speed_mps=0.5)


# ── Test 1: stationary object 30 m ahead, ego 10 m/s toward it ───────────

def test_stationary_object_ego_closing():
    result = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    assert result.distance_m == pytest.approx(30.0)
    assert result.closing_speed_mps == pytest.approx(10.0)
    assert result.ttc_s == pytest.approx(3.0)
    assert result.reason == TTCReason.SPEED_DEADBAND_ZEROED  # |v_obj|=0 < trusted


# ── Test 2: two vehicles co-travelling -> TTC = inf ───────────────────────

def test_co_travelling_vehicles_ttc_infinite():
    result = ttc_from_state(
        obj_ref_xy=(20.0, 0.0), obj_v_xy=(15.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(15.0, 0.0),
        cfg=CFG,
    )
    assert math.isinf(result.ttc_s)
    assert result.closing_speed_mps == pytest.approx(0.0, abs=1e-9)


# ── Test 3: near-tangential motion, closing just below MIN_CLOSING_SPEED ─

def test_near_tangential_motion_hits_closing_deadband():
    # Object directly ahead, moving almost perpendicular to the line of
    # sight, with a tiny closing component just under the deadband.
    # Choose ego velocity so that the radial (x) component of relative
    # velocity is slightly below MIN_CLOSING_SPEED_MPS=0.5.
    result = ttc_from_state(
        obj_ref_xy=(50.0, 0.0), obj_v_xy=(0.0, 8.0),  # fast tangential motion
        ego_xy=(0.0, 0.0), ego_v_xy=(0.3, 0.0),  # slow closing component
        cfg=CFG,
    )
    assert result.closing_speed_mps < CFG.min_closing_speed_mps
    assert result.closing_speed_mps >= 0
    assert math.isinf(result.ttc_s)
    assert result.reason == TTCReason.CLOSING_DEADBAND


# ── Test 4: opening gap -> inf, reason OPENING ────────────────────────────

def test_opening_gap():
    result = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(15.0, 0.0),  # object speeding away
        ego_xy=(0.0, 0.0), ego_v_xy=(0.0, 0.0),
        cfg=CFG,
    )
    assert result.closing_speed_mps < 0
    assert math.isinf(result.ttc_s)
    assert result.reason == TTCReason.OPENING


# ── Test 5: object speed below trusted -> treated as 0, equals stationary case ─

def test_speed_below_trusted_equals_stationary_case():
    stationary = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    jittery = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(0.2, 0.0),  # below MIN_TRUSTED_SPEED_MPS=0.5
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    # Compare field-by-field: both closing_std_mps are NaN (no covariance
    # supplied), and NaN != NaN by IEEE754, so a bare dataclass `==` would
    # spuriously fail even though every field matches.
    assert jittery.distance_m == pytest.approx(stationary.distance_m)
    assert jittery.closing_speed_mps == pytest.approx(stationary.closing_speed_mps)
    assert jittery.ttc_s == pytest.approx(stationary.ttc_s)
    assert jittery.reason == stationary.reason
    assert math.isnan(jittery.closing_std_mps) and math.isnan(stationary.closing_std_mps)


# ── Test 6: independence of deadbands ─────────────────────────────────────

def test_deadbands_are_independent():
    # High raw speed (passes trusted-speed check) but purely tangential
    # (hits the closing deadband) -- proves the two checks are separate.
    result = ttc_from_state(
        obj_ref_xy=(20.0, 0.0), obj_v_xy=(0.0, 12.0),  # 12 m/s, well above trusted
        ego_xy=(0.0, 0.0), ego_v_xy=(0.0, 0.0),
        cfg=CFG,
    )
    assert result.reason != TTCReason.SPEED_DEADBAND_ZEROED  # trusted-speed check passed
    assert result.reason == TTCReason.CLOSING_DEADBAND  # but closing check failed
    assert math.isinf(result.ttc_s)


# ── Test 7: frame invariance -- rotate/translate the whole scene ─────────

def test_frame_invariance_under_rotation_and_translation():
    base = ttc_from_state(
        obj_ref_xy=(30.0, 5.0), obj_v_xy=(1.0, -2.0),
        ego_xy=(2.0, -3.0), ego_v_xy=(9.0, 1.0),
        cfg=CFG,
    )

    theta = 0.73  # arbitrary rotation, radians
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([123.4, -56.7])  # arbitrary translation

    def rot_translate_pos(p):
        return R @ np.array(p) + t

    def rot_velocity(v):
        return R @ np.array(v)  # velocities rotate but do NOT translate

    transformed = ttc_from_state(
        obj_ref_xy=rot_translate_pos((30.0, 5.0)),
        obj_v_xy=rot_velocity((1.0, -2.0)),
        ego_xy=rot_translate_pos((2.0, -3.0)),
        ego_v_xy=rot_velocity((9.0, 1.0)),
        cfg=CFG,
    )

    assert transformed.distance_m == pytest.approx(base.distance_m)
    assert transformed.closing_speed_mps == pytest.approx(base.closing_speed_mps)
    assert transformed.ttc_s == pytest.approx(base.ttc_s)
    assert transformed.reason == base.reason


# ── Test 8: scaling -- doubling distance at fixed closing doubles TTC ────

def test_scaling_doubles_ttc_with_distance():
    near = ttc_from_state(
        obj_ref_xy=(20.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    far = ttc_from_state(
        obj_ref_xy=(40.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    assert far.closing_speed_mps == pytest.approx(near.closing_speed_mps)
    assert far.ttc_s == pytest.approx(2 * near.ttc_s)


# ── Test 9: object behind ego -- sensible result, not a crash ────────────

def test_object_behind_ego_does_not_crash():
    result = ttc_from_state(
        obj_ref_xy=(-15.0, 0.0), obj_v_xy=(0.0, 0.0),  # behind ego
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),  # ego moving away from it (+x)
        cfg=CFG,
    )
    assert math.isfinite(result.distance_m)
    assert not math.isnan(result.closing_speed_mps)
    assert result.ttc_s > 0  # inf or a positive finite number, never negative/NaN
    assert isinstance(result.reason, TTCReason)


# ── Extra: closing_std diagnostic (A2 step 6) ─────────────────────────────

def test_closing_std_diagnostic():
    no_cov = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
    )
    assert math.isnan(no_cov.closing_std_mps)

    P_vv = np.diag([4.0, 1.0])  # velocity covariance: sigma_x=2, sigma_y=1
    with_cov = ttc_from_state(
        obj_ref_xy=(30.0, 0.0), obj_v_xy=(0.0, 0.0),
        ego_xy=(0.0, 0.0), ego_v_xy=(10.0, 0.0),
        cfg=CFG,
        obj_v_cov_xy=P_vv,
    )
    # u = (1, 0) here (object due east of ego) -> closing_std = sqrt(u^T P u) = sqrt(4) = 2
    assert with_cov.closing_std_mps == pytest.approx(2.0)
