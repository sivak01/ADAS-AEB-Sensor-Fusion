"""
src/ttcf/ttc/ttc_math.py — STEP 07 Part A: the one TTC formula (G8, G9).

Pure function, no I/O, no state -- used identically by the tracker path
(Part B, later) and the ground-truth builder (stepD0), so predicted and
GT TTC are computed exactly the same way (G9). Every subtle mistake in
this project's history lives here (step07 §1), so this module stays
small and heavily tested.

SIGN-CONVENTION NOTE (found before implementing, not a design choice):
step07_ttc.md's own prose, read completely literally -- dx,dy =
obj_ref_xy - ego_xy (A2 step 1), then closing = (v_obj - ego_v) . (dx,dy)
/ distance (A2 step 3, using the OBJECT's velocity for vx,vy) -- computes
the NEGATIVE of the closing speed its own test A4.1 requires (stationary
object 30m ahead, ego closing at 10 m/s -> the literal formula gives
closing=-10, not the specified +10; verified numerically before writing
this file). The convention used here reproduces +10, matches
v2/ttc.py's already-validated formula (dx=ego-obj there; mathematically
equivalent to what's below), and matches the physical definition
(closing speed positive when distance is shrinking): the position vector
still points from ego to the object (as A2 step 1 literally says), but
the relative velocity is EGO's velocity minus the object's, not the
object's minus ego's.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ttcf.types import TTCReason


@dataclass(frozen=True)
class TTCMathConfig:
    """The only two config values this pure function needs -- passed in
    explicitly, never read from the config module implicitly, so tests
    can vary them freely (step07 A3)."""

    min_trusted_speed_mps: float
    min_closing_speed_mps: float


@dataclass(frozen=True)
class TTCMathResult:
    distance_m: float
    closing_speed_mps: float
    ttc_s: float
    reason: TTCReason
    closing_std_mps: float  # NaN if no velocity covariance was supplied


def ttc_from_state(
    obj_ref_xy,
    obj_v_xy,
    ego_xy,
    ego_v_xy,
    cfg: TTCMathConfig,
    obj_v_cov_xy: Optional[np.ndarray] = None,
) -> TTCMathResult:
    """distance / closing-speed / TTC from one snapshot of relative
    motion. Global frame, metres and m/s throughout. See the module
    docstring for the sign-convention note. obj_v_cov_xy, if given, is
    the object's 2x2 velocity covariance (from the track's KF state) --
    used only for the closing_std diagnostic (DEC-10), never to change
    distance/closing/ttc themselves."""
    obj_ref_xy = np.asarray(obj_ref_xy, dtype=float)
    ego_xy = np.asarray(ego_xy, dtype=float)
    obj_v_xy = np.asarray(obj_v_xy, dtype=float)
    ego_v_xy = np.asarray(ego_v_xy, dtype=float)

    # A2 step 1: position vector from ego to the object.
    dx, dy = obj_ref_xy[0] - ego_xy[0], obj_ref_xy[1] - ego_xy[1]
    distance = math.hypot(dx, dy)

    reason = TTCReason.OK

    # A2 step 2: trusted-speed deadband -- jitter on an object that isn't
    # really moving must not be read as real motion. Note this triggers
    # even for an object whose raw speed is EXACTLY 0 (0 < threshold is
    # still true) -- deliberate, so a literally-stationary object and a
    # "speed below trust" object are handled identically (A4 test 5).
    obj_speed = math.hypot(obj_v_xy[0], obj_v_xy[1])
    v_obj_x, v_obj_y = float(obj_v_xy[0]), float(obj_v_xy[1])
    if obj_speed < cfg.min_trusted_speed_mps:
        v_obj_x, v_obj_y = 0.0, 0.0
        reason = TTCReason.SPEED_DEADBAND_ZEROED

    if distance == 0.0:
        # Coincident points: no meaningful radial direction to measure a
        # closing speed along. Not explicitly spec'd, but must not crash
        # (the "sensible result, not a crash" spirit of A4 test 9).
        return TTCMathResult(0.0, 0.0, float("inf"), reason, float("nan"))

    # A2 step 3, sign-corrected per the module docstring: relative
    # velocity is EGO's velocity minus the (possibly zeroed) object's,
    # dotted with the ego-to-object position vector -- positive when the
    # gap is shrinking.
    rel_vx, rel_vy = ego_v_xy[0] - v_obj_x, ego_v_xy[1] - v_obj_y
    closing = (rel_vx * dx + rel_vy * dy) / distance

    # A2 step 4: closing deadband -- not just <= 0. OPENING is the more
    # specific case of "closing <= MIN_CLOSING_SPEED_MPS" where the gap
    # is actively growing; CLOSING_DEADBAND covers the rest (including
    # exactly 0).
    if closing < 0:
        reason = TTCReason.OPENING
        ttc = float("inf")
    elif closing <= cfg.min_closing_speed_mps:
        reason = TTCReason.CLOSING_DEADBAND
        ttc = float("inf")
    else:
        ttc = distance / closing

    # A2 step 6: optional closing_std diagnostic from a velocity
    # covariance block (DEC-10's uncertainty diagnostic, used later by
    # Part B / step08 -- not decided or acted on here).
    if obj_v_cov_xy is not None:
        u = np.array([dx, dy]) / distance
        closing_std = float(math.sqrt(max(0.0, u @ np.asarray(obj_v_cov_xy, dtype=float) @ u)))
    else:
        closing_std = float("nan")

    return TTCMathResult(distance, closing, ttc, reason, closing_std)
