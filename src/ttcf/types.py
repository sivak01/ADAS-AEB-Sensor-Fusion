"""
src/ttcf/types.py — shared dataclasses passed between every pipeline stage.

Timestamps are integer MICROSECONDS (t_us), matching nuScenes' own
`sample_data.timestamp` / `sample.timestamp` fields exactly (step00 §5).
Convert to float seconds only inside math that needs it — never at a type
boundary, so no step silently loses precision by round-tripping through
floats.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np


def _check_t_us(t_us: object, owner: str) -> None:
    if not isinstance(t_us, (int, np.integer)) or isinstance(t_us, bool):
        raise TypeError(f"{owner}.t_us must be an int (microseconds), got {type(t_us).__name__}")


@dataclass(frozen=True)
class RawEvent:
    """One sensor channel's own record at one instant — never another
    channel's borrowed timestamp/pose/calibration (G2)."""

    t_us: int
    channel: str
    modality: str
    sample_data_token: str
    ego_pose_token: str
    calibrated_sensor_token: str
    filename: str
    is_key_frame: bool
    scene_token: str

    def __post_init__(self) -> None:
        _check_t_us(self.t_us, "RawEvent")


@dataclass(frozen=True, eq=False)
class Detection:
    """A single-object measurement in the global frame (G1), already
    resolved through this event's own transform (G2).

    eq=False: xy_global/R_global are numpy arrays, and the dataclass-
    generated __eq__ would compare them via `==` inside a tuple, which
    raises ("truth value of an array is ambiguous") instead of returning
    bool — identity-based equality is fine here since nothing needs
    value-equality on detections.
    """

    t_us: int
    channel: str
    modality: str
    xy_global: np.ndarray  # shape (2,)
    R_global: np.ndarray  # shape (2, 2)
    ref_point_kind: str  # per DEC-1 — e.g. "nearest_surface_to_ego" or "centroid"
    cls: Optional[str] = None
    n_points: Optional[int] = None
    aux: dict = field(default_factory=dict)  # e.g. radar Doppler raw values — informational only
    # Optional genuine second measurement channel (global frame), e.g.
    # radar's own Doppler velocity under the G17 ablation
    # (RADAR_DOPPLER_ABLATION). None for every adapter/run by default —
    # the tracker does a position-only update unless BOTH fields are set
    # (step06, process_scan). Never populated from aux automatically; an
    # adapter must explicitly opt in.
    velocity_global: Optional[np.ndarray] = None  # shape (2,), [vx, vy]
    R_velocity_global: Optional[np.ndarray] = None  # shape (2, 2)

    def __post_init__(self) -> None:
        _check_t_us(self.t_us, "Detection")


@dataclass(frozen=True, eq=False)
class TrackSnapshot:
    """A track's shared-KF state at one instant. sensors_in_estimate is
    which sensors contributed to THIS estimate, not a lifetime property
    (G11 — no lifetime sensor bookkeeping)."""

    track_id: str
    t_us: int
    x: np.ndarray  # shape (4,) [x, y, vx, vy], global frame
    P: np.ndarray  # shape (4, 4)
    n_updates: int
    sensors_in_estimate: frozenset
    first_t_us: int

    def __post_init__(self) -> None:
        _check_t_us(self.t_us, "TrackSnapshot")
        _check_t_us(self.first_t_us, "TrackSnapshot.first_t_us")


class TTCReason(Enum):
    OK = "OK"
    NOT_ELIGIBLE = "NOT_ELIGIBLE"
    SPEED_DEADBAND_ZEROED = "SPEED_DEADBAND_ZEROED"
    CLOSING_DEADBAND = "CLOSING_DEADBAND"
    OPENING = "OPENING"
    BEHIND = "BEHIND"
    OUT_OF_PATH = "OUT_OF_PATH"


@dataclass(frozen=True)
class TTCResult:
    t_us: int
    track_id: str
    distance_m: float
    closing_speed_mps: float
    ttc_s: float  # may be inf (G8 deadband), never a raw division artifact
    reason: TTCReason
    closing_std_mps: float

    def __post_init__(self) -> None:
        _check_t_us(self.t_us, "TTCResult")


class ActionTier(Enum):
    NONE = "NONE"
    GRADUAL = "GRADUAL"
    AEB = "AEB"


@dataclass(frozen=True)
class ActionDecision:
    t_us: int
    tier: ActionTier
    driving_track_id: Optional[str]
    consecutive_count: int  # debounce count (G16)

    def __post_init__(self) -> None:
        _check_t_us(self.t_us, "ActionDecision")
