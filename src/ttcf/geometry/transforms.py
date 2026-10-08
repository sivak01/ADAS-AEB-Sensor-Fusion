"""
src/ttcf/geometry/transforms.py — sensor -> ego -> global transforms (G1, G2).

Every sensor reports in its own local frame. Comparing anything across time or
across sensors requires everything in one global (world) frame first — get
this wrong and a parked car appears to move at the ego vehicle's own speed.
This is the single most load-bearing utility in the project (step02).

Built directly against `nuscenes.utils.geometry_utils.transform_matrix` and
`pyquaternion.Quaternion` — never hand-rolled (step02_transform.md's explicit
rule). Neither V1 nor v2 of the predecessor project followed this rule (both
hand-rolled their own transform_matrix in src/geometry.py and v2 imported it
unchanged), so there is no tested precedent to carry over for this piece —
built fresh against the devkit itself, whose signature step00's preflight
already captured:
    transform_matrix(translation: ndarray, rotation: Quaternion,
                      inverse: bool = False) -> ndarray (4x4)
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.geometry_utils import transform_matrix
from pyquaternion import Quaternion

from ttcf.types import RawEvent


def pose_matrix(translation, rotation) -> np.ndarray:
    """4x4 homogeneous transform from a translation (3,) and a nuScenes
    [w, x, y, z] quaternion — via the devkit's own transform_matrix, never
    hand-rolled quaternion math."""
    return transform_matrix(np.asarray(translation, dtype=float), Quaternion(rotation))


# FrameContext.from_event() cache, keyed by (which NuScenes instance, the
# event's own calibrated_sensor_token, the event's own ego_pose_token) — the
# id(nusc) component matters only if more than one NuScenes instance is ever
# loaded in one process (e.g. mini + trainval); tokens are not guaranteed
# unique across different loaded databases.
_frame_context_cache: dict[tuple[int, str, str], "FrameContext"] = {}


@dataclass(frozen=True)
class FrameContext:
    """Holds ONE event's own calibrated_sensor and ego_pose records — the
    only way to get a transform for that event (G2). No function anywhere
    accepts a bare token, a "sample", or a channel= argument that could
    quietly pick a different channel's pose."""

    calibrated_sensor: dict
    ego_pose: dict

    @staticmethod
    def from_event(nusc: NuScenes, event: RawEvent) -> "FrameContext":
        key = (id(nusc), event.calibrated_sensor_token, event.ego_pose_token)
        cached = _frame_context_cache.get(key)
        if cached is not None:
            return cached
        ctx = FrameContext(
            calibrated_sensor=nusc.get("calibrated_sensor", event.calibrated_sensor_token),
            ego_pose=nusc.get("ego_pose", event.ego_pose_token),
        )
        _frame_context_cache[key] = ctx
        return ctx


def _apply(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to (N,2) or (N,3) points, batched
    (one matrix multiply for all N points, not a rebuild-per-point loop —
    same technique as the predecessor project's points_to_global)."""
    points = np.asarray(points, dtype=float)
    was_2d = points.shape[1] == 2
    if was_2d:
        points = np.hstack([points, np.zeros((points.shape[0], 1))])
    n = points.shape[0]
    points_h = np.vstack([points.T, np.ones((1, n))])  # (4, N)
    out_h = T @ points_h
    out = out_h[:3].T  # (N, 3)
    return out[:, :2] if was_2d else out


def _apply_vector(vectors: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Like _apply(), but for FREE VECTORS (e.g. a velocity) -- homogeneous
    w=0 instead of w=1, so translation never contributes (a direction has
    no position to translate). Reuses the exact same transform chain as
    point transforms (one rotation implementation, not a second hand-
    rolled one) -- the predecessor project's own vector_to_global()
    independently arrived at this identical w=0 trick."""
    vectors = np.asarray(vectors, dtype=float)
    was_2d = vectors.shape[1] == 2
    if was_2d:
        vectors = np.hstack([vectors, np.zeros((vectors.shape[0], 1))])
    n = vectors.shape[0]
    vectors_h = np.vstack([vectors.T, np.zeros((1, n))])  # (4, N), w=0 not 1
    out_h = T @ vectors_h
    out = out_h[:3].T
    return out[:, :2] if was_2d else out


def sensor_to_ego(points, ctx: FrameContext) -> np.ndarray:
    T = pose_matrix(ctx.calibrated_sensor["translation"], ctx.calibrated_sensor["rotation"])
    return _apply(points, T)


def sensor_to_ego_vector(vectors, ctx: FrameContext) -> np.ndarray:
    T = pose_matrix(ctx.calibrated_sensor["translation"], ctx.calibrated_sensor["rotation"])
    return _apply_vector(vectors, T)


def ego_to_global_vector(vectors, ctx: FrameContext) -> np.ndarray:
    T = pose_matrix(ctx.ego_pose["translation"], ctx.ego_pose["rotation"])
    return _apply_vector(vectors, T)


def sensor_to_global_vector(vectors, ctx: FrameContext) -> np.ndarray:
    """Rotate a free vector (e.g. radar's own Doppler velocity, native to
    the sensor's own frame) all the way to the global frame -- same
    sensor->ego->global chain as a point, minus any translation."""
    return ego_to_global_vector(sensor_to_ego_vector(vectors, ctx), ctx)


def ego_to_sensor(points, ctx: FrameContext) -> np.ndarray:
    T = transform_matrix(
        np.asarray(ctx.calibrated_sensor["translation"], dtype=float),
        Quaternion(ctx.calibrated_sensor["rotation"]),
        inverse=True,
    )
    return _apply(points, T)


def ego_to_global(points, ctx: FrameContext) -> np.ndarray:
    T = pose_matrix(ctx.ego_pose["translation"], ctx.ego_pose["rotation"])
    return _apply(points, T)


def global_to_ego(points, ctx: FrameContext) -> np.ndarray:
    T = transform_matrix(
        np.asarray(ctx.ego_pose["translation"], dtype=float),
        Quaternion(ctx.ego_pose["rotation"]),
        inverse=True,
    )
    return _apply(points, T)


def global_to_ego_vector(vectors, ctx: FrameContext) -> np.ndarray:
    """Inverse of ego_to_global_vector -- e.g. a track's own global-frame
    KF velocity, rotated into ego frame for BEV drawing (step12). Added
    alongside ego_to_global_vector/sensor_to_global_vector (G17's own
    vector-rotation helpers) rather than ported from anywhere -- the
    predecessor project never needed a global->ego vector direction."""
    T = transform_matrix(
        np.asarray(ctx.ego_pose["translation"], dtype=float),
        Quaternion(ctx.ego_pose["rotation"]),
        inverse=True,
    )
    return _apply_vector(vectors, T)


def sensor_to_global(points, ctx: FrameContext) -> np.ndarray:
    return ego_to_global(sensor_to_ego(points, ctx), ctx)


def global_to_sensor(points, ctx: FrameContext) -> np.ndarray:
    return ego_to_sensor(global_to_ego(points, ctx), ctx)


def ego_yaw(ctx: FrameContext) -> float:
    """Ego heading (rad), extracted via pyquaternion's own yaw_pitch_roll —
    the tested idiom found in the predecessor project's own geometry test,
    not hand-rolled trig on the quaternion components."""
    return float(Quaternion(ctx.ego_pose["rotation"]).yaw_pitch_roll[0])


def rotate_cov_ego_to_global(R_ego: np.ndarray, yaw: float) -> np.ndarray:
    """Rotate a 2x2 ego-frame covariance (sigma_long, sigma_lat convention)
    into the global frame: R_g = Rz . R_e . Rz^T. Used by adapters so a
    per-sensor noise spec given in the ego frame becomes a global-frame
    covariance the shared KF can use directly."""
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s], [s, c]])
    return Rz @ np.asarray(R_ego, dtype=float) @ Rz.T


def ego_pose_timestamp(ctx: FrameContext) -> int:
    """The ego_pose record's OWN timestamp (int microseconds) — never
    another channel's (G2)."""
    return int(ctx.ego_pose["timestamp"])
