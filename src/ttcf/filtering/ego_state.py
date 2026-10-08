"""
src/ttcf/filtering/ego_state.py — the ego vehicle's own velocity, derived (G5, G15).

nuScenes' ego_pose has no velocity field, so ego velocity must be derived
with the same discipline as any tracked object's -- run through the SAME
shared KF (step03's purpose), never a raw two-point finite difference
(what-not-to-do.md #3).

Ported from the predecessor project's v2/adapters.py ego_pose_lookup(): feed
ego_pose.translation through a ConstantVelocityKF with R = EGO_POSE_STD_M^2 . I,
one estimator per scene, reset on every scene boundary (a scene is a distinct
driving-log segment -- unrelated to G11's "no scene-boundary reset for
TRACKS"; this is specifically about not carrying ego state across an
unrelated log). NOT ported: v2 only ever looked up ego state AT an existing
2Hz sample_id, with no extrapolation, because it stayed at keyframe rate
throughout. This project reads native-rate sweeps/ (G3), so state_at(t_us)
here supports causal prediction to ANY timestamp, not just keyframe instants
-- genuinely new. Also new: v2 used ONE shared sigma_a for ego and tracked
objects; this project's config.py deliberately split SIGMA_A_EGO from SIGMA_A
at step00 (DEC-9) -- consumed here, not re-decided.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass

import numpy as np
from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion

from ttcf import config
from ttcf.filtering.kalman import ConstantVelocityKF


@dataclass(frozen=True)
class EgoState:
    t_us: int
    x: float
    y: float
    vx: float
    vy: float
    speed_mps: float
    heading_rad: float


class EgoStateEstimator:
    """One estimator per scene -- never carry state across scenes."""

    def __init__(self, nusc: NuScenes, scene_token: str, active_channels=None):
        self._nusc = nusc
        self._scene_token = scene_token
        channels = active_channels if active_channels is not None else config.ACTIVE_CHANNELS.value
        self._poses = self._collect_ego_poses(nusc, scene_token, channels)
        if not self._poses:
            raise ValueError(f"No ego_pose records found for scene {scene_token}")
        self._pose_times = [t for t, _ in self._poses]

        # Incremental cache: a persistent KF advanced up to the latest
        # pose folded in so far, plus how many poses (from the start)
        # have been processed. state_at() with increasing t_us (the
        # common access pattern -- sweeping forward through a scene) only
        # ever processes NEW poses since the last call, instead of
        # rebuilding the whole chain from scratch every time. A call for
        # an earlier t_us than what's cached falls back to a from-scratch
        # rebuild (_state_at_from_scratch) -- slower, but still exactly
        # correct, and this step's own real-data access patterns never
        # need it (found this optimisation necessary after step03's
        # per-scene speed-vs-time test took several minutes with the
        # naive rebuild-every-call version across ~400 query points and
        # ~1000+ collected poses per scene).
        self._cache_kf: ConstantVelocityKF | None = None
        self._cache_progressed_idx = 0
        self._cache_latest_rotation = None

    @staticmethod
    def _collect_ego_poses(nusc: NuScenes, scene_token: str, channels):
        """One ego_pose per unique ego_pose_token, from the active
        channels' sample_data records in this scene (native rate, not
        just 2Hz keyframes), sorted by the ego_pose record's OWN
        timestamp (step03 spec)."""
        channel_set = set(channels)
        seen_tokens = set()
        poses = []
        for sd in nusc.sample_data:
            channel = nusc.get(
                "sensor", nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])["sensor_token"]
            )["channel"]
            if channel not in channel_set:
                continue
            if nusc.get("sample", sd["sample_token"])["scene_token"] != scene_token:
                continue
            ep_token = sd["ego_pose_token"]
            if ep_token in seen_tokens:
                continue
            seen_tokens.add(ep_token)
            ep = nusc.get("ego_pose", ep_token)
            poses.append((int(ep["timestamp"]), ep))
        poses.sort(key=lambda p: p[0])
        return poses

    @staticmethod
    def _R() -> np.ndarray:
        return (config.EGO_POSE_STD_M.value ** 2) * np.eye(2)

    def _run_chain_from_scratch(self, target_idx: int):
        """Build a fresh KF from self._poses[:target_idx] (poses[0] as
        init, the rest as predict+update). Returns (kf, latest_rotation).
        Used both for the persistent cache's first fill and as the
        correctness fallback for a backward (earlier-than-cached) call."""
        R = self._R()
        kf = ConstantVelocityKF(sigma_a=config.SIGMA_A_EGO.value)
        t0, ep0 = self._poses[0]
        kf.init(t0, np.array(ep0["translation"][:2]), R, v0_std=config.V0_STD_MPS.value)
        last_t = t0
        latest_rotation = ep0["rotation"]
        for t, ep in self._poses[1:target_idx]:
            kf.predict((t - last_t) / 1e6)
            kf.update(np.array(ep["translation"][:2]), R)
            last_t = t
            latest_rotation = ep["rotation"]
        return kf, latest_rotation

    def _finalize(self, kf: ConstantVelocityKF, t_us: int, latest_rotation) -> EgoState:
        final = kf.state_at(t_us)
        vx, vy = final.velocity
        heading = float(Quaternion(latest_rotation).yaw_pitch_roll[0])
        return EgoState(
            t_us=t_us,
            x=float(final.position[0]),
            y=float(final.position[1]),
            vx=float(vx),
            vy=float(vy),
            speed_mps=float(np.hypot(vx, vy)),
            heading_rad=heading,
        )

    def state_at(self, t_us: int) -> EgoState:
        """Causal (G15): uses only poses with timestamp <= t_us, predicts
        forward to t_us. Never looks at poses in the future of t_us."""
        t_us = int(t_us)
        # Number of poses with timestamp <= t_us.
        target_idx = bisect.bisect_right(self._pose_times, t_us)
        if target_idx == 0:
            raise ValueError(f"No ego_pose at or before t_us={t_us} for scene {self._scene_token}")

        if target_idx < self._cache_progressed_idx:
            # Earlier than what's cached -- rebuild from scratch rather
            # than trying to "rewind" the persistent KF (still exactly
            # correct, just not the fast path).
            kf, latest_rotation = self._run_chain_from_scratch(target_idx)
            return self._finalize(kf, t_us, latest_rotation)

        if self._cache_kf is None:
            self._cache_kf, self._cache_latest_rotation = self._run_chain_from_scratch(target_idx)
            self._cache_progressed_idx = target_idx
        elif target_idx > self._cache_progressed_idx:
            R = self._R()
            last_t = self._poses[self._cache_progressed_idx - 1][0]
            for t, ep in self._poses[self._cache_progressed_idx : target_idx]:
                self._cache_kf.predict((t - last_t) / 1e6)
                self._cache_kf.update(np.array(ep["translation"][:2]), R)
                last_t = t
                self._cache_latest_rotation = ep["rotation"]
            self._cache_progressed_idx = target_idx

        return self._finalize(self._cache_kf, t_us, self._cache_latest_rotation)
