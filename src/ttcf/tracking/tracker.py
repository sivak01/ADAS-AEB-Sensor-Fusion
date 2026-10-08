"""
src/ttcf/tracking/tracker.py — STEP 06: shared short-memory tracker (G10, G11).

Event-driven: ShortMemoryTracker.process_scan(dets, t_us) -> list[TrackSnapshot],
one call per sensor scan. Detections from one scan never update the same
track twice (NxM Hungarian assignment). Early fusion: LiDAR, radar, and
camera detections all write into ONE shared track set -- there is no
separate per-sensor tracker or later merge step (G10).

Ported from the predecessor project's v2/central_tracker.py: predict-then-
gate ordering, eviction BEFORE gating (an already-expired track can never
steal a scan's detections), innovation-covariance gating (S = P_pred + R,
via the shared KF's own mahalanobis_sq/innovation_cov -- step03), and the
early-fusion architecture itself (one tracker, fed identically whether one
sensor's stream or the merged stream).

NOT ported (G11 / what-not-to-do.md #1): v2's MAX_MISSED_SECONDS-scale
eviction (this project uses the much shorter EVICTION_GAP_S); v2's scene-
boundary reset as its own subsystem (short-lived tracks make this
unnecessary -- a tracker instance is simply given one scene's events at a
time); v2's lifetime-growing sensors_seen set and unbounded Track.history
-- replaced by MAX_MEMORY_UPDATES-bounded recent-sensor tracking (DEC-5).

Genuine complexity increase over v2: v2's process_event always handled
exactly one detection (a trivial 1xN assignment); process_scan here
handles a LIST of detections per scan, needing real NxM Hungarian
assignment with an infeasible-cost sentinel for below-gate pairs.

A literal spec detail: an unmatched track is "untouched" this scan -- its
internal KF state is NOT mutated (unlike v2, which mutates every track's
predict every event). Mathematically equivalent for a linear CV-KF
(predict(a) then predict(b) == predict(a+b)) and cheaper; the tracker
still returns a same-instant, predicted-to-t_us snapshot for every live
track via a non-mutating peek (state_at) -- it just doesn't commit that
peek internally unless the track is actually matched.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

from ttcf import config
from ttcf.filtering.kalman import ConstantVelocityKF, H4
from ttcf.types import Detection, TrackSnapshot

# Substituted for any track/detection pair outside the gate -- large
# enough Hungarian always prefers any feasible pairing over one at this
# cost, but finite so linear_sum_assignment (which needs a finite cost
# matrix) still runs. Same technique as v2/gating.py's INFEASIBLE_COST.
INFEASIBLE_COST = 1e6


def _kf_update_args(det: Detection):
    """Position-only update by default (every official run, every sensor,
    unchanged). Combined position+velocity update (H=H4) only when a
    detection carries a genuine second measurement channel -- currently
    only radar's own Doppler, under the G17 ablation
    (RADAR_DOPPLER_ABLATION, approved 2026-10-08) -- checked generically
    here via Detection's own optional fields, never a sensor-name check
    (this tracker's own "zero references to sensor names" discipline)."""
    if det.velocity_global is not None:
        z = np.concatenate([det.xy_global, det.velocity_global])
        R = np.zeros((4, 4))
        R[:2, :2] = det.R_global
        R[2:, 2:] = det.R_velocity_global
        return z, R, H4
    return det.xy_global, det.R_global, None


@dataclass(frozen=True)
class AssociationLogEntry:
    """One decision the tracker made for one incoming detection -- matched
    or spawned, its own d^2 (None if spawned -- there is no 'winning
    track' to report a distance to), and which track it ended up on.
    Class is recorded here (never used in gating -- DEC-2)."""

    t_us: int
    channel: str
    cls: Optional[str]
    outcome: str  # "matched" or "spawned"
    d2: Optional[float]
    track_id: str


@dataclass
class _Track:
    """Internal bookkeeping wrapping one ConstantVelocityKF -- track-level
    metadata (id, recent-sensor window, update count) does not belong in
    the sensor-agnostic KF itself (step03's own separation of concerns)."""

    track_id: str
    kf: ConstantVelocityKF
    first_t_us: int
    last_update_t_us: int
    n_updates: int
    recent_channels: deque


class ShortMemoryTracker:
    """No per-sensor trackers merged afterward, no scene-boundary reset
    subsystem, no lifetime sensor bookkeeping, no appearance re-ID
    (what-not-to-do.md §1). Ignores Detection.aux entirely (G17) -- a
    genuine second measurement channel (e.g. radar Doppler under the
    RADAR_DOPPLER_ABLATION, approved 2026-10-08) is carried on its OWN
    dedicated fields (velocity_global/R_velocity_global), never aux, and
    is off (None) for every adapter/run by default; see _kf_update_args."""

    def __init__(self, sigma_a: Optional[float] = None):
        self._sigma_a = sigma_a if sigma_a is not None else config.SIGMA_A.value
        self._tracks: dict[str, _Track] = {}
        self._next_id = 0
        self.association_log: list[AssociationLogEntry] = []
        self.n_matches = 0
        self.n_spawns = 0
        self.n_evictions = 0
        self._n_snapshots_total = 0
        self._n_snapshots_multi_sensor = 0

    def _new_track_id(self) -> str:
        tid = f"trk_{self._next_id:06d}"
        self._next_id += 1
        return tid

    def _evict_stale(self, t_us: int) -> None:
        gap_s = config.EVICTION_GAP_S.value
        for tid in list(self._tracks.keys()):
            if (t_us - self._tracks[tid].last_update_t_us) / 1e6 > gap_s:
                del self._tracks[tid]
                self.n_evictions += 1

    def process_scan(self, dets: list[Detection], t_us: int) -> list[TrackSnapshot]:
        t_us = int(t_us)

        # Evict BEFORE gating -- an already-expired track can never steal
        # this scan's detections (v2 precedent, G11).
        self._evict_stale(t_us)

        # 1. Predict every live track to t_us NON-mutatingly (peek).
        track_ids = list(self._tracks.keys())
        peeked = {tid: self._tracks[tid].kf.state_at(t_us) for tid in track_ids}

        # 2-3. Cost matrix: d^2 via the peeked KF's own mahalanobis_sq
        # (S = P_pred + R_det, the incoming detection's own noise
        # included -- G7), gated at CHI2_GATE.
        n_tracks, n_dets = len(track_ids), len(dets)
        cost = np.full((n_tracks, n_dets), INFEASIBLE_COST)
        gate = config.CHI2_GATE.value
        for i, tid in enumerate(track_ids):
            for j, det in enumerate(dets):
                d2 = peeked[tid].mahalanobis_sq(det.xy_global, det.R_global)
                if d2 <= gate:
                    cost[i, j] = d2

        matched_tids: set[str] = set()
        matched_det_idx: set[int] = set()
        if n_tracks and n_dets:
            row_idx, col_idx = linear_sum_assignment(cost)
            for r, c in zip(row_idx, col_idx):
                if cost[r, c] >= INFEASIBLE_COST:
                    continue
                tid = track_ids[r]
                det = dets[c]
                d2 = float(cost[r, c])
                track = self._tracks[tid]

                # 4. Matched -> predict (mutating, to t_us) then update.
                dt_s = (t_us - track.kf._t_us) / 1e6
                track.kf.predict(dt_s)
                z, R, H = _kf_update_args(det)
                track.kf.update(z, R, H=H)
                track.n_updates += 1
                track.last_update_t_us = t_us
                track.recent_channels.append(det.channel)

                self.association_log.append(
                    AssociationLogEntry(
                        t_us=t_us, channel=det.channel, cls=det.cls,
                        outcome="matched", d2=d2, track_id=tid,
                    )
                )
                self.n_matches += 1
                matched_tids.add(tid)
                matched_det_idx.add(c)

        # Unmatched detections -> spawn tentative tracks (velocity 0,
        # std V0_STD_MPS -- KF.init()'s own behaviour, step03).
        newly_spawned_tids: set[str] = set()
        for j, det in enumerate(dets):
            if j in matched_det_idx:
                continue
            tid = self._new_track_id()
            kf = ConstantVelocityKF(sigma_a=self._sigma_a)
            kf.init(t_us, det.xy_global, det.R_global, v0_std=config.V0_STD_MPS.value)
            self._tracks[tid] = _Track(
                track_id=tid,
                kf=kf,
                first_t_us=t_us,
                last_update_t_us=t_us,
                n_updates=1,
                recent_channels=deque([det.channel], maxlen=config.MAX_MEMORY_UPDATES.value),
            )
            self.association_log.append(
                AssociationLogEntry(
                    t_us=t_us, channel=det.channel, cls=det.cls,
                    outcome="spawned", d2=None, track_id=tid,
                )
            )
            self.n_spawns += 1
            newly_spawned_tids.add(tid)

        # Unmatched pre-existing tracks: untouched (no internal mutation) --
        # see module docstring for why this is safe.

        # Every currently-active track gets a snapshot at t_us: matched/
        # newly-spawned tracks' kf is already exactly there; everything
        # else uses its non-mutating peek from step 1.
        snapshots = []
        for tid, track in self._tracks.items():
            if tid in matched_tids or tid in newly_spawned_tids:
                state = track.kf
            else:
                state = peeked[tid]
            sensors_in_estimate = frozenset(track.recent_channels)

            self._n_snapshots_total += 1
            if len(sensors_in_estimate) >= 2:
                self._n_snapshots_multi_sensor += 1

            snapshots.append(
                TrackSnapshot(
                    track_id=tid,
                    t_us=t_us,
                    x=state.x.copy(),
                    P=state.P.copy(),
                    n_updates=track.n_updates,
                    sensors_in_estimate=sensors_in_estimate,
                    first_t_us=track.first_t_us,
                )
            )
        return snapshots

    @property
    def multi_sensor_fraction(self) -> float:
        """Fraction of all snapshots ever emitted whose sensors_in_estimate
        had >=2 sensors -- to detect V1's dilution problem later."""
        if self._n_snapshots_total == 0:
            return 0.0
        return self._n_snapshots_multi_sensor / self._n_snapshots_total
