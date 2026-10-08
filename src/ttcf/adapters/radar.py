"""
src/ttcf/adapters/radar.py — STEP 04b: radar detection adapter (Doppler OFF
by default; see RADAR_DOPPLER_ABLATION below for the opt-in exception).

Turns one RawEvent (RADAR_FRONT/FRONT_LEFT/FRONT_RIGHT) into a list[Detection]
in the global frame: load (devkit's own default point filters) -> transform
to ego -> cluster per channel, per scan, in xy only (z ignored -- unreliable)
-> one Detection per cluster, DEC-1 nearest-surface reference point.

No coarse ROI crop here (unlike step04a's LiDAR adapter) -- step04b §3 doesn't
ask for one, radar point counts are already small (no compute concern LiDAR's
tens-of-thousands-of-points crop existed to address), and step05's candidate
gate already runs downstream (run_pipeline.py's run_real_stream, stream-
agnostic) regardless of which sensor produced the detection.

Precedent check (V1's Step_3_2_Radar_Fusion.ipynb / V2's adapters.py):
- PORTED: DBSCAN clustering, eps=1.5m (2x LiDAR's 0.7m -- sparser returns
  spread wider per object), min_samples=1 (radar rarely gives >1-2 points
  per object per scan; requiring more would drop real single-blip
  detections). Clustered per-channel-per-scan (V2's variant, not V1's
  pooled-across-5-channels variant -- V2's own stated reason: pooling mixes
  points from slightly different real timestamps/channels, which this
  project's own G2 rule ("never borrow another channel's timestamp/pose")
  agrees with).
- NOT PORTED: V1/V2's centroid reference point -- this project's DEC-1
  (nearest-surface-to-ego) is a project-wide, cross-sensor decision
  (README_MASTER.md §6: "stays consistent across sensors"), reusing
  lidar.py's own _cluster_reference_point unchanged rather than giving
  radar a different rule.
- NOT PORTED: V1's extra `dyn_prop in {0,2,6}` stationary-clutter filter on
  top of the devkit's own defaults. Confirmed devkit defaults directly
  (invalid_states=[0], dynprop_states=range(0,7) i.e. NO dynprop filtering,
  ambig_states=[3]) -- V1's filter was its own addition, not something "the
  devkit already offers" (step04b §3 step 1's own distinction). step04b's
  spec frames ghost returns as a test of the DOWNSTREAM pipeline (forward-
  path filter/tracker/debounce), not something to hack around here --
  decision confirmed with Siva 2026-09-23 (docs/decisions.md), diverges
  from V1 on purpose.
- NOT PORTED: Doppler wired into any filter/TTC/KF anywhere -- already this
  project's own G17 (Doppler OFF by default), consistent with V2's own R10
  rule and what-not-to-do.md. Stored in aux only, ignored by the tracker,
  UNLESS `RADAR_DOPPLER_ABLATION=True` (post-step11, approved 2026-10-08),
  in which case vx_comp/vy_comp are additionally rotated to the global
  frame and attached as Detection.velocity_global/R_velocity_global -- the
  tracker (step06) then does a combined position+velocity KF update instead
  of position-only. OFF by default for every official result in this
  project; prompted by the predecessor project's own measured finding that
  this was its single largest accuracy win (radar standalone error -31%).
- No R-measurement-method precedent reused (V1's own attempt for radar was
  found unusable per lessons-from-v1-v2.md item A.10 -- 2.4x fragmentation,
  ~86% inlier rate); measure_sensor_R.py (extended for radar) re-measures
  fresh against THIS project's own adapter/tracker instead, exactly as
  step04a did for LiDAR.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from nuscenes.nuscenes import NuScenes
from sklearn.cluster import DBSCAN

from ttcf import config
from ttcf.adapters.lidar import _cluster_reference_point
from ttcf.geometry.transforms import (
    FrameContext,
    ego_to_global,
    ego_yaw,
    rotate_cov_ego_to_global,
    sensor_to_ego,
    sensor_to_global_vector,
)
from ttcf.types import Detection, RawEvent

# RadarPointCloud row indices, per the devkit's own from_file() docstring
# field order: x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid
# ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms.
_DYN_PROP = 3
_RCS = 5
_VX_COMP = 8
_VY_COMP = 9
_X_RMS = 12
_Y_RMS = 13

# dynProp values 1 (stationary), 3 (stationary candidate), 5 (crossing
# stationary) -- the devkit's own table (RadarPointCloud.from_file docstring)
# -- are the "ghost return" candidates step04b §3 names (guard rails, signs,
# manhole covers). Recorded per-detection for step10 false-brake
# ATTRIBUTION only; never used to filter here (see module docstring).
_STATIONARY_DYN_PROP = frozenset({1, 3, 5})


@dataclass(frozen=True)
class RadarScanResult:
    detections: list
    n_points: int
    n_clusters: int


def process_radar_scan(nusc: NuScenes, event: RawEvent, cfg=config, rng=None) -> RadarScanResult:
    """The full pipeline for one radar channel's one scan, exposing
    intermediate diagnostics (raw point count) for the real-data report
    (step04b §5 test 6) -- radar_detections() below is the thin public
    wrapper matching the spec's literal list[Detection] interface."""
    from nuscenes.utils.data_classes import RadarPointCloud

    ctx = FrameContext.from_event(nusc, event)
    path = cfg.DATAROOT / event.filename
    # No extra filter args -> devkit's own class-level defaults (see module
    # docstring) -- verified directly: invalid_states=[0], dynprop_states=
    # range(0,7) (no dynprop filtering), ambig_states=[3].
    pc = RadarPointCloud.from_file(str(path))
    n_points = pc.points.shape[1]
    if n_points == 0:
        return RadarScanResult([], 0, 0)

    points_sensor = pc.points[:3, :].T  # (N,3), sensor frame -- x,y,z rows only
    points_ego = sensor_to_ego(points_sensor, ctx)  # G1/G2: transform first, this event's own pose

    dyn_prop = pc.points[_DYN_PROP, :]
    rcs = pc.points[_RCS, :]
    vx_comp = pc.points[_VX_COMP, :]
    vy_comp = pc.points[_VY_COMP, :]
    x_rms = pc.points[_X_RMS, :]
    y_rms = pc.points[_Y_RMS, :]

    # Cluster in xy only (z ignored -- unreliable, step04b §3). min_samples=1
    # means DBSCAN never labels a point as noise (-1) here -- every point
    # that survived the devkit's own filter becomes part of some cluster.
    labels = DBSCAN(
        eps=cfg.RADAR_CLUSTER_EPS_M.value, min_samples=cfg.RADAR_CLUSTER_MIN_SAMPLES.value
    ).fit_predict(points_ego[:, :2])

    yaw = ego_yaw(ctx)
    R_radar = cfg.SENSOR_R_RADAR.value
    R_ego = np.diag([R_radar["sigma_long_m"] ** 2, R_radar["sigma_lat_m"] ** 2])
    R_global = rotate_cov_ego_to_global(R_ego, yaw)

    doppler_ablation = cfg.RADAR_DOPPLER_ABLATION.value
    if doppler_ablation:
        vel_var = cfg.RADAR_VELOCITY_NOISE_VAR_MPS2.value
        R_vel_ego = np.diag([vel_var, vel_var])  # isotropic -- see config.py's own reason

    detections = []
    cluster_labels = sorted(set(labels.tolist()) - {-1})
    for label in cluster_labels:
        mask = labels == label
        cluster_pts_ego = points_ego[mask]
        ref_ego = _cluster_reference_point(cluster_pts_ego, cfg.NEAREST_SURFACE_RADIUS_M.value)
        ref_global = ego_to_global(np.array([ref_ego]), ctx)[0][:2]

        velocity_global = None
        R_velocity_global = None
        if doppler_ablation:
            # vx_comp/vy_comp are in the RADAR SENSOR's own frame (ego-
            # motion-compensated, per the devkit's own convention) -- the
            # same sensor->ego->global chain as position, but translation-
            # free (_apply_vector/w=0, step02's own vector-rotation helpers).
            v_sensor_mean = np.array([[float(np.mean(vx_comp[mask])), float(np.mean(vy_comp[mask]))]])
            velocity_global = sensor_to_global_vector(v_sensor_mean, ctx)[0]
            R_velocity_global = rotate_cov_ego_to_global(R_vel_ego, yaw)

        detections.append(
            Detection(
                t_us=event.t_us,
                channel=event.channel,
                modality=event.modality,
                xy_global=ref_global,
                R_global=R_global,
                ref_point_kind="nearest_surface_to_ego",
                cls=None,  # DEC-2: never classify by size/shape
                n_points=int(mask.sum()),
                aux={
                    "doppler_vx_comp": float(np.mean(vx_comp[mask])),
                    "doppler_vy_comp": float(np.mean(vy_comp[mask])),
                    "rcs": float(np.mean(rcs[mask])),
                    "n_points": int(mask.sum()),
                    "x_rms": float(np.mean(x_rms[mask])),
                    "y_rms": float(np.mean(y_rms[mask])),
                    # Ghost-return ATTRIBUTION only (step10), never a filter
                    # here (step04b decision, 2026-09-23 -- see module docstring).
                    "n_stationary_pts": int(np.isin(dyn_prop[mask], list(_STATIONARY_DYN_PROP)).sum()),
                },
                velocity_global=velocity_global,
                R_velocity_global=R_velocity_global,
            )
        )

    return RadarScanResult(detections, n_points, len(cluster_labels))


def radar_detections(nusc: NuScenes, event: RawEvent, cfg=config, rng=None) -> list:
    """Public adapter interface (step04b §3): RawEvent -> list[Detection].
    `rng` accepted for interface parity with lidar_detections() (unused --
    radar clustering has no random-sampling step, unlike LiDAR's RANSAC)."""
    return process_radar_scan(nusc, event, cfg).detections
