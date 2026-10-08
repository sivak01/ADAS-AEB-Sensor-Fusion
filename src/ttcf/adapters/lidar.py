"""
src/ttcf/adapters/lidar.py — STEP 04a: LiDAR detection adapter.

Turns one RawEvent (LIDAR_TOP) into a list[Detection] in the global
frame: load -> transform to ego -> coarse ROI crop -> ground removal
(RANSAC, fit on near-field low points, applied to the whole ROI) ->
clustering (DBSCAN) -> one Detection per cluster, using the DEC-1
nearest-surface reference point.

Precedent check (V1's Step_2_1_LiDAR_Processing.ipynb):
- PORTED: the manual numpy RANSAC ground-plane algorithm. V1 keeps this
  as a validated *reference* implementation only (its actual pipeline
  used Open3D's segment_plane() instead) -- this project uses the manual
  version as the real pipeline choice, since Open3D isn't a dependency
  here and is a heavy addition this project doesn't need for a fair
  rebuild. Same tuned starting numbers (dist_thresh=0.2m, n_points=3,
  iters=100), re-validated on this project's own real-data check.
- PORTED: sklearn.cluster.DBSCAN with V1's tuned eps=0.7m/min_samples=10
  (per the step04a clustering-library decision, 2026-09-23).
- NOT PORTED: V1's cluster reference point is the cluster CENTROID; this
  project's DEC-1 chose nearest-surface-to-ego instead -- built fresh.
- NOT PORTED: V1's cluster shape/size filtering (CLUSTER_MIN_HEIGHT_M,
  CLUSTER_MAX_ASPECT_RATIO, etc.) -- step04a §4 explicitly forbids
  classifying objects with size heuristics (DEC-2).
- Clustering runs on the full 3D (x,y,z) obstacle points, matching V1's
  own dimensionality for its eps value -- clustering only in 2D (x,y)
  with the SAME eps would systematically shrink pairwise distances
  (dropping the z term), effectively over-merging relative to what
  eps=0.7 was tuned for.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from nuscenes.nuscenes import NuScenes
from sklearn.cluster import DBSCAN

from ttcf import config
from ttcf.geometry.transforms import FrameContext, ego_to_global, ego_yaw, rotate_cov_ego_to_global, sensor_to_ego
from ttcf.types import Detection, RawEvent


def _fit_ground_plane_ransac(points_xyz: np.ndarray, dist_thresh: float, n_points: int, iters: int, rng=None):
    """Manual RANSAC plane fit -- ported from V1's own reference
    implementation. Returns (normal, d) with normal @ p + d == 0 on the
    plane, or None if there are too few points to fit."""
    rng = rng if rng is not None else np.random.default_rng(0)
    n = len(points_xyz)
    if n < n_points:
        return None

    best_count = -1
    best_plane = None
    for _ in range(iters):
        idx = rng.choice(n, n_points, replace=False)
        sample = points_xyz[idx]
        v1, v2 = sample[1] - sample[0], sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm == 0:
            continue
        normal = normal / norm
        d = -np.dot(normal, sample[0])
        distances = np.abs(points_xyz @ normal + d)
        count = int(np.sum(distances < dist_thresh))
        if count > best_count:
            best_count, best_plane = count, (normal, d)
    return best_plane


def _ground_mask_from_plane(points_xyz: np.ndarray, plane, dist_thresh: float) -> np.ndarray:
    if plane is None:
        return np.zeros(len(points_xyz), dtype=bool)
    normal, d = plane
    distances = np.abs(points_xyz @ normal + d)
    return distances < dist_thresh


def _cluster_reference_point(cluster_points_ego: np.ndarray, radius_m: float) -> np.ndarray:
    """DEC-1 nearest-surface: mean of the cluster's points within
    radius_m of the single nearest point by range from the ego origin --
    not the bare minimum point alone (too noisy, per step04a §3)."""
    xy = cluster_points_ego[:, :2]
    ranges = np.linalg.norm(xy, axis=1)
    near_face = xy[ranges <= ranges.min() + radius_m]
    return near_face.mean(axis=0)


@dataclass(frozen=True)
class LidarSweepResult:
    detections: list
    plane: Optional[tuple]
    n_roi_points: int
    n_ground_points: int
    n_obstacle_points: int
    n_clusters: int


def process_lidar_sweep(nusc: NuScenes, event: RawEvent, cfg=config, rng=None) -> LidarSweepResult:
    """The full pipeline for one LIDAR_TOP sweep, exposing intermediate
    diagnostics (plane params, point counts) for the real-data report
    (step04a §3.4) -- lidar_detections() below is the thin public
    wrapper matching the spec's literal list[Detection] interface."""
    from nuscenes.utils.data_classes import LidarPointCloud

    ctx = FrameContext.from_event(nusc, event)
    path = cfg.DATAROOT / event.filename
    pc = LidarPointCloud.from_file(str(path))
    points_sensor = pc.points[:3, :].T  # (N,3), sensor frame

    points_ego = sensor_to_ego(points_sensor, ctx)  # (N,3), ego frame (G1/G2 order: transform first)

    max_range = cfg.CORRIDOR_MAX_RANGE_M.value + cfg.LIDAR_ROI_SLACK_M.value
    half_width = (
        cfg.CORRIDOR_HALF_WIDTH_M.value + cfg.CANDIDATE_EXTRA_MARGIN_M.value + cfg.LIDAR_ROI_SLACK_M.value
    )
    # Exclude ego-vehicle self-returns (the sensor hitting its own roof/
    # mount at point-blank range) -- found necessary on real data (see
    # LIDAR_MIN_RANGE_M's own reason in config.py): without this, a
    # single sweep produced a 9805-point "cluster" at x=0-1.4m, z=1.4-1.8m,
    # exactly the LiDAR's own mount height, not a real external object.
    xy_range = np.linalg.norm(points_ego[:, :2], axis=1)
    roi_mask = (
        (xy_range > cfg.LIDAR_MIN_RANGE_M.value)
        & (points_ego[:, 0] > 0)
        & (points_ego[:, 0] < max_range)
        & (np.abs(points_ego[:, 1]) < half_width)
        & (points_ego[:, 2] > cfg.LIDAR_ROI_Z_MIN_M.value)
        & (points_ego[:, 2] < cfg.LIDAR_ROI_Z_MAX_M.value)
    )
    roi_points = points_ego[roi_mask]
    if len(roi_points) == 0:
        return LidarSweepResult([], None, 0, 0, 0, 0)

    # RANSAC fits the plane on NEAR-FIELD LOW points only (step04a §3
    # step 4: avoids car roofs/curbs at range contaminating the fit),
    # then that fitted plane is applied to the WHOLE ROI to decide
    # ground/non-ground membership.
    ranges = np.linalg.norm(roi_points[:, :2], axis=1)
    fit_mask = (ranges < cfg.LIDAR_GROUND_FIT_RANGE_M.value) & (roi_points[:, 2] < cfg.LIDAR_GROUND_FIT_Z_MAX_M.value)
    fit_points = roi_points[fit_mask]

    plane = _fit_ground_plane_ransac(
        fit_points, cfg.GROUND_RANSAC_DIST_THRESH_M.value, cfg.GROUND_RANSAC_N_POINTS.value,
        cfg.GROUND_RANSAC_ITERS.value, rng=rng,
    )
    ground_mask = _ground_mask_from_plane(roi_points, plane, cfg.GROUND_RANSAC_DIST_THRESH_M.value)
    obstacle_points = roi_points[~ground_mask]
    n_ground = int(ground_mask.sum())

    if len(obstacle_points) == 0:
        return LidarSweepResult([], plane, len(roi_points), n_ground, 0, 0)

    labels = DBSCAN(
        eps=cfg.LIDAR_CLUSTER_EPS_M.value, min_samples=cfg.LIDAR_CLUSTER_MIN_SAMPLES.value
    ).fit_predict(obstacle_points)  # 3D clustering -- matches V1's own dimensionality for this eps value

    yaw = ego_yaw(ctx)
    R_lidar = cfg.SENSOR_R_LIDAR.value
    R_ego = np.diag([R_lidar["sigma_long_m"] ** 2, R_lidar["sigma_lat_m"] ** 2])
    R_global = rotate_cov_ego_to_global(R_ego, yaw)

    detections = []
    cluster_labels = sorted(set(labels.tolist()) - {-1})
    for label in cluster_labels:
        cluster_pts_ego = obstacle_points[labels == label]
        ref_ego = _cluster_reference_point(cluster_pts_ego, cfg.NEAREST_SURFACE_RADIUS_M.value)
        ref_global = ego_to_global(np.array([ref_ego]), ctx)[0][:2]
        extent_m = float(np.ptp(cluster_pts_ego[:, :2], axis=0).max())

        detections.append(
            Detection(
                t_us=event.t_us,
                channel=event.channel,
                modality=event.modality,
                xy_global=ref_global,
                R_global=R_global,
                ref_point_kind="nearest_surface_to_ego",
                cls=None,  # DEC-2: never classify by size/shape
                n_points=int((labels == label).sum()),
                aux={"cluster_extent_m": extent_m},
            )
        )

    return LidarSweepResult(
        detections, plane, len(roi_points), n_ground, len(obstacle_points), len(cluster_labels)
    )


def lidar_detections(nusc: NuScenes, event: RawEvent, cfg=config, rng=None) -> list:
    """Public adapter interface (step04a §3): RawEvent -> list[Detection]."""
    return process_lidar_sweep(nusc, event, cfg, rng=rng).detections
