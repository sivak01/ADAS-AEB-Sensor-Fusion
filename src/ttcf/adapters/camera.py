"""
src/ttcf/adapters/camera.py — STEP 04c: camera detection adapter, genuinely
single-sensor (G14).

Turns one RawEvent (CAM_FRONT) into a list[Detection] in the global frame:
2D detector boxes (pre-cached, never run live here -- inference is slow) ->
drop truncated-bottom boxes -> bottom-centre pixel -> camera ray via
intrinsics -> ground-plane intersection in the ego frame (DEC-6) -> global
frame, with a range-dependent R (step04c §3.3).

**No LiDAR, radar, or GT-derived depth/position anywhere in this file**
(what-not-to-do.md §6) -- this is the exact thing V1's own `camera` baseline
got wrong (see the precedent check in docs/decisions.md, 2026-09-24): V1's
`camera` row looked up the median point-cloud return inside each 2D box for
depth, so its "camera-only" result was secretly fused with another sensor;
V1's separate mono-only pipeline (built later specifically to correct this)
used genuinely leak-free similar-triangles depth (assumed per-class object
height) instead. This project's own isolation test
(tests/test_camera_isolation.py) statically scans this module's source and
its imports for the other two sensors' loader classes, their own dataset
channel-name prefixes, and any GT-annotation access -- so this mistake is
structurally, not just procedurally, impossible to repeat. (Those exact
forbidden names are intentionally NOT spelled out here, so this docstring
itself never trips the isolation test's own naive text scan.)

Precedent check (V1's Step_2_3_YOLOv5.ipynb-family notebooks):
- PORTED: Ultralytics YOLOv8n, CPU inference, conf=0.35/iou=0.45, COCO
  classes [0,1,2,3,5,7] (person/bicycle/car/motorcycle/bus/truck) -- V1's
  own tuned choice on this exact dataset/model, confirmed with Siva
  2026-09-24 (our own GPU -- GeForce GT 710, 2GB VRAM -- can't meaningfully
  accelerate a heavier model).
- NOT PORTED: V1's `camera` (LiDAR-assisted) depth -- forbidden by design,
  see above.
- NOT PORTED: V1's `camera_mono` similar-triangles depth (assumed object
  height / pixel height * focal length) -- DEC-6 (confirmed 2026-09-24)
  chose ground-plane back-projection instead, which has ZERO precedent in
  V1/V2 (checked directly); built entirely fresh. Bias mechanism differs:
  similar-triangles' bias is systematic per-instance (assumed height is
  often wrong for THIS specific object); ground-plane's bias is systematic
  per-range/pitch (grows with range^2, fails on slopes) -- a documented
  limitation, not a hidden one.
- NOT PORTED: V1's unified 6-camera global tracker (Hungarian assignment,
  DIST_THRESHOLD, MAX_MISSED_FRAMES) -- MOT-scope machinery this project's
  own G11/what-not-to-do.md §1 already reject; this adapter only needs
  per-event Detection output, not a multi-frame camera-only tracker.
- No detection-caching precedent exists in V1 (results were just
  overwritten per run) -- scripts/cache_camera_detections.py is built
  fresh, per step04c §2's own explicit requirement.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
from nuscenes.nuscenes import NuScenes

from ttcf import config
from ttcf.geometry.transforms import FrameContext, ego_to_global, ego_yaw, rotate_cov_ego_to_global, sensor_to_ego
from ttcf.types import Detection, RawEvent

CACHE_DIR = Path("outputs/cache/camera_dets")

# The 2D-box dict schema this adapter expects, from either the on-disk
# cache (scripts/cache_camera_detections.py's own output) or an injected
# box_source (tests): {"bbox": [x1, y1, x2, y2], "cls_id": int, "conf": float}.
BoxSource = Callable[[NuScenes, RawEvent], list]


def _cache_path(model_key: str, sample_data_token: str) -> Path:
    return CACHE_DIR / model_key / f"{sample_data_token}.json"


def _load_cached_boxes(event: RawEvent, cfg=config) -> Optional[list]:
    """Reads pre-computed 2D boxes from the on-disk cache written by
    scripts/cache_camera_detections.py -- NEVER runs live YOLO inference
    here (inference is slow and pre-computed in bulk, step04c §2). Returns
    None on a cache miss; the caller decides how to handle it."""
    path = _cache_path(cfg.CAMERA_DETECTOR_MODEL.value, event.sample_data_token)
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _pixel_ray_ego(u: float, v: float, K: np.ndarray, ctx: FrameContext) -> tuple:
    """Returns (camera_origin_ego, ray_direction_ego) for pixel (u, v).
    K^-1 [u,v,1]^T gives the ray direction in the CAMERA's own frame
    (step04c §3.2 step 1); rotated to ego by reusing the tested
    sensor_to_ego() POINT-transform (step02) on two points and
    differencing -- this project's own rule is to never hand-roll
    quaternion/rotation math a second time."""
    ray_cam = np.linalg.inv(K) @ np.array([u, v, 1.0])
    two_points_sensor = np.array([[0.0, 0.0, 0.0], ray_cam])
    two_points_ego = sensor_to_ego(two_points_sensor, ctx)
    origin_ego, tip_ego = two_points_ego[0], two_points_ego[1]
    return origin_ego, tip_ego - origin_ego


def _ground_intersection_ego(origin_ego: np.ndarray, direction_ego: np.ndarray, ground_z: float):
    """Intersects a ray with the flat ground plane z=ground_z (ego frame,
    DEC-6). None if the ray can never reach the ground ahead of the camera
    (pointing level or upward, or behind) -- both mean "no usable ground
    contact from this pixel", not an error to paper over."""
    dz = float(direction_ego[2])
    if dz >= -1e-9:
        return None
    t = (ground_z - origin_ego[2]) / dz
    if t <= 0:
        return None
    return origin_ego + t * direction_ego


def _camera_R(range_m: float, focal_px: float, cam_height_m: float, cfg=config) -> np.ndarray:
    """Range-dependent camera R (step04c §3.3, ego-frame long/lat diagonal
    covariance). sigma_long grows with range^2 (a fixed pixel error at the
    box's bottom edge maps to a growing ground-range error as the viewing
    ray becomes more grazing); sigma_lat grows linearly with range."""
    sigma_v = cfg.CAMERA_PIXEL_SIGMA_V_PX.value
    sigma_u = cfg.CAMERA_PIXEL_SIGMA_U_PX.value
    sigma_long = (range_m**2) * sigma_v / (focal_px * cam_height_m)
    sigma_lat = range_m * sigma_u / focal_px
    return np.diag([sigma_long**2, sigma_lat**2])


@dataclass(frozen=True)
class CameraFrameResult:
    detections: list
    n_raw_boxes: int
    n_class_filtered: int
    n_truncated_dropped: int
    n_ground_intersection_failed: int


def process_camera_frame(
    nusc: NuScenes, event: RawEvent, cfg=config, box_source: Optional[BoxSource] = None
) -> CameraFrameResult:
    """The full pipeline for one CAM_FRONT frame. `box_source`, if given,
    REPLACES the on-disk cache lookup entirely -- how tests inject
    synthetic boxes without a real cache or YOLO (test_camera_isolation.py's
    dependency-injection requirement, step04c §5 test 1)."""
    ctx = FrameContext.from_event(nusc, event)
    cs = ctx.calibrated_sensor
    K = np.array(cs["camera_intrinsic"], dtype=float)
    cam_height_m = float(cs["translation"][2])
    focal_px = float(K[0, 0])

    raw_boxes = box_source(nusc, event) if box_source is not None else _load_cached_boxes(event, cfg)
    if raw_boxes is None:
        raise FileNotFoundError(
            f"No cached camera detections for sample_data {event.sample_data_token} "
            f"(model={cfg.CAMERA_DETECTOR_MODEL.value!r}) -- run "
            f"scripts/cache_camera_detections.py first."
        )
    n_raw = len(raw_boxes)

    sd = nusc.get("sample_data", event.sample_data_token)
    image_h = int(sd["height"])
    margin = cfg.CAMERA_TRUNCATED_BOTTOM_MARGIN_PX.value
    relevant = cfg.CAMERA_RELEVANT_COCO_CLASSES.value

    class_filtered = [b for b in raw_boxes if b["cls_id"] in relevant]
    n_class_filtered = n_raw - len(class_filtered)

    kept, n_truncated = [], 0
    for b in class_filtered:
        _, _, _, y2 = b["bbox"]
        if y2 >= image_h - margin:
            n_truncated += 1
            continue
        kept.append(b)

    yaw = ego_yaw(ctx)
    ground_z = cfg.GROUND_Z_EGO.value

    detections, n_ground_fail = [], 0
    for b in kept:
        x1, y1, x2, y2 = b["bbox"]
        u, v = (x1 + x2) / 2.0, y2  # bottom-centre pixel (step04c §3.2)

        origin_ego, direction_ego = _pixel_ray_ego(u, v, K, ctx)
        point_ego = _ground_intersection_ego(origin_ego, direction_ego, ground_z)
        if point_ego is None:
            n_ground_fail += 1
            continue

        range_m = float(np.linalg.norm(point_ego - origin_ego))
        R_ego = _camera_R(range_m, focal_px, cam_height_m, cfg)
        R_global = rotate_cov_ego_to_global(R_ego, yaw)
        ref_global = ego_to_global(np.array([point_ego]), ctx)[0][:2]

        detections.append(
            Detection(
                t_us=event.t_us,
                channel=event.channel,
                modality=event.modality,
                xy_global=ref_global,
                R_global=R_global,
                ref_point_kind="nearest_surface_to_ego",
                cls=relevant.get(b["cls_id"]),  # recorded, never used for gating (DEC-2)
                n_points=None,  # not point-cloud based -- no meaningful n_points for a 2D box
                aux={"bbox": b["bbox"], "conf": b["conf"], "range_m": range_m},
            )
        )

    return CameraFrameResult(
        detections=detections,
        n_raw_boxes=n_raw,
        n_class_filtered=n_class_filtered,
        n_truncated_dropped=n_truncated,
        n_ground_intersection_failed=n_ground_fail,
    )


def camera_detections(
    nusc: NuScenes, event: RawEvent, cfg=config, rng=None, box_source: Optional[BoxSource] = None
) -> list:
    """Public adapter interface (step04c §3): RawEvent -> list[Detection].
    `rng` accepted for interface parity with lidar_detections()/
    radar_detections() (unused -- this adapter has no random-sampling step)."""
    return process_camera_frame(nusc, event, cfg, box_source=box_source).detections
