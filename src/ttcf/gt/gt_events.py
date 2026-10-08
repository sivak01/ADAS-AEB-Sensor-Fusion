"""
src/ttcf/gt/gt_events.py — STEP D0: the ground-truth event table (the
"answer key"), built before any detector or tracker exists.

One GT row = one (keyframe, in-path GT object) pair. Uses the SAME
corridor / TTC / distance-definition code the pipeline itself will use
later (G9) -- one implementation, two callers:
  - in-path decision: ttcf.geometry.corridor.footprint_intersects_corridor
  - reference point:  ttcf.geometry.corridor.nearest_point_on_footprint (DEC-1)
  - ego velocity:     ttcf.filtering.ego_state.EgoStateEstimator (step03, causal)
  - GT TTC:           ttcf.ttc.ttc_math.ttc_from_state (step07 Part A)

GT velocity uses the devkit's OWN nusc.box_velocity() -- never a hand-rolled
finite difference (what-not-to-do.md §3; nuscenes-reference.md §4 names
the predecessor project's own compute_ground_truth_ttc() as the exact
wrong pattern to re-derive). It may return NaN when neighbouring
annotations are missing or too far apart in time -- recorded as
`vel_valid`, excluded from the TTC-based action label, but still counted
in the row.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import Box
from pyquaternion import Quaternion

from ttcf import config
from ttcf.filtering.ego_state import EgoStateEstimator
from ttcf.geometry.corridor import footprint_intersects_corridor, nearest_point_on_footprint
from ttcf.geometry.transforms import FrameContext, ego_to_global, global_to_ego
from ttcf.ttc.ttc_math import TTCMathConfig, ttc_from_state

# global_to_ego/ego_to_global only ever read ctx.ego_pose (never
# ctx.calibrated_sensor) -- GT box positions are already global-frame
# (sample_annotation.translation), so no real sensor calibration is
# needed here; this identity stands in for the unused field.
_IDENTITY_CS = {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]}

# Box.corners() returns 8 corners as two 4-corner faces (front face then
# rear face, each ordered top-left/top-right/bottom-right/bottom-left);
# these 4 indices pick one corner from each of the 4 vertical edges, in
# order, tracing the rectangle's perimeter exactly once (verified against
# a known box: center=(10,0,0), size=(2,4,1.5), yaw=0 -> x in [8,12], y in
# [-1,1], corners in this order go (12,1)->(12,-1)->(8,-1)->(8,1)).
_FOOTPRINT_CORNER_IDX = [0, 1, 5, 4]

_ACTION_SEVERITY = {"NONE": 0, "GRADUAL": 1, "AEB": 2}
_SEVERITY_TO_ACTION = {v: k for k, v in _ACTION_SEVERITY.items()}


@dataclass(frozen=True)
class GTEventRow:
    scene_token: str
    scene_name: str
    sample_token: str
    t_us: int
    instance_token: str
    category: str
    attributes: str  # semicolon-joined attribute names
    visibility_level: int  # 1-4 (v0-40 .. v80-100)
    num_lidar_pts: int
    num_radar_pts: int
    distance_m: float
    closing_speed_mps: float
    gt_ttc_s: float
    gt_action: str  # "NONE" / "GRADUAL" / "AEB"
    vel_valid: bool
    obs_lidar: bool
    obs_radar: bool
    obs_camera: bool
    obs_any: bool


@dataclass(frozen=True)
class KeyframeRow:
    scene_token: str
    scene_name: str
    sample_token: str
    t_us: int
    n_in_path: int
    keyframe_action: str
    is_empty_corridor: bool


def gt_action_from_ttc(ttc_s: float) -> str:
    if ttc_s <= config.TTC_AEB_S.value:
        return "AEB"
    if ttc_s <= config.TTC_GRADUAL_S.value:
        return "GRADUAL"
    return "NONE"


def _footprint_corners_global(ann: dict) -> np.ndarray:
    """(4,3) footprint corners in the GLOBAL frame, ordered around the
    rectangle's perimeter -- see _FOOTPRINT_CORNER_IDX."""
    box = Box(center=ann["translation"], size=ann["size"], orientation=Quaternion(ann["rotation"]))
    return box.corners()[:, _FOOTPRINT_CORNER_IDX].T


def build_scene_gt_events(nusc: NuScenes, scene_token: str):
    """Returns (gt_rows: list[GTEventRow], keyframe_rows: list[KeyframeRow])
    for every keyframe in one scene."""
    scene = nusc.get("scene", scene_token)
    ego_est = EgoStateEstimator(nusc, scene_token)
    ttc_cfg = TTCMathConfig(
        min_trusted_speed_mps=config.MIN_TRUSTED_SPEED_MPS.value,
        min_closing_speed_mps=config.MIN_CLOSING_SPEED_MPS.value,
    )
    categories = set(config.GT_CATEGORIES.value)
    half_width = config.CORRIDOR_HALF_WIDTH_M.value
    max_range = config.CORRIDOR_MAX_RANGE_M.value

    gt_rows: list[GTEventRow] = []
    keyframe_rows: list[KeyframeRow] = []

    sample_token = scene["first_sample_token"]
    while sample_token:
        sample = nusc.get("sample", sample_token)
        t_us = int(sample["timestamp"])

        # GT instant / ego pose per step D0 §3.1: the LIDAR_TOP keyframe
        # sample_data record's own (raw, unsmoothed) ego_pose -- position
        # only. Ego VELOCITY still comes from the causal, KF-derived
        # EgoStateEstimator (step03), never a finite difference (G5).
        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        ego_pose_raw = nusc.get("ego_pose", lidar_sd["ego_pose_token"])
        ego_xy_global = np.array(ego_pose_raw["translation"][:2])

        ego_state = ego_est.state_at(t_us)
        ego_v_xy_global = np.array([ego_state.vx, ego_state.vy])

        ctx = FrameContext(calibrated_sensor=_IDENTITY_CS, ego_pose=ego_pose_raw)

        n_in_path = 0
        severities_this_keyframe: list[int] = []

        for ann_token in sample["anns"]:
            ann = nusc.get("sample_annotation", ann_token)
            category = ann["category_name"]
            if category not in categories:
                continue

            corners_global = _footprint_corners_global(ann)
            corners_ego = global_to_ego(corners_global, ctx)[:, :2]
            if not footprint_intersects_corridor(corners_ego, half_width, max_range):
                continue
            n_in_path += 1

            # DEC-1 reference point, computed the same way as corridor
            # membership (nearest point of the footprint), then mapped
            # back to global frame -- TTC itself is computed in the
            # global frame since it's frame-invariant (step07 Part A
            # test 7) and ego's KF state / box_velocity are already
            # naturally global-frame quantities.
            nearest_ego = nearest_point_on_footprint(corners_ego)
            ref_point_global = ego_to_global(
                np.array([[nearest_ego[0], nearest_ego[1], 0.0]]), ctx
            )[0][:2]

            raw_v = nusc.box_velocity(ann_token)
            vel_valid = bool(np.all(np.isfinite(raw_v[:2])))
            obj_v_xy = raw_v[:2] if vel_valid else np.array([0.0, 0.0])

            result = ttc_from_state(ref_point_global, obj_v_xy, ego_xy_global, ego_v_xy_global, ttc_cfg)
            gt_action = gt_action_from_ttc(result.ttc_s) if vel_valid else "NONE"
            severities_this_keyframe.append(_ACTION_SEVERITY[gt_action])

            attrs = [nusc.get("attribute", t)["name"] for t in ann["attribute_tokens"]]
            visibility = int(ann["visibility_token"])
            obs_lidar = ann["num_lidar_pts"] >= config.MIN_LIDAR_PTS.value
            obs_radar = ann["num_radar_pts"] >= config.MIN_RADAR_PTS.value
            obs_camera = visibility >= config.MIN_VISIBILITY_LEVEL.value

            gt_rows.append(
                GTEventRow(
                    scene_token=scene_token,
                    scene_name=scene["name"],
                    sample_token=sample_token,
                    t_us=t_us,
                    instance_token=ann["instance_token"],
                    category=category,
                    attributes=";".join(attrs),
                    visibility_level=visibility,
                    num_lidar_pts=int(ann["num_lidar_pts"]),
                    num_radar_pts=int(ann["num_radar_pts"]),
                    distance_m=result.distance_m,
                    closing_speed_mps=result.closing_speed_mps,
                    gt_ttc_s=result.ttc_s,
                    gt_action=gt_action,
                    vel_valid=vel_valid,
                    obs_lidar=obs_lidar,
                    obs_radar=obs_radar,
                    obs_camera=obs_camera,
                    obs_any=(obs_lidar or obs_radar or obs_camera),
                )
            )

        keyframe_action = _SEVERITY_TO_ACTION[max(severities_this_keyframe, default=0)]
        keyframe_rows.append(
            KeyframeRow(
                scene_token=scene_token,
                scene_name=scene["name"],
                sample_token=sample_token,
                t_us=t_us,
                n_in_path=n_in_path,
                keyframe_action=keyframe_action,
                is_empty_corridor=(n_in_path == 0),
            )
        )

        sample_token = sample["next"]

    return gt_rows, keyframe_rows


def build_all_gt_events(nusc: NuScenes):
    """Runs build_scene_gt_events over every scene. Returns
    (all_gt_rows, all_keyframe_rows)."""
    all_gt_rows: list[GTEventRow] = []
    all_keyframe_rows: list[KeyframeRow] = []
    for scene in nusc.scene:
        gt_rows, keyframe_rows = build_scene_gt_events(nusc, scene["token"])
        all_gt_rows.extend(gt_rows)
        all_keyframe_rows.extend(keyframe_rows)
    return all_gt_rows, all_keyframe_rows
