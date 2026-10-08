"""
scripts/measure_sensor_R.py — STEP 04a §5 / STEP 04b §6: measure a
sensor's own R (sigma_long, sigma_lat) empirically from GT residuals,
TUNE SPLIT ONLY. `--sensor lidar` (default) or `--sensor radar`.

For every keyframe, every in-path GT object (DEC-1/DEC-2 scope) with a
"high" point count for that sensor (num_lidar_pts / num_radar_pts) is
matched to its NEAREST adapter detection (by position, ego frame, at
that keyframe) — this is a coarse nearest-neighbour match, not real data
association (the tracker doesn't exist yet at this stage of the build
order), so an occasional residual may be against the wrong real-world
object if two candidates sit close together. This is a known
limitation, stated here and in each step's report, not hidden.

Residual = nearest_detection_ego - gt_reference_ego, decomposed into
(longitudinal, lateral) since the ego frame already has that axis
alignment (x forward, y left) — no extra rotation needed.

Radar-specific note: a keyframe's 3 radar channels (DEC-3) each have
their own sample_data entry and own pose at that same nominal instant —
detections from all 3 are pooled per keyframe (any of the 3 could see a
given GT object), each converted to ego frame via ITS OWN channel's
FrameContext (never borrowed, G2), while the GT object's own ego-frame
reference point still uses LIDAR_TOP's pose (this project's established
GT-instant convention, stepD0 §3.1). The resulting few-cm pose mismatch
between channels within one nominal keyframe is a stated approximation,
not treated as a hidden source of error.

Per step04a §5 / step04b §6: label R MEASURED only if the inlier rate
clears MEASURED_INLIER_RATE_MIN, and even then, NEVER silently swap the
value into config.py — this script only reports; a human decision +
explicit approval is required to promote a SENSOR_R_* status.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.geometry.corridor import candidate_gate, footprint_intersects_corridor, nearest_point_on_footprint  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.gt.gt_events import _footprint_corners_global  # noqa: E402
from ttcf.types import RawEvent  # noqa: E402

INLIER_GATE_M = 2.0  # step04a §5's own example gate
MEASURED_INLIER_RATE_MIN = 0.70  # step04a §5's own example threshold


def _event_from_sd(sd, channel: str, modality: str, scene_token: str) -> RawEvent:
    return RawEvent(
        t_us=int(sd["timestamp"]), channel=channel, modality=modality,
        sample_data_token=sd["token"], ego_pose_token=sd["ego_pose_token"],
        calibrated_sensor_token=sd["calibrated_sensor_token"], filename=sd["filename"],
        is_key_frame=bool(sd["is_key_frame"]), scene_token=scene_token,
    )


def _mad_sigma(x: np.ndarray) -> float:
    med = np.median(x)
    return float(1.4826 * np.median(np.abs(x - med)))


# ── Sensor-specific plumbing ───────────────────────────────────────────────

def _lidar_detections_ego(nusc, sample, scene_token) -> tuple[list, FrameContext]:
    from ttcf.adapters.lidar import process_lidar_sweep

    sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    event = _event_from_sd(sd, "LIDAR_TOP", "lidar", scene_token)
    ctx = FrameContext.from_event(nusc, event)
    result = process_lidar_sweep(nusc, event)
    det_ego = [
        global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2] for d in result.detections
    ]
    # candidate_gate, same filter run_pipeline.py applies before the
    # tracker (step05) -- without it, LiDAR's own ROI crop already keeps
    # this list small, but applying the SAME filter as radar keeps the
    # two sensors' R-measurement methodology consistent.
    det_ego = [d for d in det_ego if candidate_gate(d, config)]
    return det_ego, ctx


def _radar_detections_ego(nusc, sample, scene_token) -> tuple[list, FrameContext]:
    from ttcf.adapters.radar import radar_detections

    channels = ["RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT"]
    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    gt_ctx = FrameContext.from_event(  # GT reference point still uses LIDAR_TOP's pose (see module docstring)
        nusc, _event_from_sd(lidar_sd, "LIDAR_TOP", "lidar", scene_token)
    )

    det_ego = []
    for channel in channels:
        sd = nusc.get("sample_data", sample["data"][channel])
        event = _event_from_sd(sd, channel, "radar", scene_token)
        ctx = FrameContext.from_event(nusc, event)
        for d in radar_detections(nusc, event):
            det_ego.append(global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2])
    # candidate_gate BEFORE nearest-neighbour matching (G13 finding,
    # step04b): without this, a keyframe's pooled 3-channel radar output
    # (no ROI crop, ghost returns not filtered -- both deliberate, step04b
    # decision 2026-09-23) produced 40-136 candidate detections per
    # keyframe, and "nearest of 100+ candidates" is close to a GT object
    # by sheer density, not real accuracy -- a genuine bias in the R
    # measurement, not a real signal. Restricting to candidate_gate-
    # passing detections matches what the real pipeline (run_pipeline.py)
    # already does before the tracker, for a fair comparison.
    det_ego = [d for d in det_ego if candidate_gate(d, config)]
    return det_ego, gt_ctx


def _camera_detections_ego(nusc, sample, scene_token) -> tuple[list, FrameContext]:
    from ttcf.adapters.camera import camera_detections

    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    gt_ctx = FrameContext.from_event(  # GT reference point still uses LIDAR_TOP's pose (see module docstring)
        nusc, _event_from_sd(lidar_sd, "LIDAR_TOP", "lidar", scene_token)
    )
    sd = nusc.get("sample_data", sample["data"]["CAM_FRONT"])
    event = _event_from_sd(sd, "CAM_FRONT", "camera", scene_token)
    ctx = FrameContext.from_event(nusc, event)
    dets = camera_detections(nusc, event)  # reads the on-disk cache; raises if not cached yet
    det_ego = [
        global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2] for d in dets
    ]
    det_ego = [d for d in det_ego if candidate_gate(d, config)]
    return det_ego, gt_ctx


_SENSOR_CONFIG = {
    "lidar": {"gt_field": "num_lidar_pts", "min_pts": 50, "detections_fn": _lidar_detections_ego,
              "config_param": "SENSOR_R_LIDAR"},
    # Radar returns are far sparser than LiDAR's (tune-split in-path GT
    # median num_radar_pts is 0; >=5 leaves only 3 samples). >=3 keeps a
    # defensible "well-observed by radar" sample (25 rows) matching
    # radar's own known sparsity, rather than reusing LiDAR's >=50 bar.
    "radar": {"gt_field": "num_radar_pts", "min_pts": 3, "detections_fn": _radar_detections_ego,
              "config_param": "SENSOR_R_RADAR"},
    # Camera has no point-count field -- nuScenes' own visibility_level
    # (1-4) is the closest equivalent to "well-observed"; the GT table
    # itself already uses level>=2 for "observable at all" (obs_camera,
    # MIN_VISIBILITY_LEVEL) -- this measurement uses the stricter level==4
    # (80-100% visible) for a genuinely "well-observed" sample.
    "camera": {"gt_field": "visibility_token", "min_pts": 4, "detections_fn": _camera_detections_ego,
               "config_param": "SENSOR_R_CAMERA"},
}


def _gt_field_value(ann: dict, field: str) -> int:
    """visibility_token is a string ("1".."4", per gt_events.py's own
    already-tested handling of this exact field) -- every other gt_field
    (num_lidar_pts/num_radar_pts) is already a plain int."""
    if field == "visibility_token":
        return int(ann["visibility_token"])
    return ann[field]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sensor", choices=["lidar", "radar", "camera"], default="lidar")
    args = parser.parse_args()
    spec = _SENSOR_CONFIG[args.sensor]

    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    gt_categories = set(config.GT_CATEGORIES.value)
    half_width = config.CORRIDOR_HALF_WIDTH_M.value
    max_range = config.CORRIDOR_MAX_RANGE_M.value

    residuals_long: list[float] = []
    residuals_lat: list[float] = []
    n_candidates = 0
    n_no_detection_at_all = 0

    for scene_entry in split["tune"]:
        scene = nusc.get("scene", scene_entry["token"])
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = nusc.get("sample", sample_token)

            high_pt_gt_refs = []
            gt_ctx_for_corridor = None
            for ann_token in sample["anns"]:
                ann = nusc.get("sample_annotation", ann_token)
                if ann["category_name"] not in gt_categories:
                    continue
                if _gt_field_value(ann, spec["gt_field"]) < spec["min_pts"]:
                    continue
                if gt_ctx_for_corridor is None:
                    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
                    gt_ctx_for_corridor = FrameContext.from_event(
                        nusc, _event_from_sd(lidar_sd, "LIDAR_TOP", "lidar", scene["token"])
                    )
                corners_global = _footprint_corners_global(ann)
                corners_ego = global_to_ego(corners_global, gt_ctx_for_corridor)[:, :2]
                if not footprint_intersects_corridor(corners_ego, half_width, max_range):
                    continue  # scope to objects the adapter's ROI is designed to see
                high_pt_gt_refs.append(nearest_point_on_footprint(corners_ego))

            if high_pt_gt_refs:
                det_ego, _ = spec["detections_fn"](nusc, sample, scene["token"])
                for gt_ref_ego in high_pt_gt_refs:
                    n_candidates += 1
                    if not det_ego:
                        n_no_detection_at_all += 1
                        continue  # a total miss: no residual to record, counts against inlier rate below
                    dists = [float(np.linalg.norm(d - gt_ref_ego)) for d in det_ego]
                    nearest = det_ego[int(np.argmin(dists))]
                    residual = nearest - gt_ref_ego
                    residuals_long.append(float(residual[0]))
                    residuals_lat.append(float(residual[1]))

            sample_token = sample["next"]

    residuals_long_arr = np.array(residuals_long)
    residuals_lat_arr = np.array(residuals_lat)
    residual_norm = np.hypot(residuals_long_arr, residuals_lat_arr)
    inlier_mask = residual_norm < INLIER_GATE_M
    n_inlier = int(inlier_mask.sum())
    # Denominator is every candidate attempted (n_candidates), including
    # total misses (n_no_detection_at_all) -- a miss is the worst outcome
    # and must not be silently dropped from the rate.
    inlier_rate = n_inlier / n_candidates if n_candidates else 0.0

    sigma_long = _mad_sigma(residuals_long_arr[inlier_mask]) if n_inlier else float("nan")
    sigma_lat = _mad_sigma(residuals_lat_arr[inlier_mask]) if n_inlier else float("nan")

    param_name = spec["config_param"]
    print(f"Sensor: {args.sensor} ({param_name})")
    print(f"High-{spec['gt_field']} (>= {spec['min_pts']}) in-path GT candidates (tune split): {n_candidates}")
    print(f"  of which total misses (no detection at all that keyframe): {n_no_detection_at_all}")
    print(f"  of which matched to SOME detection: {len(residual_norm)}")
    print(f"Inlier rate (residual < {INLIER_GATE_M} m, out of all {n_candidates} candidates): "
          f"{inlier_rate:.1%} ({n_inlier}/{n_candidates})")
    print(f"Robust sigma_long (MAD, inliers only, n={n_inlier}): {sigma_long:.3f} m")
    print(f"Robust sigma_lat  (MAD, inliers only, n={n_inlier}): {sigma_lat:.3f} m")
    if inlier_rate > MEASURED_INLIER_RATE_MIN:
        print(f"\n-> Inlier rate exceeds {MEASURED_INLIER_RATE_MIN:.0%}: eligible to label {param_name} MEASURED.")
        print("   NOT applied automatically (never silently swap in a measured value) -- "
              "report + explicit approval required before config.py is changed.")
    else:
        print(f"\n-> Inlier rate does NOT exceed {MEASURED_INLIER_RATE_MIN:.0%}: {param_name} stays ASSUMED.")


if __name__ == "__main__":
    main()
