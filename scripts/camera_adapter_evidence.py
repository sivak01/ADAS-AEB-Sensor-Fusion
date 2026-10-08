"""
scripts/camera_adapter_evidence.py — STEP 04c §5 test 8 / §6: real-data BEV
overlay of adapter detections vs. GT boxes, image overlay of projected
reference points, and a depth-error-by-range-band table. Informational
(by-eye recall/false-detection + error characterization), not pass/fail.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.adapters.camera import process_camera_frame  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.types import RawEvent  # noqa: E402

OUT_DIR = Path("outputs/figs/step04c")
SCENE_NAMES = ["scene-0103", "scene-0061", "scene-1094"]  # tune split, same scenes step04b used
RANGE_BANDS_M = [(0, 15), (15, 30), (30, 50), (50, 200)]


def _event_from_sd(sd, channel, modality, scene_token) -> RawEvent:
    return RawEvent(
        t_us=int(sd["timestamp"]), channel=channel, modality=modality,
        sample_data_token=sd["token"], ego_pose_token=sd["ego_pose_token"],
        calibrated_sensor_token=sd["calibrated_sensor_token"], filename=sd["filename"],
        is_key_frame=bool(sd["is_key_frame"]), scene_token=scene_token,
    )


def main():
    from nuscenes.nuscenes import NuScenes
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    gt_categories = set(config.GT_CATEGORIES.value)

    # ── Depth-error-by-range-band table (nearest-GT match, informational) ──
    band_errors = {band: [] for band in RANGE_BANDS_M}
    n_frames_checked = 0

    for scene_name in SCENE_NAMES:
        scene = next(s for s in nusc.scene if s["name"] == scene_name)
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = nusc.get("sample", sample_token)
            cam_sd = nusc.get("sample_data", sample["data"]["CAM_FRONT"])
            cam_event = _event_from_sd(cam_sd, "CAM_FRONT", "camera", scene["token"])
            try:
                result = process_camera_frame(nusc, cam_event)
            except FileNotFoundError:
                sample_token = sample["next"]
                continue
            n_frames_checked += 1

            lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
            gt_ctx = FrameContext.from_event(
                nusc, _event_from_sd(lidar_sd, "LIDAR_TOP", "lidar", scene["token"])
            )
            gt_positions_ego = []
            for ann_token in sample["anns"]:
                ann = nusc.get("sample_annotation", ann_token)
                if ann["category_name"] not in gt_categories:
                    continue
                g = np.array([ann["translation"][:2]])
                gt_positions_ego.append(global_to_ego(np.hstack([g, [[0.0]]]), gt_ctx)[0][:2])

            cam_ctx = FrameContext.from_event(nusc, cam_event)
            for d in result.detections:
                det_ego = global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), cam_ctx)[0][:2]
                if not gt_positions_ego:
                    continue
                dists = [float(np.linalg.norm(det_ego - g)) for g in gt_positions_ego]
                nearest_err = min(dists)
                range_m = d.aux["range_m"]
                for lo, hi in RANGE_BANDS_M:
                    if lo <= range_m < hi:
                        band_errors[(lo, hi)].append(nearest_err)
                        break

            sample_token = sample["next"]

    print(f"Depth-error-by-range-band (nearest-GT-object error, {n_frames_checked} frames checked, "
          f"scenes={SCENE_NAMES}):")
    for band, errs in band_errors.items():
        if errs:
            errs_arr = np.array(errs)
            print(f"  {band[0]:3d}-{band[1]:3d}m: n={len(errs):4d}  mean_err={errs_arr.mean():5.2f}m  "
                  f"median_err={np.median(errs_arr):5.2f}m")
        else:
            print(f"  {band[0]:3d}-{band[1]:3d}m: n=0 (no detections in this band)")

    # ── BEV figures: one representative keyframe per scene ─────────────────
    for scene_name in SCENE_NAMES:
        scene = next(s for s in nusc.scene if s["name"] == scene_name)
        sample_tokens = []
        sample_token = scene["first_sample_token"]
        while sample_token and len(sample_tokens) < 40:
            sample_tokens.append(sample_token)
            sample_token = nusc.get("sample", sample_token)["next"]
        chosen = sample_tokens[len(sample_tokens) // 2]
        sample = nusc.get("sample", chosen)

        cam_sd = nusc.get("sample_data", sample["data"]["CAM_FRONT"])
        cam_event = _event_from_sd(cam_sd, "CAM_FRONT", "camera", scene["token"])
        try:
            result = process_camera_frame(nusc, cam_event)
        except FileNotFoundError:
            print(f"  (skipping BEV figure for {scene_name} @ {chosen[:8]}: not cached)")
            continue
        cam_ctx = FrameContext.from_event(nusc, cam_event)

        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        gt_ctx = FrameContext.from_event(
            nusc, _event_from_sd(lidar_sd, "LIDAR_TOP", "lidar", scene["token"])
        )
        gt_points_ego = []
        for ann_token in sample["anns"]:
            ann = nusc.get("sample_annotation", ann_token)
            if ann["category_name"] not in gt_categories:
                continue
            g = np.array([ann["translation"][:2]])
            gt_points_ego.append(global_to_ego(np.hstack([g, [[0.0]]]), gt_ctx)[0][:2])

        det_points_ego = [
            global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), cam_ctx)[0][:2]
            for d in result.detections
        ]
        in_path_flags = [candidate_gate(p, config) for p in det_points_ego]
        n_in_path = sum(in_path_flags)
        print(f"{scene_name} @ {chosen[:8]}: raw_boxes={result.n_raw_boxes} "
              f"class_filtered_out={result.n_class_filtered} truncated_dropped={result.n_truncated_dropped} "
              f"ground_fail={result.n_ground_intersection_failed} detections={len(result.detections)} "
              f"in_path_candidates={n_in_path} gt_objects={len(gt_points_ego)}")

        fig, ax = plt.subplots(figsize=(6, 8))
        for gt_xy in gt_points_ego:
            ax.scatter(*gt_xy, marker="s", s=60, facecolors="none", edgecolors="green", label="GT")
        for det_xy, in_path in zip(det_points_ego, in_path_flags):
            ax.scatter(*det_xy, marker="x", s=50, c=("red" if in_path else "gray"),
                       label="detection (in-path candidate)" if in_path else "detection (out of candidate gate)")
        ax.scatter(0, 0, marker="^", c="black", s=100, label="ego")
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), fontsize=8)
        ax.set_xlabel("x, ego forward (m)")
        ax.set_ylabel("y, ego left (m)")
        ax.set_xlim(-10, 90)
        ax.set_ylim(-40, 40)
        ax.set_aspect("equal")
        ax.set_title(f"{scene_name} @ {chosen[:8]}: camera detections vs GT (BEV)")
        fig.tight_layout()
        fig.savefig(OUT_DIR / f"bev_{scene_name}_{chosen[:8]}.png", dpi=130)
        plt.close(fig)


if __name__ == "__main__":
    main()
