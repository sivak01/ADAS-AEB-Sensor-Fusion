"""
scripts/lidar_adapter_evidence.py — STEP 04a §6 test 7: real-data BEV
overlay of adapter detections vs. GT boxes, on a few keyframes.
Informational (by-eye recall/false-detection check), not a pass/fail test.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.adapters.lidar import process_lidar_sweep  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.types import RawEvent  # noqa: E402

OUT_DIR = Path("outputs/figs/step04a")


def _event_from_lidar_sd(nusc, sd, scene_token) -> RawEvent:
    return RawEvent(
        t_us=int(sd["timestamp"]), channel="LIDAR_TOP", modality="lidar",
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

    # Pick 3 keyframes from the tune split's busiest-looking scene
    # (scene-1094, from stepD0's per-scene table: 35 in-path GT objects,
    # the most of any tune-split scene) for a varied check. MUST be a
    # tune-split scene (configs/split.json) -- step04a Do NOT #6 forbids
    # using the eval split here. An earlier version of this script used
    # scene-1077, which is actually in the EVAL split -- caught and fixed
    # before this step's report was written; see stepXX_report.md.
    scene = next(s for s in nusc.scene if s["name"] == "scene-1094")
    sample_token = scene["first_sample_token"]
    sample_tokens = []
    while sample_token and len(sample_tokens) < 40:
        sample_tokens.append(sample_token)
        sample_token = nusc.get("sample", sample_token)["next"]
    chosen = [sample_tokens[i] for i in (5, 15, 25)]

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    gt_categories = set(config.GT_CATEGORIES.value)

    for sample_token in chosen:
        sample = nusc.get("sample", sample_token)
        sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        event = _event_from_lidar_sd(nusc, sd, scene["token"])
        ctx = FrameContext.from_event(nusc, event)

        result = process_lidar_sweep(nusc, event)
        print(
            f"{sample_token[:8]}: roi_pts={result.n_roi_points} ground={result.n_ground_points} "
            f"obstacle={result.n_obstacle_points} clusters={result.n_clusters} "
            f"detections={len(result.detections)}"
        )

        # In-path GT objects at this keyframe (candidate-gate membership,
        # same generous position-only test the adapter itself is scored
        # against by step05 downstream).
        gt_in_path = 0
        gt_points_ego = []
        for ann_token in sample["anns"]:
            ann = nusc.get("sample_annotation", ann_token)
            if ann["category_name"] not in gt_categories:
                continue
            gt_global = np.array([ann["translation"][:2]])
            gt_ego = global_to_ego(np.hstack([gt_global, [[0.0]]]), ctx)[0][:2]
            if candidate_gate(gt_ego, config):
                gt_in_path += 1
            gt_points_ego.append((gt_ego, ann["category_name"]))

        det_points_ego = [
            global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2]
            for d in result.detections
        ]

        print(f"  in-path GT objects: {gt_in_path}, adapter detections: {len(result.detections)}")

        fig, ax = plt.subplots(figsize=(6, 8))
        for gt_xy, cat in gt_points_ego:
            ax.scatter(*gt_xy, marker="s", s=60, facecolors="none", edgecolors="green", label="GT")
        for det_xy in det_points_ego:
            ax.scatter(*det_xy, marker="x", s=60, c="red", label="detection")
        ax.scatter(0, 0, marker="^", c="black", s=100, label="ego")
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())
        ax.set_xlabel("x, ego forward (m)")
        ax.set_ylabel("y, ego left (m)")
        ax.set_xlim(-5, 60)
        ax.set_ylim(-15, 15)
        ax.set_aspect("equal")
        ax.set_title(f"{scene['name']} @ {sample_token[:8]}: LiDAR detections vs GT (BEV)")
        fig.tight_layout()
        fig.savefig(OUT_DIR / f"bev_{sample_token[:8]}.png", dpi=130)
        plt.close(fig)


if __name__ == "__main__":
    main()
