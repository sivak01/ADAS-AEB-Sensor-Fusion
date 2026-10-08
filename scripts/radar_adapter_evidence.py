"""
scripts/radar_adapter_evidence.py — STEP 04b §5 test 6: real-data BEV
overlay of adapter detections vs. GT boxes, plus raw-points-per-object vs
clusters-per-object statistics, on a few TUNE-split scenes. Informational
(by-eye recall/false-detection check), not a pass/fail test.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.adapters.radar import process_radar_scan  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.types import RawEvent  # noqa: E402

OUT_DIR = Path("outputs/figs/step04b")
CHANNELS = ["RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT"]
# Scenes chosen from the TUNE split (configs/split.json / DEC-8) -- step04a
# §2's own recorded mistake (using an eval-split scene) makes this worth
# stating explicitly rather than assuming.
SCENE_NAMES = ["scene-0103", "scene-0061", "scene-1094"]
ASSOC_RADIUS_M = 5.0  # generous: associates a cluster to a GT object for the raw-vs-cluster stat only


def _event_from_sd(sd, channel, scene_token) -> RawEvent:
    return RawEvent(
        t_us=int(sd["timestamp"]), channel=channel, modality="radar",
        sample_data_token=sd["token"], ego_pose_token=sd["ego_pose_token"],
        calibrated_sensor_token=sd["calibrated_sensor_token"], filename=sd["filename"],
        is_key_frame=bool(sd["is_key_frame"]), scene_token=scene_token,
    )


def _scan_all_channels(nusc, sample, scene_token):
    """Returns (total_raw_points, all_detections_with_ego_ctx) pooled
    across the 3 DEC-3 radar channels for one keyframe."""
    total_raw = 0
    dets_with_ctx = []
    for channel in CHANNELS:
        sd = nusc.get("sample_data", sample["data"][channel])
        event = _event_from_sd(sd, channel, scene_token)
        ctx = FrameContext.from_event(nusc, event)
        result = process_radar_scan(nusc, event)
        total_raw += result.n_points
        for d in result.detections:
            dets_with_ctx.append((d, ctx))
    return total_raw, dets_with_ctx


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

    # ── Raw-points-per-object vs clusters-per-object, across all keyframes
    # of the 3 scenes (uses the GT's own devkit-reported num_radar_pts,
    # rather than re-deriving a raw-point/object association ourselves --
    # nuScenes already provides that exact per-annotation number).
    per_object_raw = []
    per_object_clusters = []
    per_keyframe_totals = []  # (total_raw_points, n_clusters) for false-detection context

    for scene_name in SCENE_NAMES:
        scene = next(s for s in nusc.scene if s["name"] == scene_name)
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = nusc.get("sample", sample_token)
            total_raw, dets_with_ctx = _scan_all_channels(nusc, sample, scene["token"])
            per_keyframe_totals.append((total_raw, len(dets_with_ctx)))

            lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
            gt_ctx = FrameContext.from_event(
                nusc, RawEvent(
                    t_us=int(lidar_sd["timestamp"]), channel="LIDAR_TOP", modality="lidar",
                    sample_data_token=lidar_sd["token"], ego_pose_token=lidar_sd["ego_pose_token"],
                    calibrated_sensor_token=lidar_sd["calibrated_sensor_token"], filename=lidar_sd["filename"],
                    is_key_frame=True, scene_token=scene["token"],
                )
            )
            det_positions_ego = [
                global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2]
                for d, ctx in dets_with_ctx
            ]

            for ann_token in sample["anns"]:
                ann = nusc.get("sample_annotation", ann_token)
                if ann["category_name"] not in gt_categories or ann["num_radar_pts"] == 0:
                    continue
                gt_global = np.array([ann["translation"][:2]])
                gt_ego = global_to_ego(np.hstack([gt_global, [[0.0]]]), gt_ctx)[0][:2]
                n_nearby_clusters = sum(
                    1 for p in det_positions_ego if np.linalg.norm(p - gt_ego) <= ASSOC_RADIUS_M
                )
                per_object_raw.append(ann["num_radar_pts"])
                per_object_clusters.append(n_nearby_clusters)

            sample_token = sample["next"]

    per_object_raw = np.array(per_object_raw)
    per_object_clusters = np.array(per_object_clusters)
    print(f"Raw-points-per-object vs clusters-per-object (within {ASSOC_RADIUS_M}m), "
          f"{len(per_object_raw)} GT objects with num_radar_pts>0, scenes={SCENE_NAMES}:")
    print(f"  mean num_radar_pts (devkit-reported): {per_object_raw.mean():.2f}")
    print(f"  mean nearby clusters (this adapter): {per_object_clusters.mean():.2f}")
    print(f"  objects with 0 nearby clusters (missed): {(per_object_clusters == 0).sum()}/{len(per_object_clusters)}")
    print(f"  objects with >1 nearby cluster (fragmented): {(per_object_clusters > 1).sum()}/{len(per_object_clusters)}")

    totals = np.array(per_keyframe_totals)
    print(f"\nPer-keyframe totals (3-channel pooled, {len(totals)} keyframes): "
          f"mean raw points={totals[:, 0].mean():.1f}, mean clusters={totals[:, 1].mean():.1f}")

    # ── BEV figure: one representative keyframe per scene.
    for scene_name in SCENE_NAMES:
        scene = next(s for s in nusc.scene if s["name"] == scene_name)
        sample_tokens = []
        sample_token = scene["first_sample_token"]
        while sample_token and len(sample_tokens) < 40:
            sample_tokens.append(sample_token)
            sample_token = nusc.get("sample", sample_token)["next"]
        chosen = sample_tokens[len(sample_tokens) // 2]  # a mid-scene keyframe
        sample = nusc.get("sample", chosen)

        total_raw, dets_with_ctx = _scan_all_channels(nusc, sample, scene["token"])
        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        gt_ctx = FrameContext.from_event(
            nusc, RawEvent(
                t_us=int(lidar_sd["timestamp"]), channel="LIDAR_TOP", modality="lidar",
                sample_data_token=lidar_sd["token"], ego_pose_token=lidar_sd["ego_pose_token"],
                calibrated_sensor_token=lidar_sd["calibrated_sensor_token"], filename=lidar_sd["filename"],
                is_key_frame=True, scene_token=scene["token"],
            )
        )

        gt_points_ego = []
        for ann_token in sample["anns"]:
            ann = nusc.get("sample_annotation", ann_token)
            if ann["category_name"] not in gt_categories:
                continue
            gt_global = np.array([ann["translation"][:2]])
            gt_points_ego.append(global_to_ego(np.hstack([gt_global, [[0.0]]]), gt_ctx)[0][:2])

        det_points_ego = [
            global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2]
            for d, ctx in dets_with_ctx
        ]
        in_path_flags = [candidate_gate(p, config) for p in det_points_ego]
        n_in_path = sum(in_path_flags)
        print(f"{scene_name} @ {chosen[:8]}: raw_points(3ch)={total_raw} clusters={len(dets_with_ctx)} "
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
        ax.set_xlim(-30, 90)
        ax.set_ylim(-40, 40)
        ax.set_aspect("equal")
        ax.set_title(f"{scene_name} @ {chosen[:8]}: radar detections vs GT (BEV)")
        fig.tight_layout()
        fig.savefig(OUT_DIR / f"bev_{scene_name}_{chosen[:8]}.png", dpi=130)
        plt.close(fig)


if __name__ == "__main__":
    main()
