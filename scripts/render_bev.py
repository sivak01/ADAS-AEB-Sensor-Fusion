"""
scripts/render_bev.py — STEP 12 Part A: static per-GT-keyframe BEV PNGs.

Runs the FUSED stream once per scene (Siva's 2026-10-08 scope choice —
fused only; see step12_bev_visualization.md §3) and renders one BEV panel
per GT keyframe: raw (pre-candidate-gate) detections, the candidate-gate
corridor, GT boxes (DEC-2's own vehicle/pedestrian filter, reused —
bev.gt_category_allowed), and the live tracker's own tracks colored by
TTC urgency (DEC-13). Drawing math itself lives in src/ttcf/viz/bev.py
and is unit-tested there (tests/test_bev.py); this script only wires real
data through it.

Part B (--video): a per-scene animated GIF at NATIVE rate -- every real
merged event becomes one frame (not just GT keyframes), approved by Siva
after reviewing Part A's PNGs (2026-10-08, "ok"). GIF, not MP4: no
ffmpeg binary is installed in this environment, and installing one is a
new dependency (README_MASTER.md §2: "ask before installing anything
heavy") for a cosmetic choice that doesn't change anything the step
actually asked for. PIL (already a dependency, via matplotlib) writes
animated GIFs natively, and -- unlike a fixed-fps video -- a GIF's own
per-frame `duration` list lets each frame be held for exactly its own
real inter-event gap, which is a MORE literal reading of "fps measured
from real deltas" (the one V1 BEV idea step12 §1 explicitly keeps) than
a single averaged fps number for the whole clip would be.

Between GT keyframes (most native-rate frames), there is no real GT to
show -- GT boxes are only ever drawn on the exact event that IS a
keyframe's own LIDAR_TOP sample_data (same t_us as a real
`sample.timestamp`, nuScenes' own guarantee), never interpolated or held
over from the last keyframe, so the video never implies GT precision the
dataset doesn't actually provide.

Reuses extrapolate.py's own private `_extrapolate_snapshot` (the
throwaway-KF-forward helper) and gt_events.py's own private
`_footprint_corners_global` rather than reimplementing either — same
precedent as radar.py importing lidar.py's `_cluster_reference_point`.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ttcf import config  # noqa: E402
from ttcf.data.event_stream import merge_channel_events, scene_channel_events  # noqa: E402
from ttcf.evaluation.extrapolate import _extrapolate_snapshot  # noqa: E402
from ttcf.filtering.ego_state import EgoStateEstimator  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego, global_to_ego_vector  # noqa: E402
from ttcf.gt.gt_events import _footprint_corners_global, build_scene_gt_events  # noqa: E402
from ttcf.ttc.estimator import TTCEstimator  # noqa: E402
from ttcf.tracking.tracker import ShortMemoryTracker  # noqa: E402
from ttcf.viz import bev  # noqa: E402
from run_pipeline import _channel_adapter_dispatch  # noqa: E402

# global_to_ego/global_to_ego_vector only ever read ctx.ego_pose -- GT
# annotations and the tracker's own global-frame state need no real
# sensor calibration, so this identity stands in for the unused field
# (same pattern gt/gt_events.py's own _IDENTITY_CS already uses).
_IDENTITY_CS = {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]}


def _collect_scene_run(nusc, scene_token: str):
    """Runs the fused stream once, recording BOTH what the real tracker
    needs (candidate-gate-filtered detections -> process_scan, exactly
    run_pipeline.run_real_stream's own loop) AND the RAW, pre-gate
    detections per event for BEV display (step12 §3 explicitly wants
    raw detections shown, which run_real_stream doesn't expose)."""
    dispatch = _channel_adapter_dispatch("fused")
    channels = list(dispatch.keys())

    ego_est = EgoStateEstimator(nusc, scene_token)
    tracker = ShortMemoryTracker(sigma_a=config.SIGMA_A.value)
    snapshot_histories: dict = defaultdict(list)
    detection_log = []  # [(t_us, channel, list[Detection])], time order

    per_channel_events = [scene_channel_events(nusc, scene_token, ch) for ch in channels]
    for event in merge_channel_events(*per_channel_events):
        raw_dets = dispatch[event.channel](nusc, event, cfg=config)
        detection_log.append((event.t_us, event.channel, raw_dets))

        ctx = FrameContext.from_event(nusc, event)
        dets = [
            d for d in raw_dets
            if candidate_gate(global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2])
        ]
        snapshots = tracker.process_scan(dets, event.t_us)
        for snap in snapshots:
            snapshot_histories[snap.track_id].append(snap)

    return ego_est, dict(snapshot_histories), detection_log


def _latest_detections_at_or_before(detection_log, t_us: int) -> dict:
    """Per channel, the most recent RAW scan at or before t_us (causal —
    G15). A channel with no such scan yet is simply absent."""
    latest: dict = {}
    for ev_t_us, channel, dets in detection_log:
        if ev_t_us > t_us:
            continue
        cur = latest.get(channel)
        if cur is None or ev_t_us > cur[0]:
            latest[channel] = (ev_t_us, dets)
    return latest


def _tracks_at_keyframe(snapshot_histories: dict, kf_t_us: int, ego_state, ctx, estimator, cfg=config) -> list:
    """Causally extrapolate every track to kf_t_us -- the same rule
    evaluation/extrapolate.py's own extrapolate_track uses (last snapshot
    at/before kf_t_us; STALE/dropped past EVICTION_GAP_S), reusing its
    exact throwaway-KF helper rather than a second, parallel
    'pos + v*dt' approximation."""
    out = []
    for track_id, history in snapshot_histories.items():
        causal = [s for s in history if s.t_us <= kf_t_us]
        if not causal:
            continue
        snapshot = max(causal, key=lambda s: s.t_us)
        if (kf_t_us - snapshot.t_us) / 1e6 > cfg.EVICTION_GAP_S.value:
            continue
        extrapolated = _extrapolate_snapshot(snapshot, kf_t_us, cfg.SIGMA_A.value)
        ttc_result = estimator.estimate(extrapolated, ego_state)
        xy_ego = global_to_ego(np.array([[extrapolated.x[0], extrapolated.x[1], 0.0]]), ctx)[0][:2]
        v_ego = global_to_ego_vector(np.array([extrapolated.x[2:4]]), ctx)[0]
        out.append({"track_id": track_id, "xy_ego": xy_ego, "vxy_ego": v_ego, "ttc_s": ttc_result.ttc_s})
    return out


def _gt_boxes_at_keyframe(nusc, sample: dict, ctx, cfg=config) -> list:
    """ALL vehicle/pedestrian GT boxes in view (DEC-2's class filter via
    bev.gt_category_allowed) -- not restricted to in-path objects, so the
    panel can visually compare detections/tracks against GT the forward
    corridor itself excludes, too (a sanity check the metrics alone can't give)."""
    boxes = []
    for ann_token in sample["anns"]:
        ann = nusc.get("sample_annotation", ann_token)
        if not bev.gt_category_allowed(ann["category_name"], cfg):
            continue
        corners_global = _footprint_corners_global(ann)
        corners_ego = global_to_ego(corners_global, ctx)[:, :2]
        boxes.append(corners_ego)
    return boxes


def render_scene(nusc, scene_token: str, out_dir: Path, split_label: str) -> int:
    scene = nusc.get("scene", scene_token)
    _, keyframe_rows = build_scene_gt_events(nusc, scene_token)
    ego_est, snapshot_histories, detection_log = _collect_scene_run(nusc, scene_token)
    estimator = TTCEstimator(config)

    scene_out = out_dir / scene_token[:8]
    scene_out.mkdir(parents=True, exist_ok=True)
    eval_tag = " [EVAL]" if split_label == "eval" else ""  # DEC-14

    n_written = 0
    for kf in keyframe_rows:
        sample = nusc.get("sample", kf.sample_token)
        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        ego_pose_raw = nusc.get("ego_pose", lidar_sd["ego_pose_token"])
        ctx = FrameContext(calibrated_sensor=_IDENTITY_CS, ego_pose=ego_pose_raw)

        ego_state = ego_est.state_at(kf.t_us)
        tracks = _tracks_at_keyframe(snapshot_histories, kf.t_us, ego_state, ctx, estimator)
        gt_boxes = _gt_boxes_at_keyframe(nusc, sample, ctx)

        detections_ego = []
        for channel, (_ev_t_us, dets) in _latest_detections_at_or_before(detection_log, kf.t_us).items():
            for d in dets:
                detections_ego.append(global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2])

        fig, ax = plt.subplots(figsize=(7, 9))
        bev.new_bev_axes(ax, title=f"{scene['name']} @ t={kf.t_us}{eval_tag}")
        bev.draw_corridor(ax)
        bev.draw_detections(ax, detections_ego, label="raw detections")
        bev.draw_gt_boxes(ax, gt_boxes)
        bev.draw_tracks(ax, tracks)
        bev.draw_track_legend_entry(ax)
        bev.draw_ego(ax)
        ax.legend(loc="upper right", fontsize=6)

        out_path = scene_out / f"keyframe_{kf.sample_token[:8]}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        n_written += 1

    return n_written


_MIN_FRAME_MS = 30.0  # readability floor
_MAX_FRAME_MS = 500.0  # cap so one long real gap doesn't freeze playback


def render_scene_video(nusc, scene_token: str, out_dir: Path, split_label: str, max_frames=None):
    """Part B: one native-rate GIF per scene (see module docstring for
    why GIF, not MP4, and why frame durations come from real deltas).
    `max_frames` truncates the event stream for a quick test render."""
    dispatch = _channel_adapter_dispatch("fused")
    channels = list(dispatch.keys())
    scene = nusc.get("scene", scene_token)
    eval_tag = " [EVAL]" if split_label == "eval" else ""  # DEC-14

    _, keyframe_rows = build_scene_gt_events(nusc, scene_token)
    gt_boxes_by_t_us = {}
    for kf in keyframe_rows:
        sample = nusc.get("sample", kf.sample_token)
        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        ego_pose_raw = nusc.get("ego_pose", lidar_sd["ego_pose_token"])
        ctx = FrameContext(calibrated_sensor=_IDENTITY_CS, ego_pose=ego_pose_raw)
        gt_boxes_by_t_us[kf.t_us] = _gt_boxes_at_keyframe(nusc, sample, ctx)

    ego_est = EgoStateEstimator(nusc, scene_token)
    tracker = ShortMemoryTracker(sigma_a=config.SIGMA_A.value)
    estimator = TTCEstimator(config)

    per_channel_events = [scene_channel_events(nusc, scene_token, ch) for ch in channels]
    events = list(merge_channel_events(*per_channel_events))
    if max_frames is not None:
        events = events[:max_frames]
    if not events:
        return None, 0

    deltas_ms = [(events[i + 1].t_us - events[i].t_us) / 1000.0 for i in range(len(events) - 1)]
    median_ms = float(np.median(deltas_ms)) if deltas_ms else _MIN_FRAME_MS

    fig, ax = plt.subplots(figsize=(7, 9))
    pil_frames: list = []
    durations_ms: list = []

    for i, event in enumerate(events):
        raw_dets = dispatch[event.channel](nusc, event, cfg=config)
        ctx = FrameContext.from_event(nusc, event)
        dets = [
            d for d in raw_dets
            if candidate_gate(global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2])
        ]
        snapshots = tracker.process_scan(dets, event.t_us)

        ego_state = ego_est.state_at(event.t_us)
        tracks = []
        for snap in snapshots:
            ttc_result = estimator.estimate(snap, ego_state)
            xy_ego = global_to_ego(np.array([[snap.x[0], snap.x[1], 0.0]]), ctx)[0][:2]
            v_ego = global_to_ego_vector(np.array([snap.x[2:4]]), ctx)[0]
            tracks.append({"track_id": snap.track_id, "xy_ego": xy_ego, "vxy_ego": v_ego, "ttc_s": ttc_result.ttc_s})

        detections_ego = [
            global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2] for d in raw_dets
        ]
        # Only on the exact keyframe event -- never interpolated/held over
        # (module docstring: no false GT precision between real samples).
        gt_boxes = gt_boxes_by_t_us.get(event.t_us, [])

        ax.clear()
        bev.new_bev_axes(ax, title=f"{scene['name']} @ t={event.t_us}{eval_tag} ({event.channel})")
        bev.draw_corridor(ax)
        bev.draw_detections(ax, detections_ego, label="raw detections")
        if gt_boxes:
            bev.draw_gt_boxes(ax, gt_boxes)
        bev.draw_tracks(ax, tracks)
        bev.draw_track_legend_entry(ax)
        bev.draw_ego(ax)
        ax.legend(loc="upper right", fontsize=6)

        fig.canvas.draw()
        w, h = fig.canvas.get_width_height()
        img = Image.frombuffer("RGBA", (w, h), fig.canvas.buffer_rgba(), "raw", "RGBA", 0, 1).convert("RGB")
        pil_frames.append(img)
        this_delta = deltas_ms[i] if i < len(deltas_ms) else median_ms
        durations_ms.append(min(max(this_delta, _MIN_FRAME_MS), _MAX_FRAME_MS))

    plt.close(fig)

    scene_out = out_dir / scene_token[:8]
    scene_out.mkdir(parents=True, exist_ok=True)
    out_path = scene_out / f"{scene_token[:8]}_bev.gif"
    pil_frames[0].save(
        out_path, save_all=True, append_images=pil_frames[1:], duration=durations_ms, loop=0,
    )
    return out_path, len(pil_frames)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=None, help="One scene token. Default: every scene in --split.")
    parser.add_argument("--split", choices=["tune", "eval"], default="tune")
    parser.add_argument("--video", action="store_true", help="Part B: native-rate animated GIF instead of static keyframe PNGs.")
    parser.add_argument("--max-frames", type=int, default=None, help="--video only: truncate the event stream (quick test render).")
    args = parser.parse_args()

    from nuscenes.nuscenes import NuScenes

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())

    if args.scene:
        scene_tokens = [args.scene]
        # DEC-14: an explicit --scene is always allowed; its EVAL label
        # comes from split.json membership, not from --split (which only
        # drives the "render every scene" default case below).
        split_label = "eval" if args.scene in {s["token"] for s in split["eval"]} else "tune"
    else:
        scene_tokens = [s["token"] for s in split[args.split]]
        split_label = args.split

    out_dir = Path("outputs/figs/bev")
    for scene_token in scene_tokens:
        if args.video:
            out_path, n = render_scene_video(nusc, scene_token, out_dir, split_label, max_frames=args.max_frames)
            print(f"scene {scene_token[:8]} ({split_label}): {n}-frame GIF -> {out_path}")
        else:
            n = render_scene(nusc, scene_token, out_dir, split_label)
            print(f"scene {scene_token[:8]} ({split_label}): {n} keyframe PNGs -> {out_dir / scene_token[:8]}")


if __name__ == "__main__":
    main()
