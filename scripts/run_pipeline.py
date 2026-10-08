"""
scripts/run_pipeline.py — STEP 09 §3.1 (synthetic) / STEP 04a-c (real,
single-sensor) / STEP 11 §3 (fused, eval split): runs the pipeline
end-to-end and writes the pipeline log step09's extrapolation functions
consume, or scores a real stream against ground truth.

`run_synthetic` / `StaticEgoEstimator`: the original synthetic-only path
(step09), unchanged -- a simple scan sequence + constant-velocity ego
source, no real dataset needed.

`run_real_stream` / `score_real_stream`: the real, native-rate run --
event stream -> adapter(s) -> step05 candidate gate -> tracker -> TTC ->
path relevance -> debounce -> step09 extrapolation to GT keyframes ->
step10 scoring. --stream accepts "lidar", "radar", "camera", or "fused"
("radar" merges the 3 DEC-3 front radar channels; "fused" merges ALL
active channels -- LIDAR_TOP + 3 radars + CAM_FRONT -- into one
chronological stream feeding the SAME tracker, early fusion per G10, not
a separate per-sensor tracker merged afterward; "camera"/"fused" both
read ONLY from the on-disk cache scripts/cache_camera_detections.py
writes -- run that first, or camera_detections() raises
FileNotFoundError). --split eval requires --final AND a config hash
matching configs/frozen_config.json exactly (step11 §5.2) -- enforced by
metrics.guard_eval_scoring, reused rather than re-implemented here.

Pipeline log schema (step09 §3.1), written under outputs/runs/<run_name>/:
  - track_log.csv: one row per (real update, live track) -- t_us,
    TrackSnapshot fields, TTCResult fields, path-relevance verdict,
    is_critical_object flag.
  - action_log.csv: the action-decision timeline (ActionRecord per scan).
  - association_log.csv: the tracker's own decision log (matched/spawned,
    d^2, winning track) from step06.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ttcf import config  # noqa: E402
from ttcf.data.event_stream import merge_channel_events, scene_channel_events  # noqa: E402
from ttcf.evaluation.extrapolate import system_view_at  # noqa: E402
from ttcf.evaluation.metrics import EvalSplitGuardError, KeyframeGT, KeyframeSystem, guard_eval_scoring, score_run  # noqa: E402
from ttcf.filtering.ego_state import EgoState, EgoStateEstimator  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.gt.gt_events import build_scene_gt_events  # noqa: E402
from ttcf.tracking.relevance import path_relevance  # noqa: E402
from ttcf.tracking.tracker import ShortMemoryTracker  # noqa: E402
from ttcf.ttc.action import ActionTierDebouncer  # noqa: E402
from ttcf.ttc.estimator import TTCEstimator, critical_object  # noqa: E402


class StaticEgoEstimator:
    """Minimal ego source for a synthetic run: constant velocity from an
    origin. Real runs use ttcf.filtering.ego_state.EgoStateEstimator
    instead (needs a real NuScenes scene) -- same .state_at(t_us)
    interface, so callers downstream (extrapolate_track, system_view_at)
    don't need to know which kind they were given."""

    def __init__(self, x0=0.0, y0=0.0, vx=0.0, vy=0.0, heading=0.0, t0_us=0):
        self.x0, self.y0, self.vx, self.vy, self.heading, self.t0_us = x0, y0, vx, vy, heading, t0_us

    def state_at(self, t_us: int) -> EgoState:
        dt = (int(t_us) - self.t0_us) / 1e6
        return EgoState(
            t_us=int(t_us), x=self.x0 + self.vx * dt, y=self.y0 + self.vy * dt,
            vx=self.vx, vy=self.vy, speed_mps=float(np.hypot(self.vx, self.vy)),
            heading_rad=self.heading,
        )


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_synthetic(scans, ego_estimator, run_name: str = "synthetic_demo") -> Path:
    """scans: list of (t_us, list[Detection]) -- see synthetic_scenarios.py.
    Returns the output directory the log was written to."""
    tracker = ShortMemoryTracker()
    estimator = TTCEstimator()
    debouncer = ActionTierDebouncer()

    track_rows: list[dict] = []
    action_rows: list[dict] = []

    for t_us, dets in scans:
        snapshots = tracker.process_scan(dets, t_us)
        ego = ego_estimator.state_at(t_us)

        results, relevances = {}, {}
        for snap in snapshots:
            results[snap.track_id] = estimator.estimate(snap, ego)
            relevances[snap.track_id] = path_relevance(snap, ego)

        winner = critical_object(results, relevances)
        critical_id = winner[0] if winner else None
        critical_result = winner[1] if winner else None

        for snap in snapshots:
            result = results[snap.track_id]
            rel = relevances[snap.track_id]
            track_rows.append(
                {
                    "t_us": t_us,
                    "track_id": snap.track_id,
                    "x": snap.x[0], "y": snap.x[1], "vx": snap.x[2], "vy": snap.x[3],
                    "n_updates": snap.n_updates,
                    "sensors_in_estimate": ";".join(sorted(snap.sensors_in_estimate)),
                    "first_t_us": snap.first_t_us,
                    "distance_m": result.distance_m,
                    "closing_speed_mps": result.closing_speed_mps,
                    "ttc_s": result.ttc_s,
                    "ttc_reason": result.reason.value,
                    "closing_std_mps": result.closing_std_mps,
                    "relevance": rel.verdict.value,
                    "relevance_provisional": rel.provisional,
                    "is_critical_object": snap.track_id == critical_id,
                }
            )

        action_rec = debouncer.process(t_us, critical_result)
        action_rows.append(
            {
                "t_us": action_rec.t_us,
                "tier": action_rec.tier.value,
                "driving_track_id": action_rec.driving_track_id,
                "consecutive_count": action_rec.consecutive_count,
                "no_estimate": action_rec.no_estimate,
            }
        )

    out_dir = Path("outputs/runs") / run_name
    _write_csv(out_dir / "track_log.csv", track_rows)
    _write_csv(out_dir / "action_log.csv", action_rows)
    _write_csv(
        out_dir / "association_log.csv",
        [
            {"t_us": e.t_us, "channel": e.channel, "cls": e.cls, "outcome": e.outcome,
             "d2": e.d2, "track_id": e.track_id}
            for e in tracker.association_log
        ],
    )
    return out_dir


# ── STEP 04a: real, single-sensor stream run ────────────────────────────

_ADAPTER_DISPATCH = {}  # populated lazily -- importing ttcf.adapters.* eagerly
# at module load would require nuscenes/sklearn/torch even for the
# synthetic-only path, so it's deferred to first real use.


def _channel_adapter_dispatch(stream: str) -> dict:
    """Returns {channel: adapter_fn}. "fused" is the union of all three
    single-sensor dispatch tables (step11 §3: "the fused run enables
    exactly the union of the three single-sensor adapters") -- each
    channel still goes through its OWN adapter, all writing into the same
    tracker (early fusion, G10), not a separate per-sensor tracker merged
    afterward."""
    if not _ADAPTER_DISPATCH:
        from ttcf.adapters.camera import camera_detections
        from ttcf.adapters.lidar import lidar_detections
        from ttcf.adapters.radar import radar_detections

        _ADAPTER_DISPATCH["lidar"] = {"LIDAR_TOP": lidar_detections}
        _ADAPTER_DISPATCH["radar"] = {
            "RADAR_FRONT": radar_detections, "RADAR_FRONT_LEFT": radar_detections,
            "RADAR_FRONT_RIGHT": radar_detections,
        }
        _ADAPTER_DISPATCH["camera"] = {"CAM_FRONT": camera_detections}
        _ADAPTER_DISPATCH["fused"] = {
            **_ADAPTER_DISPATCH["lidar"], **_ADAPTER_DISPATCH["radar"], **_ADAPTER_DISPATCH["camera"],
        }
    if stream not in _ADAPTER_DISPATCH:
        raise ValueError(f"Unknown stream {stream!r} (only 'lidar'/'radar'/'camera'/'fused' exist as of step11)")
    return _ADAPTER_DISPATCH[stream]


def run_real_stream(nusc, scene_tokens: list[str], stream: str, cfg=config) -> dict:
    """One stream's native-rate events (every sweep, not just keyframes,
    merged chronologically across all of the stream's channels), per scene
    (tracker/ego-estimator/debouncer are reset per scene -- a scene
    boundary is a real discontinuity, same precedent step06 already cites
    for not carrying tracker state across scenes). Returns scene_token ->
    {"ego_estimator", "snapshot_histories", "debouncer"}."""
    dispatch = _channel_adapter_dispatch(stream)
    channels = list(dispatch.keys())
    scene_runs = {}

    for scene_token in scene_tokens:
        ego_est = EgoStateEstimator(nusc, scene_token)
        tracker = ShortMemoryTracker(sigma_a=cfg.SIGMA_A.value)
        debouncer = ActionTierDebouncer(cfg)
        estimator = TTCEstimator(cfg)
        snapshot_histories: dict = defaultdict(list)

        per_channel_events = [scene_channel_events(nusc, scene_token, ch) for ch in channels]
        for event in merge_channel_events(*per_channel_events):
            raw_dets = dispatch[event.channel](nusc, event, cfg=cfg)

            # Step05 candidate gate BEFORE the tracker (step04a §7's own
            # pipeline order) -- the adapter only applied the coarse ROI
            # (step04a §4 Do NOT #1); this is the first place the
            # precise-ish, still-generous candidate corridor applies.
            ctx = FrameContext.from_event(nusc, event)
            dets = [
                d for d in raw_dets
                if candidate_gate(global_to_ego(np.array([[d.xy_global[0], d.xy_global[1], 0.0]]), ctx)[0][:2], cfg)
            ]

            snapshots = tracker.process_scan(dets, event.t_us)
            for snap in snapshots:
                snapshot_histories[snap.track_id].append(snap)

            ego = ego_est.state_at(event.t_us)
            results, relevances = {}, {}
            for snap in snapshots:
                results[snap.track_id] = estimator.estimate(snap, ego)
                relevances[snap.track_id] = path_relevance(snap, ego, cfg)
            winner = critical_object(results, relevances)
            debouncer.process(event.t_us, winner[1] if winner else None)

        scene_runs[scene_token] = {
            "ego_estimator": ego_est,
            "snapshot_histories": dict(snapshot_histories),
            "debouncer": debouncer,
            "tracker": tracker,
        }

    return scene_runs


def _keyframe_gt_for_scene(nusc, scene_token: str):
    """Returns (list[KeyframeGT], sample_token -> t_us). KeyframeGT itself
    (step10's fixed type) carries no timestamp -- t_us is needed
    separately to drive extrapolation to exactly this GT instant (G15)."""
    gt_rows, keyframe_rows = build_scene_gt_events(nusc, scene_token)
    by_sample: dict = defaultdict(list)
    for row in gt_rows:
        by_sample[row.sample_token].append(row)

    out = []
    t_us_by_sample = {}
    for kf in keyframe_rows:
        rows = by_sample.get(kf.sample_token, [])
        out.append(
            KeyframeGT(
                sample_token=kf.sample_token,
                is_empty_corridor=kf.is_empty_corridor,
                object_tiers=[r.gt_action for r in rows],
                object_observability=[
                    {"obs_lidar": r.obs_lidar, "obs_radar": r.obs_radar, "obs_camera": r.obs_camera, "obs_any": r.obs_any}
                    for r in rows
                ],
            )
        )
        t_us_by_sample[kf.sample_token] = kf.t_us
    return out, t_us_by_sample


def score_real_stream(nusc, scene_runs: dict, run_name: str, cfg=config) -> dict:
    """Extrapolates each scene's real run to every one of its GT keyframes
    (step09, G15) and scores the whole tune split at once (step10). Returns
    {"all": RunMetrics, obs_field: RunMetrics} -- obs_field is
    f"obs_{run_name}" (e.g. "obs_lidar"), step10 §3.3's observable-
    restricted variant for this exact sensor."""
    all_gt_keyframes: list[KeyframeGT] = []
    system_by_sample: dict = {}

    for scene_token, run in scene_runs.items():
        gt_keyframes, t_us_by_sample = _keyframe_gt_for_scene(nusc, scene_token)
        all_gt_keyframes.extend(gt_keyframes)

        for kf in gt_keyframes:
            t_us = t_us_by_sample[kf.sample_token]
            sys_view = system_view_at(
                run["snapshot_histories"], run["ego_estimator"], t_us,
                action_debouncer=run["debouncer"], cfg=cfg,
            )
            tier = sys_view.action.tier.value if sys_view.action is not None else "NONE"
            system_by_sample[kf.sample_token] = KeyframeSystem(
                sample_token=kf.sample_token, tier=tier, no_estimate_reason=sys_view.no_estimate_reason,
            )

    # "fused" has no obs_fused field (stepD0's per-object observability is
    # only obs_lidar/obs_radar/obs_camera/obs_any) -- its own observable-
    # restricted variant is "seen by ANY active sensor", i.e. obs_any.
    obs_field = "obs_any" if run_name == "fused" else f"obs_{run_name}"
    variant_all = score_run(all_gt_keyframes, system_by_sample, run_name, "all", None, cfg.MIN_COUNT_FOR_RATE.value)
    variant_obs = score_run(
        all_gt_keyframes, system_by_sample, run_name, obs_field, obs_field, cfg.MIN_COUNT_FOR_RATE.value
    )
    return {"all": variant_all, obs_field: variant_obs}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stream", choices=["lidar", "radar", "camera", "fused"], default=None)
    parser.add_argument("--split", choices=["tune", "eval"], default="tune")
    parser.add_argument("--final", action="store_true",
                         help="Required to score the eval split (step11 §5.2) -- exactly once, "
                              "and only when the live config hash matches configs/frozen_config.json.")
    args = parser.parse_args()

    if args.stream is None:
        from synthetic_scenarios import single_object_scans

        # A closing scenario: object stationary at (30,0), ego approaching
        # at 10 m/s -- enough real updates for eligibility and a clean AEB fire.
        scans = single_object_scans(30.0, 0.0, 0.0, 0.0, 0, 100_000, 40, rng=np.random.default_rng(0))
        ego = StaticEgoEstimator(x0=0.0, vx=10.0)
        out_dir = run_synthetic(scans, ego, run_name="synthetic_demo")
        print(f"Pipeline log written to {out_dir}/ (track_log.csv, action_log.csv, association_log.csv)")
        return

    # Reuses step10's own eval-split guard (logged, single source of truth) --
    # scoring the eval split requires --final AND the live config hash to
    # match configs/frozen_config.json's own stored hash exactly (step11 §5.2).
    try:
        guard_eval_scoring(
            split=args.split, final=args.final, config_hash=config.current_config_hash(),
            frozen_config_path=Path("configs/frozen_config.json"), run_name=args.stream or "unknown",
        )
    except EvalSplitGuardError as exc:
        print(f"STOP: {exc}")
        sys.exit(1)

    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_tokens = [s["token"] for s in split[args.split]]

    print(f"Running stream={args.stream!r} on {len(scene_tokens)} {args.split}-split scenes...")
    scene_runs = run_real_stream(nusc, scene_tokens, args.stream)
    metrics = score_real_stream(nusc, scene_runs, args.stream)

    for variant, m in metrics.items():
        print(f"\n--- {args.stream} / {variant} ---")
        print(f"n_scored={m.n_scored}")
        print(f"correct_action_rate={m.correct_action_rate}")
        print(f"false_brake_rate={m.false_brake_rate}")
        print(f"missed_brake_rate={m.missed_brake_rate}")
        print(f"under_brake={m.under_brake_count} over_brake={m.over_brake_count}")
        print(f"phantom_brake={m.phantom_brake_count}/{m.phantom_brake_denominator}")
        print(f"no_estimate_reason_breakdown={m.no_estimate_reason_breakdown}")

    for scene_token, run in scene_runs.items():
        t = run["tracker"]
        print(f"\nscene {scene_token[:8]}: matches={t.n_matches} spawns={t.n_spawns} "
              f"evictions={t.n_evictions} multi_sensor_fraction={t.multi_sensor_fraction:.3f}")


if __name__ == "__main__":
    main()
