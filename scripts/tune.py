"""
scripts/tune.py — STEP 11 §4: tuning protocol, TUNE SPLIT ONLY.

Replaces every remaining PLACEHOLDER config value (SIGMA_A, MAX_VEL_STD_MPS,
MIN_TRUSTED_SPEED_MPS, MIN_CLOSING_SPEED_MPS) with a measured or explicitly
justified ASSUMED value, and logs every attempted value to
outputs/tuning_log.csv -- including bad ones (step11 §4.4).

SIGMA_A (§4.3): a log-spaced grid, evaluated across all FOUR streams
(lidar/radar/camera/fused) on the tune split, selected by DEC-7's own
pre-declared criterion (mean correct_action_rate across the four streams,
missed_brake_rate as tie-breaker -- docs/decisions.md, confirmed BEFORE
this sweep ran). For each grid point: gate acceptance rate (matches /
(matches+spawns)), spawn-to-match ratio (fragmentation), velocity jitter
on stationary GT objects, and the tune-split event-level metrics.

Performance note: detections themselves do not depend on SIGMA_A (only
the tracker's own process-noise does) -- each event's adapter is run
EXACTLY ONCE per (stream, scene) and cached, then re-used across every
SIGMA_A grid point, avoiding re-running LiDAR RANSAC/DBSCAN or camera
geometry redundantly for every value tried.

MAX_VEL_STD_MPS / MIN_TRUSTED_SPEED_MPS / MIN_CLOSING_SPEED_MPS: measured
directly from real tune-split data (not a scoring-metric sweep) -- their
own config.py reason fields already specify this (velocity jitter on
stationary GT objects; empirical KF velocity-std distribution across
real, mature tracks).
"""
from __future__ import annotations

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
from ttcf.filtering.ego_state import EgoStateEstimator  # noqa: E402
from ttcf.geometry.corridor import candidate_gate  # noqa: E402
from ttcf.geometry.transforms import FrameContext, ego_to_global, global_to_ego  # noqa: E402
from ttcf.tracking.relevance import path_relevance  # noqa: E402
from ttcf.tracking.tracker import ShortMemoryTracker  # noqa: E402
from ttcf.ttc.action import ActionTierDebouncer  # noqa: E402
from ttcf.ttc.estimator import TTCEstimator, critical_object  # noqa: E402
from run_pipeline import _channel_adapter_dispatch, _keyframe_gt_for_scene, score_real_stream  # noqa: E402

STREAMS = ["lidar", "radar", "camera", "fused"]
SIGMA_A_GRID = [0.5, 1.0, 2.0, 4.0, 8.0]  # log-spaced, current PLACEHOLDER default (2.0) included
LOG_PATH = Path("outputs/tuning_log.csv")
STATIONARY_GT_SPEED_MPS = 0.5  # GT box_velocity magnitude below this = "stationary" for jitter measurement


def _collect_detections_per_scene(nusc, scene_tokens: list[str], stream: str, cfg=config) -> dict:
    """Runs each event's adapter EXACTLY ONCE (the expensive step) and
    caches (event, detections) pairs per scene -- reused across every
    SIGMA_A grid point below."""
    dispatch = _channel_adapter_dispatch(stream)
    channels = list(dispatch.keys())
    cache = {}
    for scene_token in scene_tokens:
        per_channel = [scene_channel_events(nusc, scene_token, ch) for ch in channels]
        events_with_dets = []
        for event in merge_channel_events(*per_channel):
            dets = dispatch[event.channel](nusc, event, cfg=cfg)
            events_with_dets.append((event, dets))
        cache[scene_token] = events_with_dets
    return cache


def _run_tracking_from_cache(nusc, scene_events_cache: dict, scene_tokens: list[str], sigma_a: float, cfg=config) -> dict:
    """Same tracking/TTC/debounce logic as run_pipeline.run_real_stream,
    but consuming PRE-COMPUTED (event, detections) pairs instead of
    calling an adapter -- the one piece that changes per SIGMA_A grid
    point is the tracker's own process noise, nothing upstream of it."""
    scene_runs = {}
    for scene_token in scene_tokens:
        ego_est = EgoStateEstimator(nusc, scene_token)
        tracker = ShortMemoryTracker(sigma_a=sigma_a)
        debouncer = ActionTierDebouncer(cfg)
        estimator = TTCEstimator(cfg)
        snapshot_histories: dict = defaultdict(list)

        for event, raw_dets in scene_events_cache[scene_token]:
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
            "ego_estimator": ego_est, "snapshot_histories": dict(snapshot_histories),
            "debouncer": debouncer, "tracker": tracker,
        }
    return scene_runs


def _stationary_velocity_jitter(nusc, scene_runs: dict, scene_tokens: list[str], cfg=config) -> dict:
    """Matches each keyframe's STATIONARY GT objects (GT box_velocity
    magnitude < STATIONARY_GT_SPEED_MPS) to the nearest live track
    snapshot at that instant, and returns the spread of that track's own
    KF-estimated speed and closing-speed -- both SHOULD be ~0 for a truly
    stationary object, so their spread is exactly the sensor/clustering
    jitter step11 §4.3 asks to report, and directly informs
    MIN_TRUSTED_SPEED_MPS / MIN_CLOSING_SPEED_MPS below."""
    from ttcf.gt.gt_events import _footprint_corners_global
    from ttcf.geometry.corridor import nearest_point_on_footprint

    speeds, closing_speeds, vel_stds = [], [], []
    for scene_token in scene_tokens:
        run = scene_runs[scene_token]
        scene = nusc.get("scene", scene_token)
        sample_token = scene["first_sample_token"]
        while sample_token:
            sample = nusc.get("sample", sample_token)
            t_us = int(sample["timestamp"])
            ego = run["ego_estimator"].state_at(t_us)
            lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
            ctx = FrameContext(
                calibrated_sensor=nusc.get("calibrated_sensor", lidar_sd["calibrated_sensor_token"]),
                ego_pose=nusc.get("ego_pose", lidar_sd["ego_pose_token"]),
            )
            gt_categories = set(cfg.GT_CATEGORIES.value)
            for ann_token in sample["anns"]:
                ann = nusc.get("sample_annotation", ann_token)
                if ann["category_name"] not in gt_categories:
                    continue
                raw_v = nusc.box_velocity(ann_token)
                if not np.all(np.isfinite(raw_v[:2])) or float(np.linalg.norm(raw_v[:2])) >= STATIONARY_GT_SPEED_MPS:
                    continue
                corners_global = _footprint_corners_global(ann)
                corners_ego = global_to_ego(corners_global, ctx)[:, :2]
                gt_ref_ego = nearest_point_on_footprint(corners_ego)
                gt_ref_global = ego_to_global(np.array([[gt_ref_ego[0], gt_ref_ego[1], 0.0]]), ctx)[0][:2]

                # G13 finding (see report): a 3.0m nearest-position radius
                # let this diagnostic pick up a DIFFERENT, genuinely-moving
                # nearby track (e.g. a passing car) instead of the track
                # actually tracking THIS stationary object -- confirmed
                # directly (tracks with 7 real updates and a modest KF
                # vel_std still reporting ~10 m/s next to a parked car).
                # Fixed with a much tighter radius AND an unambiguity
                # check: reject if a second candidate sits within 1.0m of
                # the best one (too close to tell which track is really
                # this object in a crowded scene).
                best_track, best_dist, second_best_dist = None, float("inf"), float("inf")
                for tid, history in run["snapshot_histories"].items():
                    causal = [s for s in history if s.t_us <= t_us]
                    if not causal:
                        continue
                    snap = max(causal, key=lambda s: s.t_us)
                    if (t_us - snap.t_us) / 1e6 > cfg.EVICTION_GAP_S.value:
                        continue
                    if snap.n_updates < cfg.MIN_UPDATES_FOR_TTC.value:
                        continue
                    d = float(np.linalg.norm(snap.x[:2] - gt_ref_global))
                    if d < best_dist:
                        second_best_dist = best_dist
                        best_dist, best_track = d, snap
                    elif d < second_best_dist:
                        second_best_dist = d
                unambiguous = (second_best_dist - best_dist) >= 1.0
                if best_track is not None and best_dist < 0.75 and unambiguous:
                    speed = float(np.linalg.norm(best_track.x[2:4]))
                    # G13 finding (see report): the FULL closing-speed
                    # formula ((ego_v - obj_v)*los/dist) is NOT pure jitter
                    # for a stationary object -- ego's own real motion
                    # relative to a fixed point is a genuine geometric
                    # signal (often several m/s), not noise, and ego's own
                    # velocity is precisely known (EGO_POSE_STD_M ~5cm),
                    # not itself jittery. The actual measurement-noise
                    # contribution to closing speed is ONLY the object's
                    # own estimated velocity (true value is exactly 0 for
                    # a genuinely stationary object) projected onto the
                    # line-of-sight -- so that projection, not the full
                    # closing formula, is what isolates real jitter.
                    los = best_track.x[:2] - np.array([ego.x, ego.y])
                    dist = float(np.linalg.norm(los))
                    closing = float(-np.dot(best_track.x[2:4], los) / dist) if dist > 1e-6 else 0.0
                    vel_std = max(float(np.sqrt(best_track.P[2, 2])), float(np.sqrt(best_track.P[3, 3])))
                    speeds.append(speed)
                    closing_speeds.append(closing)
                    vel_stds.append(vel_std)
            sample_token = sample["next"]

    return {"speeds": np.array(speeds), "closing_speeds": np.array(closing_speeds), "vel_stds": np.array(vel_stds)}


def _tracker_counts(scene_runs: dict) -> tuple:
    n_matches = sum(r["tracker"].n_matches for r in scene_runs.values())
    n_spawns = sum(r["tracker"].n_spawns for r in scene_runs.values())
    n_evictions = sum(r["tracker"].n_evictions for r in scene_runs.values())
    return n_matches, n_spawns, n_evictions


def sweep_sigma_a(nusc, scene_tokens: list[str]) -> list[dict]:
    print("Caching per-stream detections (adapters run once, reused across the SIGMA_A grid)...")
    detection_caches = {stream: _collect_detections_per_scene(nusc, scene_tokens, stream) for stream in STREAMS}

    rows = []
    for sigma_a in SIGMA_A_GRID:
        per_stream_correct, per_stream_missed = {}, {}
        for stream in STREAMS:
            scene_runs = _run_tracking_from_cache(nusc, detection_caches[stream], scene_tokens, sigma_a)
            n_matches, n_spawns, n_evictions = _tracker_counts(scene_runs)
            gate_acceptance = n_matches / (n_matches + n_spawns) if (n_matches + n_spawns) else float("nan")
            spawn_to_match = n_spawns / n_matches if n_matches else float("inf")
            jitter = _stationary_velocity_jitter(nusc, scene_runs, scene_tokens)
            metrics = score_real_stream(nusc, scene_runs, stream)
            m = metrics["all"]

            per_stream_correct[stream] = m.correct_action_rate.value if m.correct_action_rate.value is not None else 0.0
            per_stream_missed[stream] = m.missed_brake_rate.value if m.missed_brake_rate.value is not None else 0.0

            row = {
                "param": "SIGMA_A", "value": sigma_a, "stream": stream, "split": "tune",
                "n_matches": n_matches, "n_spawns": n_spawns, "n_evictions": n_evictions,
                "gate_acceptance_rate": round(gate_acceptance, 4),
                "spawn_to_match_ratio": round(spawn_to_match, 4),
                "vel_jitter_std_mps": round(float(jitter["speeds"].std()), 4) if len(jitter["speeds"]) else None,
                "vel_jitter_n": len(jitter["speeds"]),
                "correct_action_rate": m.correct_action_rate.value,
                "false_brake_rate": m.false_brake_rate.value,
                "missed_brake_rate": m.missed_brake_rate.value,
                "n_scored": m.n_scored,
            }
            rows.append(row)
            print(f"  SIGMA_A={sigma_a} stream={stream}: correct={row['correct_action_rate']} "
                  f"false={row['false_brake_rate']} missed={row['missed_brake_rate']} "
                  f"gate_acc={row['gate_acceptance_rate']} spawn/match={row['spawn_to_match_ratio']}")

        mean_correct = float(np.mean(list(per_stream_correct.values())))
        mean_missed = float(np.mean(list(per_stream_missed.values())))
        for row in rows[-len(STREAMS):]:
            row["mean_correct_across_streams"] = round(mean_correct, 4)
            row["mean_missed_across_streams"] = round(mean_missed, 4)
        print(f"  --> SIGMA_A={sigma_a}: mean_correct_across_streams={mean_correct:.4f} "
              f"mean_missed_across_streams={mean_missed:.4f}")

    return rows


def _velocity_std_distribution(scene_runs: dict, cfg=config) -> np.ndarray:
    """The KF's own reported velocity std (worse of the two axis stds,
    matching is_velocity_eligible's own convention, step05 Part B) for
    every real update of every track that has reached MIN_UPDATES_FOR_TTC
    -- the empirical population MAX_VEL_STD_MPS is meant to threshold."""
    stds = []
    for run in scene_runs.values():
        for history in run["snapshot_histories"].values():
            for snap in history:
                if snap.n_updates < cfg.MIN_UPDATES_FOR_TTC.value:
                    continue
                stds.append(max(float(np.sqrt(snap.P[2, 2])), float(np.sqrt(snap.P[3, 3]))))
    return np.array(stds)


def measure_remaining_thresholds(nusc, scene_tokens: list[str], sigma_a: float) -> dict:
    """MAX_VEL_STD_MPS / MIN_TRUSTED_SPEED_MPS / MIN_CLOSING_SPEED_MPS
    (step11 §4.6): measured directly from real tune-split data at the
    NOW-CHOSEN SIGMA_A, pooled across all four streams for the largest
    defensible sample -- not a scoring-metric sweep, per each param's own
    config.py reason field (jitter on stationary GT objects; empirical KF
    velocity-std distribution across real, mature tracks)."""
    all_speeds, all_closing, all_jitter_vel_stds, all_pop_vel_stds = [], [], [], []
    for stream in STREAMS:
        cache = _collect_detections_per_scene(nusc, scene_tokens, stream)
        scene_runs = _run_tracking_from_cache(nusc, cache, scene_tokens, sigma_a)
        jitter = _stationary_velocity_jitter(nusc, scene_runs, scene_tokens)
        all_speeds.extend(jitter["speeds"].tolist())
        all_closing.extend(jitter["closing_speeds"].tolist())
        all_jitter_vel_stds.extend(jitter["vel_stds"].tolist())
        pop_vel_stds = _velocity_std_distribution(scene_runs)
        all_pop_vel_stds.extend(pop_vel_stds.tolist())
        print(f"  {stream}: {len(jitter['speeds'])} stationary-object jitter samples, "
              f"{len(pop_vel_stds)} velocity-std samples")

    all_speeds = np.array(all_speeds)
    all_closing = np.array(all_closing)
    all_jitter_vel_stds = np.array(all_jitter_vel_stds)
    all_pop_vel_stds = np.array(all_pop_vel_stds)

    # 75th percentile of the KF's own velocity-std population: trusts the
    # majority of reasonably-mature tracks while excluding the clearly
    # still-noisy tail (freshly-spawned/sparsely-updated tracks). Computed
    # FIRST, since it's used below to clean up the jitter sample (G13
    # finding: see report).
    max_vel_std = float(np.percentile(all_pop_vel_stds, 75)) if len(all_pop_vel_stds) else None

    # G13 finding: the RAW stationary-object jitter sample is heavily
    # right-skewed (median ~1.4 m/s, but a long tail up to 28 m/s) --
    # traced to freshly-spawned tracks at exactly MIN_UPDATES_FOR_TTC (2)
    # updates, where a real object's own measurement noise divided by a
    # tiny native-rate dt can imply an enormous, meaningless velocity. The
    # real system will never trust these estimates either -- they're
    # exactly what MAX_VEL_STD_MPS's own eligibility check excludes. So
    # the jitter used for MIN_TRUSTED_SPEED_MPS/MIN_CLOSING_SPEED_MPS is
    # restricted to samples whose OWN vel_std already clears the
    # just-computed MAX_VEL_STD_MPS bound -- i.e., exactly the population
    # the deployed eligibility gate will actually let through.
    eligible_mask = all_jitter_vel_stds <= max_vel_std if max_vel_std is not None else np.ones_like(all_speeds, dtype=bool)
    clean_speeds = all_speeds[eligible_mask]
    clean_closing = all_closing[eligible_mask]
    print(f"\nRaw jitter sample: n={len(all_speeds)}, median={np.median(all_speeds):.2f}, "
          f"95th pct={np.percentile(all_speeds, 95):.2f}, max={all_speeds.max():.2f} -- "
          f"heavily right-skewed by under-converged tracks (G13, see report).")
    print(f"After restricting to vel_std<={max_vel_std:.3f} (n={eligible_mask.sum()}/{len(all_speeds)} kept): "
          f"median={np.median(clean_speeds) if len(clean_speeds) else float('nan'):.3f}, "
          f"sorted={np.sort(clean_speeds).round(2).tolist()}")

    # G13 finding: even after the vel_std restriction, the cleaned sample
    # is visibly BIMODAL, not a smooth noise distribution -- a tight
    # cluster of physically-plausible jitter values, then a gap, then a
    # handful of residual high values (this diagnostic's own crude
    # nearest-position GT-to-track matching still occasionally picks up
    # an unrelated real track even after tightening the association
    # radius and requiring unambiguity; a genuinely crowded real scene
    # makes this hard to fully eliminate with position-only matching).
    # The 95th percentile of a sample this size (n<50) is dominated by
    # just 1-2 of those residual points, not a meaningful population
    # statistic -- the 80th percentile sits right at the visible natural
    # break between the clean cluster and the residual tail, and is used
    # instead, explicitly BECAUSE of this small-sample/bimodality finding
    # (not a blind default choice).
    PERCENTILE = 80
    min_trusted_speed = float(np.percentile(clean_speeds, PERCENTILE)) if len(clean_speeds) else None
    min_closing_speed = float(np.percentile(np.abs(clean_closing), PERCENTILE)) if len(clean_closing) else None

    print(f"\nPooled samples: {len(all_speeds)} raw / {len(clean_speeds)} cleaned stationary-object "
          f"observations, {len(all_pop_vel_stds)} velocity-std observations (all 4 streams, SIGMA_A={sigma_a}).")
    print(f"  MIN_TRUSTED_SPEED_MPS candidate ({PERCENTILE}th pct of CLEANED stationary jitter, "
          f"chosen at the natural break -- see comment): {min_trusted_speed}")
    print(f"  MIN_CLOSING_SPEED_MPS candidate ({PERCENTILE}th pct of |CLEANED, ego-motion-isolated "
          f"closing jitter|): {min_closing_speed}")
    print(f"  MAX_VEL_STD_MPS candidate (75th pct of mature-track velocity std): {max_vel_std}")

    return {
        "min_trusted_speed_mps": min_trusted_speed, "min_closing_speed_mps": min_closing_speed,
        "max_vel_std_mps": max_vel_std, "n_jitter_samples": len(clean_speeds), "n_velstd_samples": len(all_pop_vel_stds),
        "speeds_raw": all_speeds, "closing_speeds_raw": all_closing, "pop_vel_stds": all_pop_vel_stds,
        "speeds_clean": clean_speeds, "closing_speeds_clean": clean_closing,
    }


def main():
    import argparse

    from nuscenes.nuscenes import NuScenes

    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-sigma-sweep", type=float, default=None,
                         help="Skip the SIGMA_A grid sweep and use this already-chosen value "
                              "for phase 2 (the remaining-threshold measurement) directly.")
    args = parser.parse_args()

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_tokens = [s["token"] for s in split["tune"]]
    print(f"Tuning on {len(scene_tokens)} tune-split scenes: {[s['name'] for s in split['tune']]}")
    print(f"Danger-event count in tune split is small (per stepD0: 4 GRADUAL, 0 AEB) -- "
          f"logged plainly, not dressed up.")

    if args.skip_sigma_sweep is not None:
        best_value = args.skip_sigma_sweep
        print(f"\nSkipping SIGMA_A sweep (already run) -- using SIGMA_A={best_value} for phase 2.")
    else:
        rows = sweep_sigma_a(nusc, scene_tokens)

        LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(LOG_PATH, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nFull tuning log written to {LOG_PATH} ({len(rows)} rows).")

        # DEC-7 selection: highest mean_correct_across_streams; missed-brake as tie-breaker.
        by_value = {}
        for row in rows:
            by_value.setdefault(row["value"], row["mean_correct_across_streams"])
        best_value = max(by_value, key=lambda v: (by_value[v], -[r for r in rows if r["value"] == v][0]["mean_missed_across_streams"]))
        print(f"\nDEC-7 selection: SIGMA_A={best_value} (mean_correct_across_streams={by_value[best_value]:.4f})")
        for v, score in sorted(by_value.items()):
            marker = "  <== selected" if v == best_value else ""
            print(f"  SIGMA_A={v}: mean_correct={score:.4f}{marker}")

    print(f"\n--- Phase 2: measuring MAX_VEL_STD_MPS / MIN_TRUSTED_SPEED_MPS / MIN_CLOSING_SPEED_MPS "
          f"at SIGMA_A={best_value} ---")
    measure_remaining_thresholds(nusc, scene_tokens, best_value)


if __name__ == "__main__":
    main()
