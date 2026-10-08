# Step 04a report — LiDAR adapter + first end-to-end run

## What was built (files, one line each)
- `src/ttcf/adapters/lidar.py` — `process_lidar_sweep`/`lidar_detections`: RawEvent(LIDAR_TOP) → list[Detection]. Load → sensor→ego transform → coarse ROI crop (with ego-self-return exclusion) → RANSAC ground removal → DBSCAN clustering → DEC-1 nearest-surface reference point per cluster.
- `scripts/measure_sensor_R.py` — GT-residual measurement of LiDAR's own R (tune split only), never auto-applies MEASURED status.
- `scripts/lidar_adapter_evidence.py` — real-data BEV overlay (3 tune-split keyframes), by-eye recall/false-detection check.
- `scripts/run_pipeline.py` — extended with `run_real_stream`/`score_real_stream` and a `--stream lidar --split tune` CLI mode: the first real, native-rate, single-sensor run through the whole pipeline (event stream → adapter → step05 candidate gate → tracker → TTC → path relevance → debounce → step09 extrapolation → step10 scoring).
- `tests/test_lidar_adapter.py` — 7 synthetic tests (step04a §6, all pass).
- `outputs/figs/step04a/bev_*.png` (3 keyframes, scene-1094, tune split).
- New `config.py` params (all ASSUMED): `LIDAR_MIN_RANGE_M`, `LIDAR_ROI_SLACK_M`, `LIDAR_ROI_Z_MIN_M`, `LIDAR_ROI_Z_MAX_M`, `LIDAR_GROUND_FIT_RANGE_M`, `LIDAR_GROUND_FIT_Z_MAX_M`, `GROUND_RANSAC_DIST_THRESH_M`, `GROUND_RANSAC_N_POINTS`, `GROUND_RANSAC_ITERS`, `LIDAR_CLUSTER_EPS_M`, `LIDAR_CLUSTER_MIN_SAMPLES`, `NEAREST_SURFACE_RADIUS_M`.

## Precedent check (V1/v2) — done before coding
- **PORTED**: V1's manual numpy RANSAC ground-plane algorithm (same tuned starting numbers: dist_thresh=0.2m, n_points=3, iters=100) — V1 kept this only as a validated reference implementation (its real pipeline used Open3D), this project uses it as the actual pipeline choice since Open3D is an unnecessary heavy addition here.
- **PORTED**: `sklearn.cluster.DBSCAN` with V1's tuned `eps=0.7m, min_samples=10` (clustering-library decision, `docs/decisions.md`, 2026-09-23).
- **NOT PORTED**: V1's cluster reference point (centroid) — DEC-1 chose nearest-surface-to-ego instead, built fresh.
- **NOT PORTED**: V1's cluster shape/size filtering (`CLUSTER_MIN_HEIGHT_M`, `CLUSTER_MAX_ASPECT_RATIO`, etc.) — step04a §4 explicitly forbids classifying objects with size heuristics (DEC-2); confirmed this stays forbidden even after finding an elongated non-vehicle cluster in real data (§4 below).
- No precedent anywhere in V1/v2 for an R-measurement script targeting this project's own adapter/tracker — `measure_sensor_R.py` is built fresh, reusing only the *lesson* (V1 item A.10: fragmentation corrupted its own LiDAR/radar R measurement) as the reason MEASURED status isn't applied automatically here either.

## Decisions taken / to ask
- **DEC-1** — already confirmed (nearest-surface-to-ego); consumed here for the cluster reference point.
- **Clustering library** — confirmed before coding: `sklearn.cluster.DBSCAN` (already on disk transitively via nuscenes-devkit), not a hand-built cKDTree fallback.
- **SENSOR_R_LIDAR MEASURED promotion — decided**: stays ASSUMED (confirmed by Siva, matching the recommendation), given N=16 is thin and the matching is nearest-neighbour-only (no real data association). See §5 below and `docs/decisions.md`.

## §1 — Real-data bug found and fixed: ego-vehicle self-returns
Running the adapter on real keyframes produced 9-14 detections against only 1 real in-path GT object per keyframe — investigated as a likely bug first (G13), not accepted as a real result.

Traced to the LiDAR's own vehicle-mount self-returns: one keyframe's raw ROI contained a 9,805-point "cluster" at ego-frame x∈[0.0002, 1.41]m, z∈[1.43, 1.84]m — squarely at the roof-mounted LiDAR's own mount height, at point-blank range, not an external object. The RANSAC ground plane itself was fitting correctly (near-vertical normal, `[-0.0157, 0.0169, -0.9997]`); the bug was upstream of it — the ROI crop never excluded the sensor's own vehicle body.

**Fix**: added `LIDAR_MIN_RANGE_M = 3.0` (ASSUMED, config.py) and excluded any point with ego-frame `x²+y² ≤ 3.0²` from the ROI before ground-fitting and clustering. 3.0m comfortably exceeds a passenger vehicle's own body extent from a roof-mounted sensor.

**Verified**: re-ran on the same real keyframe and two others — the mega-cluster is gone (obstacle-point counts dropped from ~9,000+ to a sane 1,300-3,100 range); re-ran all 7 synthetic adapter tests — still pass (the synthetic ground grid's default range starts at 0.5m, so some synthetic ground points now fall inside the exclusion zone, but ground points were already excluded by RANSAC regardless, so no test needed adjustment). Re-checked on an independent scene (scene-1094, tune split) to confirm the fix isn't scene-specific — same clean result, no mega-cluster reappears.

## §2 — Process error found and fixed: evidence script used an EVAL-split scene
While investigating §1, I noticed `scripts/lidar_adapter_evidence.py` (and my own ad hoc diagnostic scripts) had been run against `scene-1077` — which `configs/split.json` (DEC-8) actually assigns to the **eval** split, not tune. This directly contradicts step04a §4 Do NOT #6 ("do not evaluate on the eval split").

**Fix**: corrected the evidence script to use `scene-1094` (the tune split's busiest scene by in-path GT count per stepD0's own per-scene table: 35 in-path objects), deleted the scene-1077-derived BEV figures, and regenerated all real-data evidence — including re-verifying §1's self-return finding — on tune-split scenes only. The self-return artifact is a sensor-mounting geometry fact (constant across the whole dataset, not scene-content-dependent), and it reproduced identically on scene-1094, so §1's fix and its justification are unaffected by the correction — but the correction itself was necessary and is recorded here plainly rather than left unremarked. `run_pipeline.py`'s real-stream mode additionally reuses step10's own `guard_eval_scoring` (rather than a separate check) so this class of mistake is structurally harder to repeat going forward.

## §3 — Static, non-GT clusters: expected, not filtered
Real keyframes still show more adapter detections (7-13) than in-path GT objects (1-3) even after the self-return fix. Investigated one representative case: a 2,076-point cluster spanning ego-frame x∈[0, 12]m at a roughly constant y≈2-4.6m, z reaching the 3.0m ROI ceiling — clearly an elongated static roadside structure (wall/fence/building edge), not a vehicle, and not a self-return (it sits well outside `LIDAR_MIN_RANGE_M`).

This is real LiDAR data of a real physical object, and step04a §4 explicitly forbids adding a shape/size heuristic to filter or classify it out (DEC-2). nuScenes' GT set only labels vehicle/pedestrian categories, so static infrastructure like this will never have a matching GT box — the adapter is intentionally a broad, position-only candidate net (its ROI is the *coarse* filter; the *precise* corridor decision is step05's job, applied downstream in `run_pipeline.py`, not in the adapter). Confirmed this is genuinely by design, not an oversight: the candidate-gate half-width (`CORRIDOR_HALF_WIDTH_M`=1.2m + `CANDIDATE_EXTRA_MARGIN_M`=1.5m = 2.7m) is deliberately generous "so an object about to enter the corridor is not missed" — this structure's near edge sits just inside that margin in one of the three checked keyframes, correctly flagged as a *candidate* to track, not a false claim of a vehicle. Documented here rather than silently accepted or silently filtered.

## §4 — Measuring R (`scripts/measure_sensor_R.py`, §5)
Scope: tune-split, in-path GT objects (DEC-1/DEC-2) with `num_lidar_pts ≥ 50` (chosen from the tune split's own distribution — only 128 in-path GT rows exist total; ≥50 gives 16 rows, a defensible "well-observed" sample, vs. 13 at ≥100 or 9 at ≥200). Each is matched to its nearest adapter detection (position only, ego frame) at that keyframe — a coarse nearest-neighbour match, not real data association (no tracker identity exists at this stage), stated as a known limitation.

**Result**: 16 candidates, 0 total misses, inlier rate (residual < 2.0m) = **81.2% (13/16)** — clears the step's own 70% MEASURED threshold. Robust (MAD) σ_long = **0.082m**, σ_lat = **0.348m**, vs. the current ASSUMED (0.3m, 0.3m). σ_long is noticeably tighter than assumed (consistent with LiDAR's excellent range resolution); σ_lat is close to the assumed value.

**Not applied automatically** — per step04a §5's explicit rule. N=16 barely clears this project's own `MIN_COUNT_FOR_RATE=10` bar with little room to spare, and the nearest-neighbour matching (not real association) means an occasional residual could be against the wrong nearby object. **Decided: `SENSOR_R_LIDAR` stays ASSUMED** — confirmed, revisit after more data (step11) or once real tracked-object identity exists to do a cleaner match.

## §5 — First end-to-end LiDAR-only run (tune split, `--stream lidar --split tune`)
Command: `.venv\Scripts\python.exe scripts\run_pipeline.py --stream lidar --split tune`. All 5 tune scenes, native-rate LiDAR-only stream (every sweep, not just keyframes), scored against every tune-split keyframe with ≥1 in-path GT object (n_scored=79, DEC-4 primary set).

| Variant | n_scored | correct_action_rate | false_brake_rate | missed_brake_rate | under/over_brake | phantom_brake |
|---|---|---|---|---|---|---|
| all (unrestricted GT) | 79 | 94.9% (75/79) [0.88, 0.98] | 0.0% (0/75) [0.00, 0.05] | **100% (4/4) [0.51, 1.00] — indicative only** | 0 / 0 | 0/121 |
| obs_lidar (LiDAR-observable GT only) | 79 | **100% (79/79) [0.95, 1.00]** | 0.0% (0/79) [0.00, 0.05] | n/a (0/0) | 0 / 0 | 0/121 |

`no_estimate_reason_breakdown` (all variant, the 4 missed brakes): `NOT_ELIGIBLE`=2, `STALE`=1, a valid-track-but-NONE-tier case=1.

Tracker counters per scene (matches/spawns/evictions, `multi_sensor_fraction` — expected 0.000 for a single-stream run, since that fraction only rises once ≥2 sensor channels feed the same tracker at step04b+):
| scene | matches | spawns | evictions |
|---|---|---|---|
| scene-0103 | 115 | 46 | 46 |
| scene-0061 | 940 | 146 | 132 |
| scene-0655 | 33 | 37 | 34 |
| scene-0796 | 117 | 55 | 53 |
| scene-1094 | 672 | 89 | 86 |

**Surprising number investigated (G13): 100% missed-brake rate on the unrestricted variant, but 0/0 (n/a) on the obs_lidar-restricted one.** All 4 tune-split danger (GRADUAL) GT rows are the *same* fast-closing `vehicle.car` across 4 consecutive keyframes in scene-0103 (closing speed 15.8→17.3 m/s, distance 38→30m, GT TTC 2.4s→1.7s). Checked `num_lidar_pts` for all 4: **3, 0, 0, 0** — LiDAR genuinely had almost no returns on this specific object at this event (radar and camera both did: `obs_radar=True` at one keyframe, `obs_camera=True` at all 4). This is a real sensing-geometry limitation of this one event (likely range/angle/occlusion at that instant), not a tracker or adapter defect — confirmed against the GT's own point-count field, not inferred. It is exactly the scenario step10's observable-restricted variant exists to separate out fairly: LiDAR-only correctly scores a clean 100% on every danger event it could actually see (0 of them, hence n/a), and the unrestricted 100%-missed number is reported plainly rather than read as "the pipeline is broken." **False-brake rate is a clean 0.0% in both variants** — no spurious triggers from the static-structure candidates described in §3, consistent with them being correctly identified as non-closing (stationary) by the TTC math regardless of being tracked as candidates.

No fusion runs yet (radar/camera adapters arrive at step04b/c) — this is a single-modality baseline only, not compared against anything.

## Tests
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
117 passed, 14 warnings in 43.78s
```
All 7 of this step's tests pass, alongside the 110 from prior steps — no regressions.

## Real-data figures
`outputs/figs/step04a/bev_fb61d590.png`, `bev_7d82aa0c.png`, `bev_8d7dcd15.png` (scene-1094, tune split — 3 varied keyframes). Recall is visibly range-dependent: close/mid-range GT boxes (x<25m) are frequently matched closely by a detection; distant boxes (x>30m) are mostly undetected in the sparsest of the 3 keyframes, consistent with LiDAR point density falling off with range. Some detections have no nearby GT box at all — the static-structure candidates described in §3.

## Flagged, not built (out of scope here)
- Radar/camera streams (`--stream radar`/`camera`) — step04b/c.
- Fused run and the 4-run comparison table (`evaluation.report.generate_four_run_table`) — needs all 3 adapters, arrives at step11.
- Any tuning of `LIDAR_CLUSTER_EPS_M`/`GROUND_RANSAC_DIST_THRESH_M` against outcomes — step04a §7 explicitly says "do not tune to improve it in this step."
