# Project status — rolling build log

One entry per step, appended in build order (README_MASTER.md §7), most recent
last. Short and scannable — full detail always lives in `reports/stepXX_report.md`
and `docs/learning/stepXX_learning.md`; this file is the index/summary, not a
replacement for either.

## Build order (README_MASTER.md §7 — NOT the numeric file order)

The GT table needs ego velocity, TTC math, and corridor geometry, so those
come first; the sensor adapters (step04a/b/c) come last, since they only
make sense once the pipeline they feed (tracker → TTC → debounce → metrics)
is built and proven on synthetic data.

| # | File | What it builds | Status |
|---|---|---|---|
| 1 | step00_config_contracts.md | dataset preflight, config, shared types | APPROVED |
| 2 | step02_transform.md | sensor→ego→global utility | APPROVED |
| 3 | step03_kf_ego_state.md | shared KF + ego-state estimator | APPROVED |
| 4 | step07_ttc.md **Part A** | pure TTC math + synthetic tests | APPROVED |
| 5 | step05_forward_path_filter.md **Part A** | corridor geometry + candidate gate | APPROVED |
| 6 | stepD0_gt_event_table.md | GT event table, split proposal | APPROVED |
| 7 | step01_event_stream.md | native-rate multi-channel event stream | APPROVED |
| 8 | step06_tracker.md | shared short-memory tracker (synthetic only) | APPROVED |
| 9 | step05_forward_path_filter.md **Part B** | track-level path relevance | APPROVED |
| 10 | step07_ttc.md **Part B** | track-level TTC estimator | APPROVED |
| 11 | step08_debounce_action.md | debounce + action tiers (synthetic) | APPROVED |
| 12 | step09_extrapolation.md | bounded, causal extrapolation to GT time | APPROVED |
| 13 | step10_metrics.md | event-level metrics + report generator | APPROVED |
| 14 | step04a_lidar_adapter.md | LiDAR adapter + first end-to-end LiDAR run | APPROVED |
| 15 | step04b_radar_adapter.md | radar adapter, Doppler OFF | APPROVED |
| 16 | step04c_camera_adapter.md | camera adapter (single-sensor) | APPROVED |
| 17 | step11_runs_tuning_report.md | four runs, tuning, freeze, final evaluation | reported (awaiting final approval) |

---

## step00_config_contracts — APPROVED
- Built: `src/ttcf/config.py` (`Param` status taxonomy: MEASURED/ASSUMED/PLACEHOLDER), `src/ttcf/types.py` (shared dataclasses), `scripts/preflight_dataset.py`.
- Decisions: DEC-3 (active channels: LIDAR_TOP + 3 front radars + CAM_FRONT), DEC-9 (full starting-values table) — both approved as proposed.
- Tests: 12/12 passing.
- Real-data: 10 scenes, 404 samples, sweeps verified on all 12 channels, devkit `1.2.0`.
- Finding: cameras measured exactly 10.00 Hz (vs. ~12 Hz commonly cited) — investigated at 3 levels, accepted as a real dataset property, not a bug. Flagged for step04c.
- Env: `.venv` created, `requirements.txt` pinned from old repo's versions (core set only, heavy ML deps excluded).

## step02_transform — APPROVED
- Built: `src/ttcf/geometry/transforms.py` (full sensor→ego→global chain, `FrameContext` with token caching, `rotate_cov_ego_to_global`), `tests/test_transforms.py`.
- Decisions: none (§6: None).
- Tests: 18/18 passing (12 step00 + 6 step02).
- Real-data: transform independently validated against nuScenes' own GT box translation — floating-point-exact match. Ego-motion evidence figure (`outputs/figs/step02/ego_motion_evidence.png`): global centroid agrees within 0.40 m across keyframes despite 4.81 m of ego motion.
- Finding (bug hunt, not just a pass): initial test-5 run failed by 2.1 m — traced to a nuScenes-mini instance labeled `vehicle.parked` on all 21 annotations while its own GT position moves ~5.4 m/s. Confirmed the transform code was correct first (exact match vs. GT), then fixed the test's candidate selection to verify true stationarity from GT position, not the attribute label alone.
- Open question carried forward: should the `vehicle.parked`-mislabeling risk be flagged into `stepD0_gt_event_table.md`'s own GT handling?

## step03_kf_ego_state — APPROVED
- Built: `src/ttcf/filtering/kalman.py` (`ConstantVelocityKF`: Joseph-form update, `state_at` non-mutating peek, `innovation_cov`/`mahalanobis_sq`), `src/ttcf/filtering/ego_state.py` (`EgoStateEstimator`, one per scene, incrementally cached).
- Decisions: none (§7: None). `SIGMA_A_EGO=1.0` behavior reported (plots look neither over- nor under-smoothed), no adjustment made.
- Tests: 30/30 passing (12 step00 + 6 step02 + 8 synthetic KF + 4 real-data ego-state).
- Real-data: NEES over 300 Monte-Carlo runs = 3.964 (ideal 4.0) — filter is internally self-consistent. All 10 scenes' speed-vs-time plots smooth and urban-plausible (`outputs/figs/step03/ego_speed_all_scenes.png`).
- Findings (two bugs caught before they compounded): (1) internal clock accumulated as float seconds drifted enough to produce a spuriously negative `dt` on an exact-timestamp query — fixed by switching to an exact-integer-microseconds clock, matching the project's own `t_us` convention. (2) `state_at()` rebuilt its full KF chain from scratch on every call — a real-data test stalled past 3 minutes at realistic query volume; fixed with an incremental cache (43s afterward).
- Open question carried forward: confirm `SIGMA_A_EGO=1.0` before stepD0's GT builder and later steps consume it downstream.

## step07_ttc Part A — APPROVED
- Built: `src/ttcf/ttc/ttc_math.py` (`ttc_from_state`, pure function), `tests/test_ttc_math.py` (10 tests).
- Decisions: none for Part A (DEC-10 belongs to Part B, later).
- Tests: 40/40 passing (30 prior + 10 this step).
- Finding (caught before writing code, not after): step07_ttc.md's own literal formula contradicted its own worked test 1 (gave closing=-10 where the spec requires +10) — verified numerically, resolved in favor of the physically-correct, test-matching, v2-validated sign convention, documented at the top of `ttc_math.py`. **step07_ttc.md itself corrected** (A2 step 3) on 2026-09-20 at Siva's request, with an inline note explaining why.

## step05_forward_path_filter Part A — APPROVED
- Built: `src/ttcf/geometry/corridor.py` (`corridor_contains`, `candidate_gate`, `nearest_point_on_footprint`, `footprint_intersects_corridor`), `tests/test_corridor.py`.
- Decisions: **DEC-1 confirmed** — B, nearest-surface-to-ego. Needed now (footprint membership rule depends on it); `config.DISTANCE_DEFINITION` updated PLACEHOLDER → ASSUMED. DEC-9 corridor values already approved at step00, just consumed here.
- Precedent check: no corridor/forward-path code exists anywhere in V1/v2 (expected — both were MOT-scoped, tracked everything). Built fresh.
- Tests: 45/45 passing (40 prior + 5 this step).
- Implemented rule stated precisely per step05's own request: GT footprint membership decided on the exact nearest point of the footprint's boundary/interior to the ego origin (real closest-point-on-polygon geometry), not centroid, not full polygon overlap.
- Flagged, not built: curved-road/intersection path prediction — a named, accepted limitation per the step file's own instruction, not silently patched.

## stepD0_gt_event_table — APPROVED
- Built: `src/ttcf/gt/gt_events.py` (`build_scene_gt_events`, `build_all_gt_events`), `scripts/build_gt_events.py`, `outputs/gt_events.csv` (315 rows), `outputs/tables/stepD0_per_scene.csv`, BEV + histogram figures.
- Decisions: **DEC-2 confirmed** (symmetric class policy, 17-category GT scope incl. `vehicle.trailer`), **DEC-4 confirmed** (keyframe-level, in-path primary set). DEC-1/DEC-9 already settled, consumed here. **DEC-8 (split) proposed, not yet approved** — `configs/split.json` NOT written.
- Precedent check: v2's `compute_ground_truth_ttc()` uses a raw finite-difference GT velocity — explicitly NOT ported (this project's own `nuscenes-reference.md` names that exact function as the wrong pattern); built against `nusc.box_velocity()` instead, reusing this project's own `EgoStateEstimator`/`ttc_from_state`.
- Tests: 50/50 passing (45 prior + 5 this step). One test-expectation mistake caught and fixed (my test assumed centroid distance; code correctly used DEC-1's nearest-surface distance).
- Real-data: 404 keyframes, 212 in-path (52.5%), 315 GT rows, 11 danger events (3.49%) — small, stated plainly. Zero vel_invalid rows, checked against the dataset-wide 0.11% NaN-velocity base rate before accepting as plausible (not a bug).
- Finding (real bug, not cosmetic): first BEV-figure version silently rendered empty for a danger scene (wrong keyframe-selection heuristic + fixed view window clipped the object off-screen) — ran without error, so could have been mistaken for "no danger here." Fixed via severity-based keyframe selection + dynamic view window.
- Finding (split-proposal constraint): all 5 AEB events sit in one scene (`scene-1100`) — no scene-level split can give both sides AEB representation. Assigned it to eval explicitly (with reasoning) rather than let a naive round-robin silently zero out eval's AEB tier.
- **DEC-8 approved** — split written to `configs/split.json`, now FIXED per stepD0's own rule (tune: scene-0103/0061/0655/0796/1094; eval: scene-1100/1077/0553/0757/0916).

## step01_event_stream — APPROVED
- Built: `src/ttcf/data/event_stream.py` (`scene_channel_events`, `merge_channel_events`, `iter_scene_events`, `keyframe_only`, `resolve_path`), `scripts/event_stream_evidence.py`.
- Decisions: none (DEC-3 already settled).
- Precedent check: ported v2's `heapq.merge` streaming technique directly (already correct there); not porting v2's eagerly-loaded `SensorEvent` shape (this step wants lazy, metadata-only events merged before any file loading, matching adapters coming near the end of the build order). No precedent anywhere for scene-bounded sweep-chain walking — built fresh.
- Tests: 56/56 passing (50 prior + 6 this step) — all passed on first implementation.
- Real-data evidence: raster plot showing native-rate interleaving (LiDAR ≈20Hz, radars ≈13Hz, camera ≈11.6Hz — matches step00), inter-arrival table, keyframe ms-offsets (up to ±35.5ms spread within one nominal "sample," confirming per-channel timing independence).

## step06_tracker — APPROVED
- Built: `src/ttcf/tracking/tracker.py` (`ShortMemoryTracker`, N×M Hungarian assignment, `AssociationLogEntry`, counters), `scripts/synthetic_scenarios.py` (reusable generators for later steps).
- Decisions: **DEC-5 confirmed** — `sensors_in_estimate` bounded to last `MAX_MEMORY_UPDATES=4` updates (new config param), not a lifetime set. Eligibility/eviction reuse existing config values.
- Precedent check: ported v2's predict-then-gate/evict-before-gate ordering and S=P+R gating; explicitly not porting v2's long eviction timer, scene-reset subsystem, or lifetime `sensors_seen`/`history` (G11). Built real N×M Hungarian assignment (v2 only ever handled 1 detection at a time).
- Tests: 66/66 passing (56 prior + 10 this step).
- Test 2 (S=P+R): real measured numbers — d²(S=P+R)=0.626 (accepted) vs. d²(P alone)=29.195 (would reject), gate=9.21.
- **Test 4 (crowd conflation) — richer finding than expected:** first scenario didn't reproduce conflation; investigated why, redesigned to match the actual failure mode (object 2 debuts near an *already-established* track with no competing detection), which revealed **track identity hijacking** — track count stayed correct (2) while the original track's identity was captured by the wrong object. Documented per what-not-to-do.md §4 / lessons-from-v1-v2.md item C.3, explicitly not fixed.

## step05_forward_path_filter Part B — APPROVED
- Built: `src/ttcf/tracking/relevance.py` (`path_relevance`, `is_velocity_eligible` — written once for step07 Part B to reuse), `tests/test_relevance.py`.
- Decisions: none.
- Precedent check: confirmed again — no corridor/forward-path code anywhere in V1/v2. Built fresh.
- Tests: 72/72 passing (66 prior + 6 this step) — all passed on first implementation.
- Design choice made explicit (not fully pinned down by spec): "approaching longitudinally" implemented as a separate condition alongside full-position extrapolation (not x-only) — catches a real edge case (a stationary roadside object has a closing longitudinal gap by definition, but must still resolve to OUT since its lateral position never moves).

## step07_ttc Part B — APPROVED
- Built: `src/ttcf/ttc/estimator.py` (`TTCEstimator.estimate`, `conservative_ttc` diagnostic, `critical_object`), `tests/test_ttc_estimator.py`.
- Decisions: **DEC-10 confirmed** — point estimate drives action (step08); conservative bound is diagnostic only, never acted on for v1.
- Precedent check: no "critical object selection" precedent anywhere in V1/v2 (an AEB-decision-layer concept this project introduces, not something MOT scope ever needed). Built fresh; reused `is_velocity_eligible` and `ttc_from_state` unchanged, as required.
- Tests: 78/78 passing (72 prior + 6 this step) — all passed on first implementation.
- Test 4 (causality) proves the causality obligation materially matters — feeding a later/faster/closer ego state gives a measurably different (wrong) answer, not just a differently-timestamped one.

## step08_debounce_action — APPROVED
- Built: `src/ttcf/ttc/action.py` (`ActionTierDebouncer`, `ActionRecord`, `LatencyRecord`), `tests/test_action.py`, `outputs/figs/step08/timeline.png`.
- Decisions: none new — DEC-9 (thresholds) and DEC-10 (point estimate) already settled, directly consumed here.
- Precedent check: no debounce/action-tier logic anywhere in V1/v2 (neither ever built a decision layer). Built fresh.
- Design choice: `ActionRecord` returned as a superset of the step00-approved `ActionDecision` (adds `no_estimate: bool`) rather than modifying the shared contract.
- Tests: 88/88 passing (78 prior + 10 this step).
- **Two real logic bugs found and fixed via failing tests** (not test-only mistakes): (1) `consecutive_count` silently stayed 0 until the exact instant a tier fired, instead of showing in-progress streak — fixed. (2) latency test assumed 1 record per debounce sequence; investigated and found the scenario legitimately fires both AEB and GRADUAL simultaneously (2 records) — fixed the test, not the code. Also one test-scenario bug in the timeline figure (V-shape too shallow to hold below the AEB threshold for `DEBOUNCE_N` updates) — fixed by widening the dip, not lowering `DEBOUNCE_N`.

## step09_extrapolation — APPROVED
- Built: `src/ttcf/evaluation/extrapolate.py` (`extrapolate_track`, `system_view_at`, `associate_tracks_to_gt`), `scripts/run_pipeline.py` (synthetic end-to-end runner, writes the pipeline log schema).
- Decisions: none.
- Precedent check: v2's `evaluate.py` (`position_at`, `match_track_to_ground_truth`) directly confirms step09's own claim that both bounding bugs (unbounded extrapolation, backward extrapolation) were found and fixed in V2 — same guards, same reasoning, ported directly using this project's own `EVICTION_GAP_S`. Deliberately not porting v2's no-covariance-growth arithmetic (reused the tested `ConstantVelocityKF.state_at` instead) or its locked-identity association (built a fresh per-instant Hungarian match, per G11).
- Tests: 98/98 passing (88 prior + 10 this step) — all passed on first implementation.
- Real-data verification N/A (no adapters yet); `run_pipeline.py` demoed end-to-end on synthetic data, log schema internally consistent with step06/07/08's already-verified behavior.

## step10_metrics — APPROVED
- Built: `src/ttcf/evaluation/metrics.py` (`score_run`, `wilson_score_interval`, `guard_eval_scoring`), `src/ttcf/evaluation/report.py` (`build_footnotes`, `generate_four_run_table`).
- Decisions: none. DEC-1/DEC-2/DEC-4/DEC-9 already settled, consumed here.
- Precedent check: v2's evaluate.py is entirely full-track MAE/RMSE/match-rate based — exactly what this step forbids. No event-level/confusion-matrix/Wilson-interval precedent anywhere. Built entirely fresh.
- Tests: 110/110 passing (98 prior + 12 this step) — all passed on first implementation. Wilson interval checked against published reference values (50/100→[0.404,0.596], 0/10→[0.0,0.2775]).
- Demoed the full report generator end-to-end with dummy 4-run data (confirmed `.md`/`.csv`/`.json` writing works), then deleted the dummy output so it can't be mistaken for step11's real results.
- Design choices made explicit: `false_brake_incl_phantom_rate` formula (combines both numerator and denominator across the two underlying rates); eval split-guard logs both successful AND refused attempts.

## step04a_lidar_adapter — APPROVED
- Built: `src/ttcf/adapters/lidar.py` (RANSAC ground removal + DBSCAN clustering + DEC-1 reference point), `scripts/measure_sensor_R.py`, `scripts/lidar_adapter_evidence.py`, `scripts/run_pipeline.py` extended with `run_real_stream`/`score_real_stream` + `--stream lidar --split tune` CLI.
- Decisions: DEC-1 and the clustering-library choice already settled, consumed here. **New, to ask**: whether to promote `SENSOR_R_LIDAR` ASSUMED→MEASURED (81.2% inlier rate, N=16 — thin sample, recommend not yet).
- Precedent check: V1's manual RANSAC ground-plane fit and V1's tuned DBSCAN params (`eps=0.7, min_samples=10`) ported; V1's centroid reference point and size/shape cluster filtering explicitly NOT ported (DEC-1, DEC-2). No R-measurement-script precedent in V1/v2; built fresh, carrying forward only V1's own lesson (item A.10) that a measured-looking R still needs human judgment before being trusted.
- Tests: 117/117 passing (110 prior + 7 this step).
- **Real-data bug found and fixed (G13)**: ego-vehicle LiDAR self-returns (9,805 points at point-blank range, sensor-mount height) were being clustered as a phantom "object." Root-caused via largest-cluster coordinate inspection (not just accepted as noise), fixed with a new `LIDAR_MIN_RANGE_M=3.0` ROI exclusion, verified against multiple scenes and the full synthetic test suite.
- **Process error found and fixed**: the real-data evidence script (and my own diagnostics) had been using `scene-1077`, which `configs/split.json` actually assigns to the EVAL split — a direct violation of step04a's own "do not evaluate on the eval split" rule. Caught before the step was reported, corrected to a tune-split scene (`scene-1094`), affected figures regenerated.
- First real LiDAR-only run (tune split, 5 scenes, native rate): n_scored=79, correct_action_rate 94.9% (all-GT) / 100% (obs_lidar-restricted), false_brake_rate 0.0% both variants, missed_brake_rate 100% (4/4, indicative-only, all-GT) / n/a (obs_lidar-restricted). Investigated the missed-brake gap (G13): all 4 tune-split danger events are the same fast-closing car across 4 consecutive keyframes with `num_lidar_pts`=3,0,0,0 — a genuine LiDAR blind spot on this one event (radar/camera did observe it), not a bug, exactly the case the observable-restricted metric exists to separate out fairly.
- Documented, not filtered: real keyframes show more adapter candidate detections than in-path GT objects even after the self-return fix, traced to static roadside structures (walls/fences) correctly passing the generous, position-only candidate gate — step04a §4 explicitly forbids adding size/shape filtering to suppress this (DEC-2), and false-brake rate stayed 0% regardless since static objects never show a closing TTC.

## step04b_radar_adapter — APPROVED
- Built: `src/ttcf/adapters/radar.py` (DBSCAN eps=1.5m/min_samples=1, devkit-default point filters only, DEC-1 reference point reused from lidar.py), `scripts/measure_sensor_R.py` generalized with `--sensor {lidar,radar}`, `scripts/radar_adapter_evidence.py`, `scripts/run_pipeline.py` generalized to support a stream backed by multiple merged channels (`--stream radar`).
- Decisions: **ghost-return filtering — confirmed before coding**: use nuscenes-devkit's own default radar point filters only, no extra `dyn_prop` stationary-clutter filter (diverges from V1 on purpose) — step04b frames ghost returns as a test of the downstream pipeline, not something to hide upstream. **New, to ask**: promote `SENSOR_R_RADAR` ASSUMED→MEASURED (92.0% inlier rate, N=25 — stronger footing than LiDAR's own case, stress-tested against a real confound and held up; see report §3).
- Precedent check: V1/V2's DBSCAN clustering (eps=1.5m, min_samples=1) ported, per-channel-per-scan (V2's variant); V1/V2's centroid reference point NOT ported (DEC-1 project-wide); V1's extra dyn_prop filter NOT ported (decision above); Doppler NOT wired into TTC/KF anywhere (already G17).
- Tests: 124/124 passing (117 prior + 7 this step).
- **Real-data confound found and fixed (G13)**: radar's own R-measurement initially pooled 40-136 unfiltered candidate detections per keyframe (3 channels, no ROI crop, ghost returns unfiltered — both deliberate), risking a "nearest of many looks accurate by chance" bias. Fixed by restricting the measurement to `candidate_gate`-passing detections (same filter the real pipeline already applies) — result was numerically IDENTICAL after the fix, strong evidence the original measurement wasn't actually biased.
- First real radar-only run (tune split, 5 scenes, native rate, 3 channels merged): correct_action_rate 94.9% (all-GT) / 98.7% (obs_radar-restricted), false_brake_rate **1.33% (1/75)** — traced (G13) to a specific `movable_object.barrier` in a construction-zone scene, deliberately excluded from GT scoring (DEC-2) but a real, physically-present stationary object with a genuinely shrinking TTC as ego approached it — exactly the "ghost return" phenomenon step04b §3 asked to be measured, now fully attributed via detection-level `aux` (stationary-point-count, near-zero compensated Doppler) plus a full-annotation cross-check. missed_brake_rate 75% (3/4, all-GT) / 0% (0/1, obs_radar-restricted) — same cross-sensor observability-gap pattern already found for LiDAR on the same 4 danger events (only 1 of 4 had any `num_radar_pts>0`).

## step04c_camera_adapter — APPROVED
- Built: `src/ttcf/adapters/camera.py` (YOLOv8n cached-detection lookup + ground-plane back-projection, DEC-6), `scripts/cache_camera_detections.py`, `scripts/measure_sensor_R.py` generalized further with `--sensor camera`, `scripts/camera_adapter_evidence.py`, `scripts/run_pipeline.py` extended with `--stream camera`.
- Decisions: detector = Ultralytics YOLOv8n/CPU, DEC-6 = ground-plane back-projection — both confirmed before coding (GPU too weak to help; DEC-6 has zero V1/V2 precedent, built fresh).
- Precedent check: found and confirmed the exact V1 mistake step04c's spec warns about — V1's own `camera` row secretly used median LiDAR-point depth inside each 2D box (not a real single-sensor result); `camera_mono` (similar-triangles depth) was V1's genuine correction. Neither depth method was ported — DEC-6 chose ground-plane back-projection instead, structurally prevented from ever leaking other-sensor data by `tests/test_camera_isolation.py`.
- Tests: 133/133 passing (124 prior + 9 this step).
- **Real dependency conflict found and fixed**: installing `ultralytics` broke `nuscenes-devkit`/`scipy` via a numpy version bump, and a subsequent opencv-python/opencv-python-headless coexistence silently corrupted the shared native `cv2` module. Root-caused and fixed (not papered over) before writing any camera code; full 124-test suite re-verified clean first.
- **Real geometry bug found and fixed in the test suite itself (G13)**: every synthetic camera test initially used an identity rotation for the camera's own extrinsic calibration, which is not physically valid (a camera's own axis convention differs from ego's) and made rays point the wrong way. Traced to the test setup, not the adapter; fixed by reusing nuScenes' own real CAM_FRONT calibration quaternion in every synthetic test.
- `GROUND_Z_EGO=0.0` sanity-checked directly against 7,858 real GT objects (median bottom height 0.049m) — no recalibration needed.
- `SENSOR_R_CAMERA` measured (27.3% inlier rate, N=88) and correctly stays ASSUMED — well below the 70% bar, not a close call, consistent with monocular depth's genuinely lower accuracy.
- Real-data evidence: depth error empirically confirmed growing with range² (median 1.78m at 0-15m → 17.29m at 50-200m, 119 keyframes checked).
- First real camera-only run (tune split, 5 scenes): correct_action_rate 94.9%, false_brake_rate 0.0%, missed_brake_rate 100% (4/4) — **identical** on both the unrestricted and camera-observable-only variants (unlike LiDAR/radar), investigated (G13) and traced to a materially different story: camera *could* see all 4 danger events (per GT visibility), but missed them due to a mix of imperfect frame-to-frame detector recall and depth noise occasionally pushing a real in-path object's estimated position outside the corridor — including one concrete example of the ground-plane method's known near-horizon numerical instability (a box "detected" 1,696m away, harmless here but flagged for a future numeric-stability guard).

## step11_runs_tuning_report — reported (awaiting final approval)
- Built: `scripts/run_pipeline.py` generalized with a "fused" stream (union of all active channels, one shared tracker, early fusion) and `--split eval --final` support; `scripts/tune.py` (SIGMA_A grid sweep + direct threshold measurement); `tests/test_run_equivalence.py` (6 tests); `config.shared_config_hash()`/`current_config_hash()` added.
- Decisions: **DEC-7 pre-declared before any sweep** — mean correct_action_rate across all 4 streams, missed_brake_rate as tie-breaker.
- Tests: 139/139 passing (133 prior + 6 this step's own — 2 pre-existing tests needed fixture updates after threshold values changed, not code bugs).
- **All remaining PLACEHOLDER config values resolved**: `SIGMA_A=0.5` (grid sweep, won outright on DEC-7's criterion), `MAX_VEL_STD_MPS=3.90`, `MIN_TRUSTED_SPEED_MPS=1.99`, `MIN_CLOSING_SPEED_MPS=1.79` (all three measured directly from real tune-split data, not a scoring sweep).
- **Three real bugs found and fixed in the measurement methodology itself (G13), documented in full in `docs/decisions.md`**: (1) raw stationary-object jitter contaminated by freshly-spawned, under-converged tracks giving nonsensical ~14 m/s "noise floors" — fixed by restricting to already-eligible-looking samples; (2) residual contamination traced to the diagnostic's own crude position-only GT-to-track matching picking up unrelated nearby moving tracks in crowded scenes — fixed with a tighter, unambiguous association; (3) `MIN_CLOSING_SPEED_MPS`'s own formula was conflating ego's real motion (not noise) with actual sensor jitter — fixed by isolating only the object's own estimated velocity component. Even after all fixes, the cleaned sample stayed visibly bimodal (n=38) — used the 80th percentile at the natural break rather than an unsupportable 95th percentile, stated as an honest small-sample limitation.
- Fused-stream sanity check confirmed real multi-sensor fusion is happening (`multi_sensor_fraction` 0.10-0.42, nonzero), and produced a genuinely rich, honest finding after full tuning: fused's `missed_brake_rate` dropped from 75% (pre-tuning) to **25%** (now catching 3 of 4 tune-split danger events — beating every single sensor: lidar 100%, radar 50%, camera 100% missed), but its `false_brake_rate` rose to 13.3% (10/75) — a real, reportable trade-off surfaced by honest measurement, not chased or tuned away (no parameter was tuned specifically for fusion, per step11 §8's own rule).
- Config frozen (SHA-256 `58709abd...6c5`), freeze approved by Siva, eval split scored once per stream with `--final` (one documented exception: camera/fused crashed on a missing eval-split cache before any scoring occurred, not a re-run of a completed score — fixed and re-attempted, full detail in `docs/decisions.md`).
- **Final eval-split result: fusion did NOT beat the best single sensor.** LiDAR-only: 97.0% correct / 0.8% false-brake (best of all 4). Fused: 87.2% correct / 11.1% false-brake (worst of all 4 on both), but tied LiDAR for best missed-brake rate (28.6%) — a real trade-off (fusion catches more danger events, at a real false-alarm cost), not a one-sided loss. Investigated (G13): half the fused false brakes trace to one scene (a crowded parking lot, "many peds, parked bicycles/scooters") — the project's own already-documented crowd-conflation limitation (step06), not a new bug.
- All remaining PLACEHOLDER values resolved (see the tuning-results entry above); config status: 0 MEASURED, 50 ASSUMED, 1 PLACEHOLDER (`SENSOR_R_CAMERA`, intentionally a function not a static value).
- Delivered: `reports/final_report.md` (full 8-section report per step11 §6), `docs/knowledge_share.md` (portfolio-ready teaching write-up per step11 §7), `reports/step11_report.md` + `docs/learning/step11_learning.md` (this step's own standard deliverables).
- Tests: 139/139 passing.
- Waiting for Siva's final review (step11 §11 STOP).

## post-step11_doppler_and_nis — exploratory, reported (tune split only)
- Prompted by Siva's request to check what changed recently in the predecessor project (`F:\Sensor fusion Research`, commits through `06166f2`). Investigated its own fusion-underperformance findings in depth (two passes, including a dedicated fork agent digging into `CLAUDE.md`, `docs/*.docx`, `config.py`, `src/geometry.py`). Conclusion: this project's own architecture (early fusion, Mahalanobis/covariance gating from day one, native per-event timestamps, devkit-default-only radar filtering) already avoids nearly every bug class found there (declutter-filter bug, FusedTracker identity fragmentation, fixed→covariance weighting, camera double-counting). Two items were genuinely new and architecture-agnostic: radar Doppler velocity as a second measurement channel, and an NIS filter sanity check. Both approved by Siva ("both").
- Built: `ConstantVelocityKF.update()` generalized to accept an optional observation matrix `H` (defaults to the unchanged position-only `_H`; `H4` = full-state identity for a combined position+velocity update) — gating (`innovation_cov`/`mahalanobis_sq`) deliberately left position-only regardless. `sensor_to_ego_vector`/`ego_to_global_vector`/`sensor_to_global_vector` added to `transforms.py` (translation-free, w=0 homogeneous-vector rotation, reusing the existing point-transform chain). `Detection` extended with optional `velocity_global`/`R_velocity_global` fields (None by default everywhere). `config.RADAR_DOPPLER_ABLATION` (default False) and `RADAR_VELOCITY_NOISE_VAR_MPS2` added. `adapters/radar.py` populates the two new fields only when the ablation flag is True (rotates `vx_comp`/`vy_comp` sensor-frame Doppler to global). `tracker.py._kf_update_args()` builds the combined 4D measurement generically off `Detection`'s own fields (no sensor-name check), called from `process_scan`'s matched-update path only (spawns stay position-only, per step06's existing spec).
- Tests: 141/141 passing (135 prior + 6 new: 2 KF combined-update tests, 2 transform vector-rotation tests, 2 tracker `_kf_update_args`/end-to-end Doppler tests) — confirmed bit-identical to the pre-change 135 on every existing call site (default `H=None`/`velocity_global=None` path).
- `scripts/nis_check.py` (new): reuses the tracker's own real matched-update `d2` log (`AssociationLogEntry.d2`, already exactly NIS, df=2) — no new tracker instrumentation needed. Result (tune split, 5 scenes, default config): every stream mildly-to-moderately UNDERCONFIDENT vs. chi2(df=2) theory (lidar mean 1.070, radar 0.847, camera 1.139, fused 1.243, all vs. theoretical mean 2.0); 0.00% of matched updates in any stream exceeded `CHI2_GATE`. Not a bug — consistent with `SENSOR_R_*` being conservatively-chosen ASSUMED values and `SIGMA_A` tuned via DEC-7's event-level criterion, not NIS. No config change.
- `scripts/doppler_ablation.py` (new): radar and fused streams, tune split, `RADAR_DOPPLER_ABLATION` False vs. True, same 5 scenes, same code path. Result: **radar-alone** improves outright (correct_action_rate 93.7%→100%, false_brake_rate 4.0%→0%, missed_brake_rate 50%→0%, n_scored=79) — matches the predecessor project's own standalone-radar finding. **Fused** is a wash (correct_action_rate 86.1%→84.8%, false_brake_rate unchanged 13.3%; the missed_brake 25%→50% move is a single-event flip on a denominator of 4, not a real signal) — LiDAR already dominates the fused stream (97% correct per step11's frozen result) and radar's own Doppler adds nothing further once LiDAR is present.
- Decision: `RADAR_DOPPLER_ABLATION` stays an ablation, default False — does not meet the bar to touch `configs/frozen_config.json` or step11's official (fused) result. Worth revisiting only for a future radar-alone-robustness scenario (e.g. degraded visibility). Full findings and numbers in `docs/decisions.md` (2026-10-08 entries) and `outputs/tables/{nis_check,doppler_ablation}.json`.
