# Step 04b report — Radar adapter (Doppler OFF) + first radar-only run

## What was built (files, one line each)
- `src/ttcf/adapters/radar.py` — `process_radar_scan`/`radar_detections`: RawEvent(radar channel) → list[Detection]. Load (devkit's own default filters) → sensor→ego transform → DBSCAN clustering in xy (eps=1.5m, min_samples=1) → DEC-1 nearest-surface reference point per cluster, Doppler/RCS/stationary-point-count stored in `aux` only (G17 — never consumed by the tracker).
- `scripts/measure_sensor_R.py` — generalized with `--sensor {lidar,radar}`; radar path pools all 3 DEC-3 radar channels per keyframe, filters through `candidate_gate` before matching (fix described in §3).
- `scripts/radar_adapter_evidence.py` — real-data BEV overlays + raw-points-per-object vs. clusters-per-object statistics, 3 tune-split scenes.
- `scripts/run_pipeline.py` — generalized `_stream_adapter` to support a stream backed by multiple merged channels; `--stream radar` now works alongside `--stream lidar`.
- `tests/test_radar_adapter.py` — 7 synthetic tests (all pass), including a devkit-default-filter check not in step04b's own numbered list.
- `outputs/figs/step04b/bev_*.png` (3 scenes, tune split).
- New `config.py` params (all ASSUMED, tunable_on="tune_split"): `RADAR_CLUSTER_EPS_M=1.5`, `RADAR_CLUSTER_MIN_SAMPLES=1`. Reused `NEAREST_SURFACE_RADIUS_M` unchanged (project-wide, DEC-1).

## Precedent check (V1/v2) — done before coding
- **PORTED**: DBSCAN clustering, `eps=1.5m` (2x LiDAR's 0.7m), `min_samples=1` (V1 `Step_3_2_Radar_Fusion.ipynb`; V2 `config.py RADAR_CLUSTER_EPS_M`). Clustered **per-channel-per-scan** (V2's variant, not V1's pooled-across-5-channels variant) — V2's own stated reason (pooling mixes points from different real timestamps/channels) agrees with this project's own G2 rule.
- **NOT PORTED**: V1/V2's centroid reference point — reused `lidar.py`'s `_cluster_reference_point` (nearest-surface-to-ego) unchanged; DEC-1 is a project-wide, cross-sensor decision, not sensor-specific.
- **NOT PORTED**: V1's extra `dyn_prop ∈ {0,2,6}` stationary-clutter filter on top of the devkit's own defaults — see the decision below, confirmed with Siva before coding.
- **NOT PORTED**: Doppler wired into any filter/TTC/KF — already this project's own G17, consistent with V2's own R10 rule.
- **NOT PORTED**: V1's own R-measurement method for radar (found unusable per lessons-from-v1-v2.md item A.10: 2.4x fragmentation, ~86% inlier rate) — `measure_sensor_R.py --sensor radar` re-measures fresh against this project's own adapter/tracker instead.

## Decision taken before coding: ghost-return filtering
V1 added an extra `dyn_prop ∈ {0,2,6}` filter on top of the devkit's own defaults, specifically to suppress stationary clutter (guard rails, signs, manhole covers) before it ever reached its tracker. I verified the devkit's actual defaults directly: `invalid_states=[0]`, `dynprop_states=range(0,7)` (no dynprop filtering at all), `ambig_states=[3]` — V1's filter was its own addition, not something "the devkit already offers" (the distinction step04b §3 step 1 itself draws).

step04b's own spec frames ghost returns as a test of the **downstream** pipeline (forward-path filter, tracker, debounce) — "the pipeline's only defences; report how many false brakes... trace to such returns" — and explicitly says "do not hack around silently." Confirmed with Siva before writing any code: **use devkit defaults only, no extra filter.** Diverges from V1 on purpose. §4 below reports exactly what this choice costs in practice.

## §1 — Real-data statistics (`scripts/radar_adapter_evidence.py`, 3 tune-split scenes, all keyframes)
1,515 GT objects (any keyframe, `vehicle.*`/`human.pedestrian.*`, `num_radar_pts>0`) across scene-0103/0061/1094:
- Mean devkit-reported `num_radar_pts`: 2.55 (confirms radar's own sparsity — median across the whole tune split is 0).
- Mean adapter clusters within 5m of each GT object: 1.39.
- 653/1515 (43%) objects have **zero** nearby clusters — consistent with `num_radar_pts` itself counting points before the devkit's own validity/ambiguity filters are applied, so some "misses" are points that were correctly discarded as invalid, not adapter failures; not independently verified per-object here, stated as a limitation of this coarse diagnostic.
- 609/1515 (40%) objects have **more than one** nearby cluster — expected given `min_samples=1` (near-zero point-merging) and the 5m association radius being generous enough to sometimes span two real, separate nearby objects (e.g. two parked cars), not necessarily true fragmentation of one object. Stated as a known limitation of this diagnostic's coarse radius, not a precise fragmentation measurement.
- Per-keyframe (3-channel pooled): mean 103.3 raw points → 87.7 clusters — `min_samples=1` compresses very little, as expected (radar returns are already spread far apart relative to `eps=1.5m`).

BEV figures show plausible spatial correspondence between detections and GT boxes along the road layout; one figure (scene-0061) shows a visible diagonal line of "out of candidate gate" detections consistent with a guard rail or road-edge structure — a direct visual instance of the ghost-return phenomenon named in step04b §3.

## §2 — Devkit-default-filter check
Added a test (`test_devkit_default_invalid_state_filter_is_respected`, not in step04b's own numbered list) directly verifying the module docstring's claim: a synthetic point flagged `invalid_state=2` (near-field artefact) never reaches clustering, confirming the devkit's own default (`invalid_states=[0]`) is doing real filtering work, not a no-op.

## §3 — Measuring R (`scripts/measure_sensor_R.py --sensor radar`, §6)
Scope: tune-split, in-path GT objects with `num_radar_pts >= 3` (radar's own sparsity means LiDAR's `>=50` bar would leave almost nothing — the tune split's own distribution has only 3 objects at `>=5`; `>=3` gives 25, a larger sample than LiDAR's own 16).

**First result (before a fix): 92.0% inlier rate (23/25), σ_long=0.305m, σ_lat=0.334m** — σ_lat notably tighter than the current ASSUMED (1.5m). Investigated as a surprising number (G13) before accepting: dumped the raw candidate pool size per keyframe and found **40-136 radar detections per keyframe** (pooled from 3 channels, no ROI crop, ghost returns not filtered — both deliberate). With that many candidates, "nearest of many" could look falsely accurate regardless of true sensor precision — a real confound, not just a disclosed limitation.

**Fix applied**: restricted both sensors' `measure_sensor_R.py` matching to `candidate_gate`-passing detections only (the same filter the real pipeline already applies before the tracker) — this cut the candidate pool from 40-136 down to 0-4 per keyframe. **Re-ran: the result was numerically identical** (92.0%, 23/25, same σ values to 3 decimal places, only difference: 1 additional recorded "total miss" that was previously masked by an out-of-corridor detection winning the naive nearest-match). This is reassuring evidence the original result was not a many-candidates artifact — the genuinely correct nearest detection was already the one being matched. σ_lat=0.334m is also physically plausible on its own terms: in-path objects sit near the sensor's boresight, where angular resolution is best (~1° resolution × ~20m typical range ≈ 0.35m, close to the measured value) — unlike a full-FOV average, which is what the literature-typical ASSUMED 1.5m more likely reflects.

**Not applied automatically** — per the same rule as step04a. This result is on somewhat stronger footing than LiDAR's own (N=25 vs 16, and specifically stress-tested against a real confound that held up), but the same structural limitation remains: nearest-neighbour matching, not real tracked-object identity, and this would be the project's first-ever MEASURED param, feeding directly into tracker gating system-wide. **Decided: `SENSOR_R_RADAR` stays ASSUMED** (confirmed by Siva) — revisit at step11 once more real data/tracking exists. See `docs/decisions.md`.

## §4 — First radar-only run (tune split, `--stream radar --split tune`)
Command: `.venv\Scripts\python.exe scripts\run_pipeline.py --stream radar --split tune`. All 5 tune scenes, native-rate radar stream (3 channels merged chronologically), scored against every tune-split keyframe with ≥1 in-path GT object (n_scored=79).

| Variant | n_scored | correct_action_rate | false_brake_rate | missed_brake_rate |
|---|---|---|---|---|
| all (unrestricted GT) | 79 | 94.9% (75/79) [0.88, 0.98] | **1.33% (1/75) [0.00, 0.07]** | 75% (3/4) [0.30, 0.95] — indicative only |
| obs_radar (radar-observable GT only) | 79 | 98.7% (78/79) [0.93, 1.00] | 1.28% (1/78) [0.00, 0.07] | 0% (0/1) [0.00, 0.79] — indicative only |

`no_estimate_reason_breakdown` (all variant, the 3 missed brakes beyond the observable one): `NOT_ELIGIBLE`=1, `NONE_IN_PATH`=2.

Tracker counters per scene (matches/spawns/evictions — much higher than LiDAR's own run, consistent with radar's higher native rate and the deliberately un-filtered candidate stream):
| scene | matches | spawns | evictions |
|---|---|---|---|
| scene-0103 | 349 | 41 | 40 |
| scene-0061 | 1041 | 133 | 118 |
| scene-0655 | 35 | 20 | 18 |
| scene-0796 | 220 | 52 | 46 |
| scene-1094 | 657 | 122 | 120 |

**Surprising number investigated (G13): the one false brake — exactly the "ghost return" phenomenon step04b §3 asked me to attribute.** Traced it to `scene-0061`, keyframe `88449a5c`, driving track `trk_000046`, which fired GRADUAL. Instrumented a diagnostic to record each matched detection's `aux` alongside the tracker's own association log: the feeding detections consistently had `n_stationary_pts == n_points` (every point in the cluster flagged `dyn_prop` stationary/stationary-candidate/crossing-stationary) and near-zero ego-motion-compensated Doppler (`vx_comp`/`vy_comp` ≈ 0) throughout — a genuinely stationary radar return, not sensor noise. Its apparent range shrank from ~18m to ~14m over ~1.1 seconds as ego drove toward it — physically correct behavior for ego approaching a fixed object (a real, legitimately shrinking TTC, not a tracking artifact). Cross-checked against every real annotation (any category, not just GT-scoped ones) near that position at that keyframe: this is a **construction-zone scene with numerous `movable_object.barrier`/`movable_object.trafficcone` annotations**, including one at ego-frame (16.17, 0.80) — directly matching the track's position. `movable_object.*` is deliberately excluded from `GT_CATEGORIES` (DEC-2: "typically static clutter"), so this is a real, physically-present static object that the evaluation's own GT scope reasonably excludes from being a "true positive" — not a bug, and exactly the textbook ghost-return case the step warned about. **Conclusion: 1/75 (1.33%) false brakes in the radar-only tune-split run trace to stationary-clutter ghost returns, all defenses (candidate gate, tracker, debounce) still let through, quantifying the real cost of the "no upstream filter" decision.**

Also investigated: `missed_brake_rate` gap between the two variants (75% "all" vs 0% "obs_radar") — same root cause pattern as step04a: only 1 of the tune split's 4 danger events had `num_radar_pts>0` (the fast-closing car in scene-0103, which had `num_radar_pts=0,2,0,0` across its 4 danger keyframes — radar observed it at only one of those 4 instants), and the system correctly scored 0% missed on the (single) event it could actually see. Not a bug — a genuine, small-sample cross-sensor observability gap, consistent with step04a's own finding for LiDAR on the same events.

No fusion run yet (camera adapter arrives at step04c) — this is a single-modality baseline only, not compared against LiDAR's own run in this report (both exist now, but the formal 4-run comparison table is step11's job).

## Tests
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
124 passed, 14 warnings in 51.57s
```
All 7 of this step's tests pass, alongside the 117 from prior steps — no regressions.

## Real-data figures
`outputs/figs/step04b/bev_scene-0103_5b03af7a.png`, `bev_scene-0061_b26e7915.png`, `bev_scene-1094_0e837c21.png` — one representative keyframe per tune-split scene, GT boxes (green squares) vs. adapter detections (gray = outside the candidate gate, red = in-path candidate).

## Flagged, not built (out of scope here)
- Camera adapter (`--stream camera`) — step04c.
- Fused run and the 4-run comparison table — needs all 3 adapters, arrives at step11.
- Any tuning of `RADAR_CLUSTER_EPS_M`/`RADAR_CLUSTER_MIN_SAMPLES` against outcomes — explicitly out of scope for this step (same rule as step04a §7).
- Doppler ablation — requires Siva's explicit separate approval (G17), not attempted here.
