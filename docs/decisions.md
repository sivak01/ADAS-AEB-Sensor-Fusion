# Decisions log

One entry per decision, in the order answered. Never edit a past entry's
choice/reason after the fact — add a new entry if a decision is revisited.

---

**2026-09-20 — DEC-3 — Active channels**
Choice: `LIDAR_TOP, RADAR_FRONT, RADAR_FRONT_LEFT, RADAR_FRONT_RIGHT, CAM_FRONT`.
Reason: approved as proposed (README_MASTER.md §6) — forward corridor only,
which is all a forward-TTC/AEB design needs; no side/rear sensors.

**2026-09-20 — DEC-9 — Initial ASSUMED/PLACEHOLDER thresholds**
Choice: the full starting-values table in `step00_config_contracts.md` §4,
approved as proposed, with no changes.
Reason: every value is ASSUMED or PLACEHOLDER, none MEASURED; each is named
for re-measurement in a specific later step (see `src/ttcf/config.py`'s
per-parameter `reason` field for the exact justification and, where
relevant, why it deliberately does NOT inherit a V1/V2 numeric value —
lessons-from-v1-v2.md item C.1).

**2026-09-24 — step04b — SENSOR_R_RADAR MEASURED promotion**
Choice: **stays ASSUMED** (Siva confirmed, matching the recommendation).
Finding behind the question: `scripts/measure_sensor_R.py --sensor radar`
(tune split, GT objects with `num_radar_pts>=3`) found a 92.0% inlier rate
(23/25, residual<2.0m) — clears step04b §6's 70% MEASURED threshold, with a
larger sample than LiDAR's own (N=25 vs N=16). Robust (MAD) σ_long=0.305m,
σ_lat=0.334m, vs. the current ASSUMED (0.5m, 1.5m) — σ_lat is much tighter
than the literature-typical assumed value, investigated (G13) and found
physically plausible (in-path objects sit near boresight, where angular
resolution is best; ~1° resolution at ~20m typical range ≈ 0.35m, matching).
Also stress-tested against a real confound: the pooled 3-channel candidate
pool (40-136 detections/keyframe, unfiltered) could make "nearest of many"
look falsely accurate — re-ran after restricting to `candidate_gate`-passing
detections (same filter the real pipeline applies) and got an IDENTICAL
result, evidence against that bias. See `reports/step04b_report.md` §3.
Reason not promoted despite the stronger sample: same fundamental
limitation as LiDAR's case (nearest-neighbour matching, not real tracked
identity), and this would be the project's first-ever MEASURED param — R
feeds directly into tracker gating (Mahalanobis, S=P_pred+R) system-wide,
so an overconfident value has broad downstream effects. `config.py` left
unchanged. Revisit at step11 once more real data/tracking exists.

**2026-09-25 — step11 — Eval-split camera/fused runs crashed on a missing prerequisite, not scored, re-attempted**
Finding: the first `--final` eval-split attempt (all 4 streams) succeeded
for lidar and radar, but camera and fused both crashed with
`FileNotFoundError` inside the camera adapter — `scripts/cache_camera_detections.py`
had only ever been run with `--split tune` in this project so far; the
eval split's own CAM_FRONT frames were never cached. Confirmed via
`outputs/eval_runs.log`: all four streams passed the guard (logged `OK`
at the same frozen-hash check), but camera/fused's own timestamps are
~2 seconds apart — far too fast for any real scoring to have happened;
the crash occurred inside the adapter, upstream of any GT comparison.
No detections, tracks, or scores were ever produced for camera/fused on
the eval split before this failure.
Action: NOT treated as "re-running the eval split to fix a bug" (step11
§5.4's own rule) — no actual scoring occurred for these two streams the
first time, so there is nothing to redo, only an incomplete prerequisite
(the eval-split cache) to complete before the real run can happen at
all. Recorded here plainly per the same rule's spirit regardless. Fixed
by running `scripts/cache_camera_detections.py --split eval`, then
re-attempting `--stream camera --final` and `--stream fused --final`
only (lidar/radar's own already-successful eval results are kept, not
re-run).

**2026-09-25 — step11 — Freeze approved**
Choice: `configs/frozen_config.json` (SHA-256 `58709abded7796ead9a7b086f82484e02ef7527aae165a6e1fae7e6fe96926c5`,
51 params) approved by Siva for the final eval-split run.
Reason: all tuning (SIGMA_A sweep + 3 direct threshold measurements) is
complete and logged (see the tuning-results entry below and
`outputs/tuning_log.csv`); no PLACEHOLDER values remain except
`SENSOR_R_CAMERA` (a function, not a static value, by design). Per
step11 §5.1's own protocol, this is the one hard gate in this step — the
eval split is scored exactly once, immediately after this approval, with
`--final` and this exact frozen hash enforced by `guard_eval_scoring`.

**2026-09-24 — step11 — Tuning results: all remaining PLACEHOLDER values resolved**
Choice: `SIGMA_A=0.5` (log-spaced grid {0.5,1,2,4,8} across all 4 streams,
selected by DEC-7's mean-correct-action-rate criterion, no tie-breaker
needed — 0.5 won outright at 0.9430 vs. 0.9335-0.9399 for the rest);
`MAX_VEL_STD_MPS=3.90` (75th pct of the KF's own velocity-std population,
n=47,336, all 4 streams pooled); `MIN_TRUSTED_SPEED_MPS=1.99` and
`MIN_CLOSING_SPEED_MPS=1.79` (80th pct of stationary-GT-object velocity
jitter, n=38, after two rounds of G13 investigation below). Full sweep
logged in `outputs/tuning_log.csv`.
Real bugs found and fixed in the measurement methodology itself before
trusting any of these three thresholds (G13 — investigated as bugs first,
not accepted as results):
1. The raw stationary-object jitter sample was dominated by freshly-
   spawned tracks at exactly `MIN_UPDATES_FOR_TTC` (2) updates, where
   measurement noise divided by a tiny native-rate dt implies an
   enormous, meaningless velocity (95th pct was 13.9 m/s — literally
   highway speed, for a "sensor noise floor"). Fixed by restricting to
   samples whose own KF `vel_std` already clears `MAX_VEL_STD_MPS`
   (computed first, precisely the population the real eligibility gate
   lets through) — this alone only partially helped (95th pct still 12.2).
2. The remaining contamination traced to the diagnostic script's own
   crude nearest-position GT-to-track matching (radius 3.0m) picking up
   a DIFFERENT, genuinely-moving nearby track instead of the track
   actually tracking a given stationary object — confirmed directly (a
   track with 7 real updates and a modest vel_std still reporting ~10 m/s
   next to a parked car, in a scene with several nearby objects). Fixed
   with a tighter (0.75m) and unambiguous (next-nearest candidate must be
   >=1m farther) association.
3. A third, conceptual fix for `MIN_CLOSING_SPEED_MPS` specifically: the
   full closing-speed formula includes ego's own REAL motion relative to
   a fixed point (often several m/s, not noise — ego's own velocity is
   precisely known, not itself jittery), so measuring "closing speed
   jitter" via the full formula gave a contaminated ~5 m/s median. Fixed
   by isolating only the object's own estimated velocity (which should be
   exactly 0 for a truly stationary object) projected onto the line of
   sight — the actual noise contribution, not ego's real motion.
Even after all three fixes, the cleaned sample (n=38) is visibly bimodal
(a tight cluster <2 m/s, then a gap, then a handful of still-residual
outliers) — a real, honestly-reported limitation of trying to isolate
one specific track's identity via position-only matching in a crowded
real scene. The 80th percentile (sitting at the natural break) was used
instead of the 95th, which a sample this size cannot support robustly.
All four values recorded ASSUMED (not MEASURED — reserved for direct
empirical measurement like sensor R), with full reasoning in each
param's own `config.py` reason field.

**2026-10-08 — post-step11 — G17: radar Doppler velocity ablation, approved**
Choice: build a genuine radar-Doppler-as-second-measurement-channel
ablation (Siva approved explicitly, 2026-10-08) PLUS an NIS filter
sanity check, both prompted by a deeper review of the predecessor
project's own recent work (`F:\Sensor fusion Research`, commits through
`06166f2`).
Reason: the predecessor project's own largest unambiguous accuracy win
(radar standalone MAE 5.53→3.80, −31%) came from feeding radar's own
Doppler velocity into its filter as real information, instead of
deriving velocity from position differences alone (which this project
already does, G17 default). Re-checked the rest of that project's recent
findings against our own architecture first (full report in-session,
2026-10-08) — almost everything else (the radar declutter filter fix,
FusedTracker identity bugs, fixed→covariance-based weighting, camera
double-counting, per-sample shared timestamps) does NOT apply, because
this project's own architecture (early fusion, Mahalanobis/covariance
gating from day one, native per-event timestamps, no extra radar filter)
already avoids those exact bug classes by design. Doppler and NIS were
the two genuinely new, evidence-backed, architecture-agnostic items.
Scope: an explicitly separate, clearly-labelled ABLATION — `RADAR_DOPPLER_ABLATION`
defaults to False for every official result already reported (step04b
through step11's frozen config); Doppler-ON is never silently the
default, and this work does NOT touch `configs/frozen_config.json` or
re-run the eval split (tune split only, exploratory).

**2026-10-08 — post-step11 — NIS filter sanity check, results**
Ran `scripts/nis_check.py` (tune split, 5 scenes, `RADAR_DOPPLER_ABLATION=False`
— the default config) — analyses the real matched-update `d2` values the
tracker already logs (`AssociationLogEntry.d2`, exactly NIS, df=2, since
gating is always position-only regardless of G17's state).
Finding: every stream's mean/median NIS sits BELOW the chi2(df=2) theoretical
values (mean=2.0, median=1.386) — lidar mean=1.070/median=0.281 (n=1873),
radar mean=0.847/median=0.202 (n=2297), camera mean=1.139/median=0.496
(n=617), fused mean=1.243/median=0.505 (n=4914); 0.00% of matched updates
in any stream exceeded `CHI2_GATE` (nominal 1%).
Interpretation: the filter is mildly to moderately UNDERCONFIDENT across
every stream (P/R together somewhat overestimate real error) — never
overconfident, and never anywhere near the gate's own false-rejection
risk. Radar is the most underconfident (mean 0.847, ~42% of theory),
consistent with `SENSOR_R_RADAR` being an ASSUMED (not MEASURED) value
(see step04b's own precedent-check note) chosen conservatively. Not a
correctness bug: a somewhat-conservative gate costs some fragmentation,
not false associations, and `SIGMA_A`/`SENSOR_R_*` were deliberately
tuned via DEC-7's own event-level criterion, not NIS — the two can
reasonably disagree without either being wrong. No config change made;
recorded as a sanity finding only, per this entry's own stated scope.
Full numbers: `outputs/tables/nis_check.json`.

**2026-10-08 — post-step11 — G17 radar Doppler ablation, results**
Ran `scripts/doppler_ablation.py` (tune split, 5 scenes, radar and fused
streams only — lidar/camera are bit-identical regardless of this flag)
with `RADAR_DOPPLER_ABLATION` False then True, same scenes, same code
path, side by side.
Finding: **radar-alone** improves outright — `correct_action_rate`
93.7%→100%, `false_brake_rate` 4.0%→0%, `missed_brake_rate` 50%→0%
(n_scored=79 keyframes both runs; the brake-event denominator is 4,
`indicative_only=True`, but the direction is unambiguous and matches the
predecessor project's own standalone-radar finding almost exactly).
**Fused** is a wash — `correct_action_rate` 86.1%→84.8% (within noise,
same n=79), `false_brake_rate` unchanged at 13.3%; `missed_brake_rate`
25%→50% is a single-event flip (1 vs 2 out of a denominator of 4,
Wilson 95% CI ~[0.15, 0.85] both ways) carrying no statistical weight.
Interpretation: Doppler is real, useful information for radar operating
alone, but in the fused stream LiDAR already dominates (97% correct per
step11's frozen-config result) and radar's own velocity measurement adds
nothing further once LiDAR is present — consistent with this project's
own fusion architecture (Mahalanobis gating already lets the
better-trusted sensor's updates dominate; G17's ablation doesn't change
what LiDAR contributes). Decision: **stays an ablation, not the default**
— `RADAR_DOPPLER_ABLATION` remains False in `config.py`; this finding
does not meet the bar to touch `configs/frozen_config.json` or the
official step11 result (fused was the frozen-config stream, and fused
showed no real improvement). Worth revisiting only if a future scenario
specifically needs radar-alone robustness (e.g., degraded-visibility
conditions where LiDAR/camera are unavailable). Full numbers:
`outputs/tables/doppler_ablation.json`.

**2026-09-24 — step11 — DEC-7: tuning selection criterion (pre-declared before any sweep)**
Choice: the mean of `correct_action_rate` across all four streams (lidar,
radar, camera, fused), with `missed_brake_rate` as a tie-breaker.
Reason: step11 §4.2's own recommended default, confirmed with Siva before
running any sweep specifically so the criterion cannot be chosen after
seeing results in a way that could favor fusion (the whole point of this
step is to test fusion's benefit, not assume it — G13, lessons A.11).
Every candidate parameter value's mean score across all four streams is
computed and logged in `outputs/tuning_log.csv`; the highest-mean value is
selected, with a missed-brake-rate comparison breaking any tie.

**2026-09-24 — step04c — 2D detector choice: Ultralytics YOLOv8n (CPU)**
Choice: `ultralytics` (YOLOv8n weights), CPU inference.
Reason: matches V1's own successful choice on this exact dataset (V1
`Step_2_3_YOLOv5.ipynb`-family notebooks used Ultralytics YOLOv8n, CPU-only,
conf=0.35/iou=0.45, COCO classes [0,1,2,3,5,7]). Confirmed with Siva before
installing: our own GPU (GeForce GT 710, 2GB VRAM) is too old/weak to
meaningfully accelerate inference, so CPU-only torch is used regardless of
detector choice, and a lightweight nano model matters for processing
~2,400 CAM_FRONT frames in reasonable time on CPU. torchvision Faster
R-CNN (the spec's other allowed option) was the alternative, not chosen.

**2026-09-24 — step04c — DEC-6: ground-plane back-projection depth method**
Choice: ground-plane back-projection (bottom-center pixel → camera ray via
intrinsics → rotate to ego via extrinsics → intersect z=GROUND_Z_EGO),
confirmed with Siva before coding.
Reason: this is the step's own proposed default (step04c_camera_adapter.md
§3.2), and has ZERO precedent in V1/V2 (checked directly) -- V1's only
genuinely camera-only depth method was similar-triangles apparent-size
(assumed per-class object height ÷ pixel height × focal length), a
different method with a different bias (systematic per-instance, vs.
ground-plane's systematic per-range/pitch bias). Built entirely fresh, no
V1 formulas/constants to port. Critically, V1's OTHER camera pipeline
("camera", not "camera_mono") secretly used median LiDAR-point depth
inside the 2D box -- exactly the leak this step's isolation test
(test_camera_isolation.py) exists to make structurally impossible to repeat.

**2026-09-23 — step04b — Ghost-return filtering: devkit defaults only, no extra filter**
Choice: use nuscenes-devkit's own default `RadarPointCloud.from_file()` filters
only (`invalid_states=[0]`, `dynprop_states=range(0,7)` — i.e. no dynprop
filtering at all, `ambig_states=[3]`, verified directly against the installed
devkit) — do NOT add V1's extra `dyn_prop ∈ {0,2,6}` stationary-clutter filter
on top.
Reason: step04b's own spec frames ghost/clutter returns (guard rails, signs,
manhole covers) as a test of the DOWNSTREAM pipeline's defenses (forward-path
filter, tracker, debounce) — "the pipeline's only defences; report how many
false brakes... trace to such returns" — and explicitly says "do not hack
around silently." Pre-filtering stationary clutter in the adapter itself
(V1's approach) would defeat this test and hide whatever residual ghost-brake
behavior actually exists. Confirmed with Siva before coding — diverges from
V1 on purpose. Each detection still records enough (`dyn_prop`-derived
stationary-point count in `aux`) for step10's false-brake attribution to
identify which false brakes trace to clutter, without filtering it upstream.

**2026-09-23 — step04a clustering library — use scikit-learn's DBSCAN**
Choice: `sklearn.cluster.DBSCAN`, not a hand-built scipy cKDTree fallback.
Reason: scikit-learn is already on disk (a transitive dependency of
`nuscenes-devkit` itself, not a new heavy install), and V1's own LiDAR
pipeline already used `sklearn.cluster.DBSCAN(eps=0.7, min_samples=10)`
successfully on this exact dataset — real, tuned precedent to start from
rather than building and validating a from-scratch alternative with no
direct precedent.

**2026-09-23 — DEC-10 — TTC point estimate vs. conservative bound for action**
Choice: point estimate drives the actual action decision (step08); the
conservative bound (`distance / (closing + k*closing_std)`) is computed
and reported as a diagnostic only, never acted on, for v1.
Reason: README_MASTER.md §6 recommendation. Keeps step08's action logic
simple and matches the point-estimate convention used throughout v1; the
diagnostic stays visible so uncertainty isn't hidden, just not yet load-
bearing.

**2026-09-20 — DEC-5 — Meaning of "2-4 updates of memory"**
Choice: a track is TTC-eligible after n_updates >= MIN_UPDATES_FOR_TTC (2)
and velocity std <= MAX_VEL_STD_MPS; evicted after EVICTION_GAP_S (0.3s) of
no update; sigma_a governs how fast old data is forgotten. No hard lifetime
cap on a track's age. `sensors_in_estimate` (step06 §3.6) is the sensors of
only the last MAX_MEMORY_UPDATES=4 updates (new config.py param, ASSUMED)
— a bounded recent-window set, not a lifetime-growing one.
Reason: v2's CentralTracker tracked `sensors_seen` as a lifetime-growing
set; explicitly not ported here per G11 ("which sensors contributed to the
CURRENT estimate," not a lifetime property) and what-not-to-do.md §1.

**2026-09-20 — DEC-8 — Tune/eval scene split**
Choice: tune = {scene-0103, scene-0061, scene-0655, scene-0796, scene-1094};
eval = {scene-1100, scene-1077, scene-0553, scene-0757, scene-0916}.
Written to `configs/split.json`, now FIXED per stepD0's own rule ("once
committed it is fixed").
Reason: all 5 AEB events in the dataset sit in a single scene (scene-1100) —
a scene-level split cannot give both sides real AEB representation, so it
was assigned to eval (the side the final Definition-of-Done table is built
from) rather than let a naive balanced-count split silently zero out eval's
AEB tier. Remaining 9 scenes round-robinned by GRADUAL+AEB danger count.
Tune side has 0 AEB examples (relies on 4 GRADUAL events) — a stated,
unavoidable tradeoff given nuScenes-mini's small danger-event count
(what-not-to-do.md §7).

**2026-09-20 — DEC-2 — Class policy across the four comparison runs**
Choice: symmetric — class is recorded on every row but never gates candidate
selection in ANY run (fused or single-sensor). GT scoped to `vehicle.*` +
`human.pedestrian.*`. False brakes diagnosed post-hoc by nearest GT category.
Reason: only camera naturally provides a class label; letting it gate while
LiDAR/radar stay class-blind would make the four-way comparison apples-to-
oranges (what-not-to-do.md §6). Camera-class gating deferred as a later,
explicitly scoped ablation.

**2026-09-20 — DEC-4 — Evaluation unit**
Choice: keyframe-level. Primary set = keyframes with ≥1 in-path GT object;
keyframes with an empty corridor reported separately as "phantom brakes,"
never mixed into the primary denominator.
Reason: pooling all keyframes (including the many with nothing in-path)
would dilute the danger-event rate with trivially-easy "nothing there"
cases and obscure the metric that actually matters.

**2026-09-20 — DEC-1 — Distance / reference-point definition**
Choice: **B — nearest-surface-to-ego**, confirmed at step05 Part A (needed
by `footprint_intersects_corridor`'s exact membership rule).
Reason: a partially-seen vehicle's visible centroid is biased ~1-2m from the
GT box centre; nearest-surface is what TTC physically needs and stays
consistent across sensors (README_MASTER.md §6). Used identically by
tracker output and ground truth (G9). `config.DISTANCE_DEFINITION` updated
from PLACEHOLDER to ASSUMED accordingly.

**2026-09-23 — step04a — SENSOR_R_LIDAR MEASURED promotion**
Choice: **stays ASSUMED** (Siva confirmed, matching the recommendation).
Finding behind the question: `scripts/measure_sensor_R.py` (tune split,
GT objects with `num_lidar_pts>=50`) found an 81.2% inlier rate (13/16,
residual<2.0m) — clears step04a §5's own 70% MEASURED threshold. Robust
(MAD) σ_long=0.082m, σ_lat=0.348m, vs. the current ASSUMED (0.3m, 0.3m).
Reason not promoted: N=16 barely clears this project's own
`MIN_COUNT_FOR_RATE=10`, and matching is nearest-neighbour-only (no real
track identity yet at this stage), so an occasional residual could be
against the wrong nearby object. `config.py` left unchanged (step04a §5:
never silently swap in a measured value) — see `reports/step04a_report.md`
§4. Revisit after more data (step11) or once real tracked-object identity
exists to do a cleaner match.

**2026-09-23 — step04a — process correction: evidence script used an eval-split scene**
Finding: `scripts/lidar_adapter_evidence.py` (and ad hoc diagnostic scripts)
had been run against `scene-1077`, which `configs/split.json` (DEC-8)
assigns to EVAL, not tune — a direct violation of step04a §4 Do NOT #6
("do not evaluate on the eval split"). Caught while investigating the
self-return bug (see `reports/step04a_report.md` §1-2), before the step
was reported. Fix: switched to `scene-1094` (tune split), deleted and
regenerated the affected figures, re-verified the self-return finding
independently against the corrected scene (reproduced identically — it is
a sensor-mounting geometry fact, not scene-content-dependent, so the
underlying fix is unaffected by the correction, but the mistake itself is
recorded here plainly). `run_pipeline.py`'s real-stream mode additionally
reuses step10's own `guard_eval_scoring` rather than a separate check, so
this class of mistake is structurally harder to repeat going forward.

**2026-10-08 — step12 — DEC-11 through DEC-14: BEV visualization, approved**
Choice: all four with Claude's own recommendation (Siva: "ok with ur
recommendation").
- **DEC-11 (orientation):** (A) ego-centered, heading-up — ego's own
  heading always points "up" on screen, matching the predecessor
  project's V1 BEV convention (`Step_8_BEV_Fusion_Video.ipynb`) and
  reading naturally for a braking/TTC context.
- **DEC-12 (viewport):** (A) fixed window every frame — ego-frame
  extent x (forward) in [-5, CORRIDOR_MAX_RANGE_M+5] = [-5, 65]m,
  y (lateral) in [-15, 15]m, matching this project's own existing
  adapter-evidence script convention (`scripts/*_adapter_evidence.py`),
  not auto-fit — a fixed scale is what makes TTC-urgency color
  meaningful frame to frame.
- **DEC-13 (TTC color-bin source):** (B) derive bins from
  `config.TTC_AEB_S`/`TTC_GRADUAL_S` (G12 — single source of truth),
  never a separate hardcoded set of numbers like V1 used.
- **DEC-14 (scene scope):** (B) any scene via explicit `--scene`,
  defaulting to tune split, with eval-split frames visibly labelled
  "EVAL" in the rendered output — no scoring/tuning happens from
  watching a video (no G13 risk), and a portfolio reader will want to
  see the actual frozen-config eval result.
Reason: full step protocol requested for this step (step12,
`step12_bev_visualization.md`) — new scope beyond README_MASTER.md §7's
original 17-step build order, prompted by Siva wanting a BEV
visualization. Static per-keyframe PNGs built first (per the step
file's own §8 STOP point); per-scene video is a separate, later
approval gate.
