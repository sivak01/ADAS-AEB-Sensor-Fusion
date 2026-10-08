# Final report — instant-ttc-fusion

**Held-out eval split, `--final`, frozen config hash `58709abded7796ead9a7b086f82484e02ef7527aae165a6e1fae7e6fe96926c5`.**
Eval split scored once per stream (with one documented exception: camera/fused
were re-attempted after a missing-cache infrastructure crash produced zero
results the first time — see §5 below); the official table below is
re-derived deterministically from that same accepted computation (verified
identical, `tests/test_run_equivalence.py`).

## 1. The four-row table (never fused alone)

| Run | Variant | n | correct_action_rate | false_brake_rate | missed_brake_rate |
|---|---|---|---|---|---|
| lidar | all | 133 | 97.0% (129/133) [0.93, 0.99] | 0.8% (1/126) [0.00, 0.04] | 28.6% (2/7) [0.08, 0.64] (indicative) |
| lidar | obs_lidar | 133 | 97.0% (129/133) [0.93, 0.99] | 0.8% (1/126) [0.00, 0.04] | 28.6% (2/7) [0.08, 0.64] (indicative) |
| radar | all | 133 | 94.0% (125/133) [0.89, 0.97] | 3.2% (4/126) [0.01, 0.08] | 42.9% (3/7) [0.16, 0.75] (indicative) |
| radar | obs_radar | 133 | 93.2% (124/133) [0.88, 0.96] | 4.7% (6/129) [0.02, 0.10] | 50.0% (2/4) [0.15, 0.85] (indicative) |
| camera | all | 133 | 95.5% (127/133) [0.91, 0.98] | 0.8% (1/126) [0.00, 0.04] | 71.4% (5/7) [0.36, 0.92] (indicative) |
| camera | obs_camera | 133 | 95.5% (127/133) [0.91, 0.98] | 0.8% (1/126) [0.00, 0.04] | 71.4% (5/7) [0.36, 0.92] (indicative) |
| **fused** | all | 133 | **87.2% (116/133)** [0.80, 0.92] | **11.1% (14/126)** [0.07, 0.18] | 28.6% (2/7) [0.08, 0.64] (indicative) |
| **fused** | obs_any | 133 | **87.2% (116/133)** [0.80, 0.92] | **11.1% (14/126)** [0.07, 0.18] | 28.6% (2/7) [0.08, 0.64] (indicative) |

Full raw data: `outputs/tables/four_run_table.{md,csv,json}`. Tuning-phase
sweep: `outputs/tuning_log.csv`.

## 2. Did fusion beat the best single sensor? — **No, not on this dataset, with this design.**

Stated plainly, with counts, per step11 §6's own requirement: **fusion did
not beat the best single sensor.** LiDAR-only has the highest
`correct_action_rate` (97.0%, 129/133) and the lowest `false_brake_rate`
(0.8%, 1/126) of all four streams. Fused's `correct_action_rate` (87.2%,
116/133) is the *lowest* of the four, and its `false_brake_rate` (11.1%,
14/126) is roughly **3.5-14x** every single sensor's own rate.

Fusion is not a uniform loss, though — this is a real trade-off, not a
one-sided defeat:

- **Missed-brake rate**: fused ties LiDAR for the *best* missed-brake rate
  (28.6%, 2/7) — substantially better than radar (42.9%) or camera (71.4%)
  alone. Fusing sensors with different, partially-overlapping blind spots
  does measurably help catch real danger events that a single sensor alone
  would miss — this is fusion's genuine benefit, visible in the data.
- **The cost is false alarms.** Combining detections from three sensors
  into one tracker creates more opportunities for a spurious track to form
  (from any one sensor's own noise/ghost returns, or from cross-sensor
  misassociation where two different real nearby objects get merged into
  one track's identity), and this project's short-memory, no-persistent-
  identity tracker design (a deliberate simplicity trade-off, see
  `README_MASTER.md`) has no long-horizon mechanism to reject these.

This pattern is **consistent between the tune and eval splits** (fused
showed the same "best missed-brake, worst false-brake" shape on tune-split
data too, before the eval split was ever touched) — this is a real,
repeatable property of the current design, not a one-off statistical
fluke on a small eval set.

## 3. Investigation of the surprising result (G13, step11 §5.4)

Fused's elevated false-brake rate (14 instances) was investigated before
being accepted, per this step's own explicit instruction ("camera beating
fused" is the named example of a result requiring investigation — this
result comes close to that exact shape). Traced every false-brake driving
track back to its own sensor mix and originating scene:

- **Half of all 14 false brakes (7) come from a single scene**:
  `scene-0916`, described in the dataset itself as *"Parking lot, bicycle
  rack, parked bicycles, bus, many peds, parked scooters, parked
  motorcycle."* This is a textbook instance of this project's own
  **already-documented, already-named-as-open limitation**: crowd
  conflation among closely-spaced same-class objects (found originally at
  step06, listed as a required named limitation below). Multiple
  *different* tracks (6 distinct track IDs) fired false brakes across this
  one scene at different times — consistent with track-identity churn in a
  crowded scene, not one persistent bad track.
- **The remaining false brakes span a genuine mix of sensor combinations**
  — 9 of 14 driving tracks involved 2 or 3 sensors, not just 1. This
  confirms the false-brake increase is not merely "one sensor's own known
  issue carrying over unchanged into fused" — genuine cross-sensor
  association effects are a real contributor.

**Conclusion: not a new bug.** The dominant contributor is the exact,
already-documented crowd-conflation limitation this project's design
explicitly declined to solve (a deliberate scope decision, not an
oversight — see `what-not-to-do.md` §4 / `lessons-from-v1-v2.md` item
C.3). The eval split was not re-run to "fix" this, per step11 §5.4's own
rule — it is reported as found.

## 4. Named limitations (step11 §6's required list)

- **Crowd conflation** (same-class objects close together, e.g.
  pedestrians/bicycles at a crosswalk or a busy parking lot) — safety-
  relevant and explicitly open (`lessons-from-v1-v2.md` item C.3, first
  found at step06, now the dominant cause of §3's own false-brake finding).
  The short-memory, no-persistent-identity tracker has no mechanism to
  resolve which of several nearby same-class objects a given detection
  truly belongs to.
- **Curved-path corridor** — the forward-path corridor is straight in the
  ego frame; no lane/path prediction exists, so a genuinely curving road
  (a turn, an intersection) is a known blind spot for path-relevance
  classification (step05 Part A's own stated scope).
- **Flat-ground camera depth** (DEC-6) — ground-plane back-projection
  assumes a flat road; range error grows with range² and fails on slopes/
  pitching (empirically confirmed at step04c: median error 1.78m at
  0-15m, growing to 17.29m at 50-200m). One concrete numerical-instability
  example found (a box "detected" 1,696m away, near the image horizon).
- **Radar Doppler disabled** (G17) — this project's radar-only baseline
  never uses the sensor's own instantaneous-velocity measurement; velocity
  comes from position history only, a deliberately conservative choice
  (see the Doppler-ablation item below).
- **Ghost radar returns** — stationary clutter (guard rails, signs,
  construction barriers) is not filtered upstream by design (step04b
  decision); the downstream pipeline (corridor filter, tracker, debounce)
  is the only defense, and it is not perfect — step04b's own real-data run
  found and fully traced one false brake to a real construction barrier.
- **Tiny number of danger events in nuScenes-mini** — the eval split has
  only 7 real danger events (2 GRADUAL, 5 AEB) behind every
  `missed_brake_rate` figure in this report; every such rate is correctly
  labelled "indicative only" throughout, per `MIN_COUNT_FOR_RATE=10`.
- **Ego-velocity KF lag** — ego velocity is derived causally from a
  Kalman filter over `ego_pose` history (G5/G6), never a raw finite
  difference; this is more robust but inherently trails the true
  instantaneous ego motion by a small amount, same as every other causal
  filter in this pipeline (never a hidden non-causal shortcut, but a real,
  structural source of small lag).

## 5. Process note: the eval-split camera/fused crash (full detail in `docs/decisions.md`)

The first `--final` eval-split attempt succeeded for lidar and radar, but
camera and fused crashed with `FileNotFoundError` — `scripts/
cache_camera_detections.py` had only ever been run with `--split tune`.
Confirmed via `outputs/eval_runs.log` that no actual scoring occurred for
camera/fused before the crash (timestamps ~2 seconds apart, the crash was
inside the adapter, upstream of any GT comparison) — this was **not**
treated as "re-running the eval split to fix a bug" (step11 §5.4), since
there was no completed score to redo, only a missing prerequisite (the
eval-split cache) to build before the real run could happen at all. Fixed
by caching the eval split's CAM_FRONT frames, then re-attempting only the
two streams that had crashed. Recorded here and in `docs/decisions.md`
plainly, per the same rule's own spirit.

## 6. Config parameter status table

Full table (51 params): see `docs/decisions.md` and `config.describe()`'s
own output. Summary: **0 MEASURED, 50 ASSUMED, 1 PLACEHOLDER**
(`SENSOR_R_CAMERA` — intentionally a range-dependent function, not a
static value, per its own reason field). Every `SENSOR_R_*` param cleared
the empirical "trustworthy" threshold at least once during this project
(LiDAR 81.2%, radar 92.0%, both N>70% inlier) but was deliberately kept
ASSUMED rather than promoted — see `docs/decisions.md`'s own entries for
the full reasoning (thin samples, coarse nearest-neighbour matching, and
for radar specifically, this would have been the project's first-ever
MEASURED value with system-wide gating effects). `SIGMA_A`,
`MAX_VEL_STD_MPS`, `MIN_TRUSTED_SPEED_MPS`, and `MIN_CLOSING_SPEED_MPS`
were the four PLACEHOLDER values resolved at this step, all via direct
measurement or a pre-declared sweep, documented in full (including three
real bugs found and fixed in the measurement methodology itself) in
`docs/decisions.md`.

## 7. "Flagged, not built" (step11 §6's required list)

- **Doppler-in-KF ablation** — using radar's raw instantaneous-velocity
  measurement directly would need Siva's explicit separate approval
  (G17); not attempted in this project's v1 scope.
- **Camera-class gating ablation** — using YOLO's own class output to
  gate/weight detections (rather than only recording it, DEC-2) was never
  built; the class-symmetric policy was maintained throughout for a fair
  four-way comparison.
- **Appearance re-identification** — no visual/feature-based re-ID exists
  anywhere in this project (deliberately, per `what-not-to-do.md` §1); the
  tracker relies entirely on position/gating, consistent with its short-
  memory, MOT-avoiding design.
- **Learned (network-based) depth** — camera-only depth uses geometric
  ground-plane back-projection (DEC-6) exclusively; no monocular depth
  network was trained or used.
- **Steering scoring** — this project scores braking action tiers only
  (NONE/GRADUAL/AEB); no lateral/steering avoidance scoring exists.
- **Curved-path prediction** — see the named limitation above; the
  corridor stays straight in the ego frame throughout.

## 8. Reproduction instructions

```
# 1. Environment (CPU-only torch; see requirements.txt's own header comment)
pip install torch==2.14.0+cpu --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt

# 2. Dataset: set TTCF_DATAROOT to the nuScenes-mini root, or place it under ./archive
export TTCF_DATAROOT=/path/to/nuscenes-mini

# 3. Cache camera detections (both splits; needed before any camera/fused run)
python scripts/cache_camera_detections.py --split tune
python scripts/cache_camera_detections.py --split eval

# 4. Reproduce the tuning sweep (writes outputs/tuning_log.csv)
python scripts/tune.py

# 5. Freeze the config (writes configs/frozen_config.json + SHA-256)
python -c "import sys; sys.path.insert(0,'src'); from ttcf import config; print(config.freeze('configs/frozen_config.json'))"

# 6. Final eval-split run (each stream; requires --final and a hash match)
python scripts/run_pipeline.py --stream lidar  --split eval --final
python scripts/run_pipeline.py --stream radar  --split eval --final
python scripts/run_pipeline.py --stream camera --split eval --final
python scripts/run_pipeline.py --stream fused  --split eval --final

# 7. Generate the official four-run table
python scripts/generate_final_table.py
```

Frozen config hash used for every result in this report:
`58709abded7796ead9a7b086f82484e02ef7527aae165a6e1fae7e6fe96926c5`
(`configs/frozen_config.json`). Dataset: nuScenes-mini, devkit 1.2.0, 10
scenes, tune/eval split per `configs/split.json` (DEC-8).
