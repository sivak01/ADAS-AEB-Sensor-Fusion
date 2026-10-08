# instant-ttc-fusion — Master Build Instructions (for Claude Code)

Read this file completely before touching any step file. Every step file assumes this one.

## 0. Roles

- **Claude Code (you):** builder. You write code, tests, and reports.
- **Siva:** reviewer, learner, and knowledge sharer. Siva must be able to understand, defend, and explain every line to someone else (portfolio / interviews). Optimise for *clarity and teachability*, not cleverness.

Consequences:
- Small, readable modules. Plain names. Docstrings that say *why*, not just *what*.
- Every step ends with a **stop-and-report**. You never start the next step without Siva writing "approved".
- Explain concepts in plain language with a tiny numeric example in the learning note.

## 1. Read first (in this order)

1. `F:\TTC Fusion\CLAUDE.md` — project mission, scope, design contract (this is the `CLAUDE_Instant_TTC_Fusion.md` file).
2. `F:\TTC Fusion\knowledge\what-not-to-do.md` — mistakes to avoid. Re-check it before every design decision.
3. `F:\TTC Fusion\knowledge\lessons-from-v1-v2.md`
4. `F:\TTC Fusion\knowledge\nuscenes-reference.md`

If any of these is missing, STOP and tell Siva. Do not proceed from memory.

## 2. Environment and paths

- OS: Windows 11. The project path contains a space: `F:\TTC Fusion`. Always quote paths in shell commands. In Python use `pathlib.Path`, never string concatenation with backslashes.
- Dataset: `F:\TTC Fusion\archive` — **READ-ONLY**. Never write, rename, or delete anything inside it. Never commit it (nuScenes licence).
- Never hard-code a path anywhere except `config.py` (`DATAROOT`, overridable by env var `TTCF_DATAROOT`).
- Python: check what is installed first (`python --version`, existing `.venv`). Use a venv at `F:\TTC Fusion\.venv`. Core deps: `nuscenes-devkit`, `pyquaternion`, `numpy`, `scipy`, `matplotlib`, `pandas`, `pytest`. **Ask before installing anything heavy** (torch, ultralytics, scikit-learn, etc.).
- Siva runs Jupyter from the F drive. Review artifacts should be PNG figures + printed summaries (notebooks optional).
- Git: propose a commit message after each approved step. Pushing to the remote requires Siva's explicit go-ahead each time (default: don't push) — updated 2026-10-09 from the original "never push" after Siva explicitly asked for the first push (origin set to `https://github.com/sivak01/Instant-TTC.git`); see `docs/decisions.md`. Provide a `.gitignore` covering `archive/`, `outputs/cache/`, `.venv/`.

## 3. Repo layout (create as steps require; do not pre-create empty scaffolding)

```
F:\TTC Fusion\
├── archive\                     # nuScenes data (read-only)
├── CLAUDE.md
├── knowledge\                   # the 3 knowledge files
├── instructions\                # these files
├── src\ttcf\
│   ├── config.py   types.py
│   ├── data\        event_stream.py
│   ├── geometry\    transforms.py   corridor.py
│   ├── filtering\   kalman.py   ego_state.py
│   ├── adapters\    lidar.py  radar.py  camera.py
│   ├── tracking\    tracker.py
│   ├── ttc\         ttc_math.py  estimator.py  action.py
│   ├── gt\          gt_events.py
│   └── evaluation\  extrapolate.py  metrics.py  report.py
├── scripts\         preflight_dataset.py  run_pipeline.py ...
├── tests\
├── configs\         split.json  frozen_config.json
├── outputs\         gt_events.csv, logs, figs, cache, tables
├── reports\         stepXX_report.md
└── docs\            decisions.md  learning\stepXX_learning.md  knowledge_share.md
```

## 4. Working protocol — applies to EVERY step

**Before coding**
1. Read the step file, and re-read the relevant rules in §5.
2. Post a plan of at most 10 lines: files you will create, interfaces, assumptions.
3. List every **Decision (DEC-n)** the step touches. Present the options and your recommendation. **Wait for Siva's answer**, then record it in `docs/decisions.md` (date, DEC id, choice, reason).

**While coding**
4. Build only what the step lists. If something tempting is out of scope, write it under "Flagged, not built" in the report (see CLAUDE.md "Working style").
5. Write the tests listed in the step file *first or alongside*. Synthetic tests must pass before any real-data run.
6. Nothing is "measured" unless it was actually measured on data. Everything else is `ASSUMED` or `PLACEHOLDER` in config (G12).

**After coding — stop and report**
7. Run all tests. Show the output.
8. Write `reports\stepXX_report.md` using the template in §8.
9. Write `docs\learning\stepXX_learning.md` (template in §8).
10. Post a short chat summary, list any surprising number as a *suspected bug first* (G13), and **STOP**. Wait for "approved".

## 5. Global rules (referenced as G1–G17 in step files)

- **G1 Global frame.** Every position is transformed to the global frame before any comparison across time or across sensors.
- **G2 Per-channel identity of time and pose.** Each `sample_data` record has its own timestamp, `ego_pose_token`, and `calibrated_sensor_token`. APIs must take the record/event itself so borrowing another channel's values is structurally impossible.
- **G3 Native rate.** Use `sweeps/` (via `sample_data.next` chains), not only 2 Hz `samples/`.
- **G4 Radar clustering.** Cluster radar points per channel, per scan, before anything downstream.
- **G5 No finite-difference velocity.** Object velocity and ego velocity come from the shared constant-velocity Kalman filter, never `(p2-p1)/dt`. (Exception: ground-truth velocity may use the devkit's `box_velocity`; it is ground truth, not the pipeline.)
- **G6 One KF class.** Sensor-agnostic. `R` passed per `update()`; `sigma_a` fixed at construction; the filter knows nothing about sensor identity.
- **G7 Gate = Mahalanobis + chi-square with `S = H·P_pred·Hᵀ + R`** (incoming detection's own R included). No fixed Euclidean gate.
- **G8 TTC uses velocity relative to ego,** with two separate deadbands: `MIN_TRUSTED_SPEED` (object speed is real vs jitter) and `MIN_CLOSING_SPEED` (`closing <= MIN_CLOSING_SPEED` returns `inf`, not `<= 0`).
- **G9 One distance definition** (DEC-1), used identically by tracker output and ground truth.
- **G10 Early fusion.** One shared track set that all sensors write into. No per-sensor trackers merged later.
- **G11 Short memory.** No multi-second eviction timer, no scene-boundary reset subsystem, no lifetime sensor bookkeeping (only "which sensors contributed to the CURRENT estimate"), no MOT metrics.
- **G12 Config is the single source of truth.** Every constant lives in `config.py` with a status: `MEASURED`, `ASSUMED`, or `PLACEHOLDER` plus a one-line reason. Never duplicate a value.
- **G13 Honest evaluation.** Tune only on the tuning split. All four runs use identical code. Report raw counts next to rates. A surprising number is a bug until proven otherwise. If fusion does not beat the best single sensor, say so plainly.
- **G14 Camera baseline is genuinely single-sensor.** No LiDAR/radar/GT-derived value may enter the camera path.
- **G15 Causality.** Nothing in the pipeline may use data with a timestamp later than the current event. (Only ground-truth construction may look ahead.)
- **G16 Debounce.** Never act on a single frame's TTC.
- **G17 Scope.** Ask before expanding. Radar Doppler is **not** fed into the KF in v1 (later, scoped ablation only).

## 6. Decision register (ask Siva; recommendations included)

| ID | Decision | Recommendation |
|---|---|---|
| DEC-1 | Distance definition: (A) centroid-to-ego, or (B) nearest-surface-to-ego | **B.** A partially seen vehicle's visible centroid is biased ~1–2 m from the GT box centre; nearest-surface is what TTC physically needs and is consistent across sensors. |
| DEC-2 | Class policy across the 4 runs | Class is **recorded but not used for gating in any run** (symmetric). GT events scoped to `vehicle.*` + `human.pedestrian.*`. Diagnose false brakes by nearest GT category. Camera-class gating = later ablation. |
| DEC-3 | Active channels | `LIDAR_TOP`, `RADAR_FRONT`, `RADAR_FRONT_LEFT`, `RADAR_FRONT_RIGHT`, `CAM_FRONT` |
| DEC-4 | Evaluation unit | Keyframe-level: primary set = keyframes with ≥1 in-path GT object; empty-corridor keyframes reported separately as "phantom brakes". |
| DEC-5 | Meaning of "2–4 updates of memory" | A track is TTC-eligible after ≥2 updates *and* velocity std below a limit; it is evicted after `EVICTION_GAP`; `sigma_a` governs how fast old data is forgotten. No hard lifetime cap. |
| DEC-6 | Camera depth method | Ground-plane back-projection of the box's bottom-centre using calibration extrinsics (no depth network) for v1. |
| DEC-7 | How to pick the single shared `sigma_a` | Pre-declare: best **mean** score across all four streams on the tuning split (not fused-only, which would favour fusion). |
| DEC-8 | Tune/eval scene split | Proposed only after seeing per-scene danger-event counts (step D0). |
| DEC-9 | Initial ASSUMED thresholds (TTC tiers, corridor, debounce N) | Confirm the proposed starting values in step 00. |
| DEC-10 | Action uses TTC point estimate or conservative bound | Point estimate for v1; report conservative bound as a diagnostic. |

## 7. Build order (this is NOT the numeric order of the diagram)

The GT table needs ego velocity, TTC math, and corridor geometry, so those come first.

| # | File | What gets built | Gate |
|---|---|---|---|
| 1 | `step00_config_contracts.md` | dataset preflight, config, shared types | approved |
| 2 | `step02_transform.md` | sensor→ego→global utility | approved |
| 3 | `step03_kf_ego_state.md` | shared KF + ego-state estimator | approved |
| 4 | `step07_ttc.md` **Part A** | pure TTC math + synthetic tests | approved |
| 5 | `step05_forward_path_filter.md` **Part A** | corridor geometry + candidate gate | approved |
| 6 | `stepD0_gt_event_table.md` | GT event table, split proposal | approved |
| 7 | `step01_event_stream.md` | native-rate multi-channel event stream | approved |
| 8 | `step06_tracker.md` | shared short-memory tracker (synthetic only) | approved |
| 9 | `step05_forward_path_filter.md` **Part B** | track-level path relevance | approved |
| 10 | `step07_ttc.md` **Part B** | track-level TTC estimator | approved |
| 11 | `step08_debounce_action.md` | debounce + action tiers (synthetic) | approved |
| 12 | `step09_extrapolation.md` | bounded, causal extrapolation to GT time | approved |
| 13 | `step10_metrics.md` | event-level metrics + report generator | approved |
| 14 | `step04a_lidar_adapter.md` | LiDAR adapter + first end-to-end LiDAR run (tune split) | approved |
| 15 | `step04b_radar_adapter.md` | radar adapter, Doppler OFF | approved |
| 16 | `step04c_camera_adapter.md` | camera adapter (single-sensor) | approved |
| 17 | `step11_runs_tuning_report.md` | four runs, tuning, freeze, ONE final evaluation | approved |

## 8. Templates

**`reports\stepXX_report.md`**
```
# Step XX report
## What was built (files, one line each)
## Decisions taken (DEC ids + Siva's answers)
## Tests (command + pass/fail output)
## Real-data verification (what you ran, what it showed, figures saved where)
## Surprising or suspicious numbers (treat as bugs first)
## Assumptions introduced (each must be in config with a status)
## Flagged, not built (out-of-scope temptations)
## Open questions for Siva
```

**`docs\learning\stepXX_learning.md`** (Siva will reuse this to teach others)
```
# Step XX — learning note
## The idea in plain language (≤ 8 sentences)
## Tiny worked numeric example
## Why it is designed this way (which past mistake it prevents — cite what-not-to-do.md section)
## How to explain this in an interview (3 bullets)
## Common misunderstandings
```

## 9. Definition of done for the whole project

- Four-row table (lidar / radar / camera / fused) with raw counts, from ONE final evaluation run on held-out scenes, with all mandatory footnotes.
- Frozen config + hash; tuning log; decisions log.
- `docs\knowledge_share.md` written for a portfolio reader.
- All synthetic tests green; all "Do NOT" items in each step file verified.
