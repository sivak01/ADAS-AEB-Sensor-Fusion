# ADAS-AEB-Sensor-Fusion

Real-time time-to-collision (TTC) estimation for an AEB (automatic
emergency braking) decision layer, fusing LiDAR, radar, and camera on
[nuScenes-mini](https://www.nuscenes.org/). One event-driven pipeline —
native sensor rate, early fusion into a single shared tracker, causal
throughout — evaluated honestly against single-sensor baselines rather
than assumed to win.

This is a scoped-down rebuild of an earlier, broader multi-object-tracking
project, narrowed specifically to the TTC/AEB decision problem.

## Headline result

Four identical pipelines (lidar-only / radar-only / camera-only / fused),
tuned on held-out scenes, scored **once** on a separate held-out eval
split:

| Run | n | correct_action_rate | false_brake_rate | missed_brake_rate |
|---|---|---|---|---|
| lidar | 133 | **97.0%** (129/133) | **0.8%** (1/126) | 28.6% (2/7, indicative) |
| radar | 133 | 94.0% (125/133) | 3.2% (4/126) | 42.9% (3/7, indicative) |
| camera | 133 | 95.5% (127/133) | 0.8% (1/126) | 71.4% (5/7, indicative) |
| **fused** | 133 | 87.2% (116/133) | 11.1% (14/126) | **28.6%** (2/7, indicative) |

**Fusion did not beat the best single sensor.** LiDAR-only wins on
accuracy and false-brake rate. Fusion's genuine benefit shows up
elsewhere: it ties LiDAR for the *best* missed-brake rate, catching real
danger events that single sensors alone missed — at the cost of a
noticeably higher false-brake rate. Investigated, not papered over: half
of fusion's false brakes trace to one crowded scene (a parking lot with
many pedestrians/bicycles), an already-named, open limitation of this
project's short-memory, no-persistent-identity tracker design, not a new
bug. Full writeup, investigation, and every named limitation:
[`reports/final_report.md`](reports/final_report.md).

## Architecture, in one paragraph

Every sensor event (not just 2Hz keyframes — native per-sensor rate) is
merged into one chronological stream and fed to **one shared
constant-velocity Kalman-filter tracker** — never a per-sensor tracker
merged after the fact. Detections gate onto tracks via Mahalanobis
distance with `S = P_pred + R` (the incoming detection's own noise
included), always position-only regardless of which measurement channels
a given sensor happens to carry. TTC uses velocity relative to ego (never
a finite difference — ego and object velocity both come from the same KF
family), with explicit deadbands separating "no real motion" from "no
real closing speed." A short-memory design deliberately avoids MOT-style
long-horizon identity tracking, scene-boundary reset logic, and
appearance re-identification — see
[`what-not-to-do.md`](what-not-to-do.md) for what was deliberately left
out and why.

Every constant lives in one `config.py`, tagged `MEASURED` / `ASSUMED` /
`PLACEHOLDER` with a one-line reason — nothing is tuned or assumed
silently. Every decision point (distance definition, class policy, which
sensors are active, how the tune/eval split works, and more) is logged
with its reasoning in [`docs/decisions.md`](docs/decisions.md).

## Repo layout

```
src/ttcf/
  config.py, types.py          shared config/types, every constant typed and justified
  data/event_stream.py         native-rate, multi-channel event merging
  geometry/transforms.py       sensor -> ego -> global frame transforms (G1/G2)
  geometry/corridor.py         forward-path corridor + candidate gate
  filtering/kalman.py          one sensor-agnostic constant-velocity KF
  filtering/ego_state.py       causal ego velocity (same KF family, never finite-difference)
  adapters/{lidar,radar,camera}.py   per-sensor detection adapters
  tracking/tracker.py          the shared short-memory tracker (early fusion)
  ttc/{ttc_math,estimator,action}.py   TTC math, track-level estimator, debounce/action tiers
  gt/gt_events.py              ground-truth event table (the "answer key")
  evaluation/{extrapolate,metrics,report}.py   causal GT-time extrapolation, event-level scoring
  viz/bev.py                   bird's-eye-view drawing primitives

scripts/        run_pipeline.py, tune.py, render_bev.py, and one-off evidence/measurement scripts
tests/          one test file per module, synthetic-first
configs/        split.json (tune/eval scenes), frozen_config.json (+ SHA-256)
reports/        per-step reports + the final four-run report
docs/           decisions log, learning notes (one per build step), portfolio knowledge-share
step*.md        the step-by-step build specification this project was built against
```

## Reproducing the final result

```bash
# 1. Environment (CPU-only torch — see requirements.txt's own header)
pip install torch==2.14.0+cpu --index-url https://download.pytorch.org/whl/cpu
pip install -r requirements.txt

# 2. Dataset: nuScenes-mini, set TTCF_DATAROOT or place it under ./archive
export TTCF_DATAROOT=/path/to/nuscenes-mini

# 3. Cache camera detections (both splits)
python scripts/cache_camera_detections.py --split tune
python scripts/cache_camera_detections.py --split eval

# 4. Tuning sweep -> outputs/tuning_log.csv
python scripts/tune.py

# 5. Final eval-split run (one stream at a time, requires --final + a config-hash match)
python scripts/run_pipeline.py --stream lidar  --split eval --final
python scripts/run_pipeline.py --stream radar  --split eval --final
python scripts/run_pipeline.py --stream camera --split eval --final
python scripts/run_pipeline.py --stream fused  --split eval --final

# 6. Official four-run table
python scripts/generate_final_table.py

# Tests (synthetic, no dataset needed)
pytest tests/ -q --ignore=tests/test_run_equivalence.py
```

Frozen config hash for every number in `reports/final_report.md`:
`58709abded7796ead9a7b086f82484e02ef7527aae165a6e1fae7e6fe96926c5`
(`configs/frozen_config.json`).

## Beyond the frozen result

Two post-freeze investigations, tune-split only, never touching the
frozen eval result (`docs/decisions.md`, 2026-10-08):
- **Radar Doppler velocity ablation** — feeding radar's own Doppler
  measurement into the tracker as a real second measurement channel
  (off by default, G17). Helps radar operating alone substantially;
  makes no real difference once fused with LiDAR, which already
  dominates the fused stream.
- **NIS filter sanity check** — every stream's real matched-update
  innovation statistics are mildly underconfident vs. chi-squared
  theory, never overconfident; a benign finding, no config change.

Plus a bird's-eye-view visualization tool (`scripts/render_bev.py`):
static per-keyframe PNGs and native-rate scene videos, ego-centered,
heading-up, tracks colored by TTC urgency, GT boxes and the forward
corridor overlaid for a visual sanity check against the real pipeline's
own outputs.

## Dataset

Built against [nuScenes-mini](https://www.nuscenes.org/nuscenes) (devkit
1.2.0, 10 scenes). The dataset itself is not included in this repo (see
`.gitignore`) — nuScenes' own license does not permit redistribution.

## Provenance

Built step by step against a pre-written build protocol
(`README_MASTER.md`), with a plan and explicit decisions posted and
approved before each step, and a report + learning note written after
every one. See `docs/project_status.md` for the full build history and
`docs/knowledge_share.md` for a portfolio-level writeup of the whole
project's design and findings.
