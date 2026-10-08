# Step 00 report

## What was built (files, one line each)
- `src/ttcf/config.py` — single source of truth: `Param(value, status, reason, tunable_on)`, `DATAROOT`/`NUSCENES_VERSION` resolution (env override, else depth-3 auto-discovery), `describe()`, `freeze()`.
- `src/ttcf/types.py` — frozen dataclasses `RawEvent`, `Detection`, `TrackSnapshot`, `TTCResult` (+ `TTCReason`), `ActionDecision` (+ `ActionTier`); `t_us` is int-checked at construction.
- `scripts/preflight_dataset.py` — dataset discovery, table-size report, sweeps check (existence + file spot-check), devkit version/signature introspection, category/attribute listing, native-rate-per-channel report, environment report.
- `tests/test_config.py` — 12 tests covering step00 §7.
- `pytest.ini` — `pythonpath = src`, so `from ttcf import ...` resolves under the `src/ttcf` layout without an editable install.
- `docs/decisions.md` — DEC-3, DEC-9 recorded.
- `requirements.txt` — pinned versions actually installed in `.venv` (see Environment section below).

## Decisions taken (DEC ids + Siva's answers)
- **DEC-3** — Active channels: `LIDAR_TOP, RADAR_FRONT, RADAR_FRONT_LEFT, RADAR_FRONT_RIGHT, CAM_FRONT`. Approved as proposed.
- **DEC-9** — Initial ASSUMED/PLACEHOLDER threshold table (step00_config_contracts.md §4). Approved as proposed, no changes.
- Both recorded in `docs/decisions.md` with date and reason.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\test_config.py -v`
```
collected 12 items

tests/test_config.py::test_every_param_has_status_and_nonempty_reason PASSED
tests/test_config.py::test_no_measured_status_exists_yet PASSED
tests/test_config.py::test_describe_renders_every_parameter PASSED
tests/test_config.py::test_freeze_hash_stable_when_nothing_changes PASSED
tests/test_config.py::test_freeze_hash_changes_when_a_value_changes PASSED
tests/test_config.py::test_freeze_writes_valid_json_with_hash PASSED
tests/test_config.py::test_dataroot_and_active_channels_are_sane PASSED
tests/test_config.py::test_raw_event_t_us_must_be_int PASSED
tests/test_config.py::test_detection_t_us_must_be_int PASSED
tests/test_config.py::test_track_snapshot_t_us_must_be_int PASSED
tests/test_config.py::test_ttc_result_t_us_must_be_int PASSED
tests/test_config.py::test_action_decision_t_us_must_be_int PASSED

12 passed in 0.63s
```

## Real-data verification (what you ran, what it showed, figures saved where)
Ran `scripts/preflight_dataset.py` against `F:\TTC Fusion\archive` (v1.0-mini). Full output saved to `outputs/logs/step00_preflight_log.txt`. Summary:

- **DATAROOT discovery**: `archive/` itself has the nuScenes layout directly (no nested Kaggle folder here) — discovered at depth 0, resolved to `F:\TTC Fusion\archive`, version `v1.0-mini`.
- **Table sizes**: 10 scenes (matches expected), 404 samples, 31,206 `sample_data` records, 18,538 `sample_annotation` records, 911 instances, 8 logs.
- **Sweeps check (G3, critical)**: every one of the 12 channels has non-keyframe `sample_data` records (range 1,911–3,531 sweeps/channel). 20 randomly sampled sweep files per channel (240 files total) all exist on disk. No STOP triggered.
- **Devkit introspection**: `nuscenes-devkit==1.2.0` (via `importlib.metadata`; the package has no `__version__` attribute). Signatures for `box_velocity`, `get_sample_data_path`, `transform_matrix`, `LidarPointCloud.from_file`, `RadarPointCloud.from_file` printed in the log — these are what later steps must code against, not memory. `list_categories()`/`list_attributes()` output captured in the log: 18 categories with real counts (`vehicle.car` n=7619 is by far the largest; several rare classes like `human.pedestrian.personal_mobility` n=25), 8 attributes.
- **Native rate per channel** (median timestamp gap): LiDAR ≈20.1 Hz, all 5 radars ≈13.3–13.4 Hz, all 6 cameras ≈10.0 Hz. See "Surprising numbers" below for the camera figure.
- **Environment**: Python 3.12.8 in `.venv`; `torch` not installed (expected); an NVIDIA GPU is visible via `nvidia-smi` but nothing in this project uses it.

## Surprising or suspicious numbers (treat as bugs first)
- **All 6 camera channels report exactly median 100.00 ms (10.00 Hz)**, which is suspiciously round against nuScenes' commonly-cited ~12 Hz nominal camera rate — investigated before accepting it. Per-scene medians for `CAM_FRONT` are consistently 100.0 ms across all 10 scenes individually (not an artifact of merging scenes with different clocks). Looking at the raw gap histogram within a single scene (`scene-0061`, `CAM_FRONT`, 224 frames), gaps are bimodal: 71 gaps ≈50 ms and 143 gaps ≈100 ms, plus a handful at 150/200 ms — i.e. real per-file timestamp jitter, not a clean isochronous 10 Hz signal, with the 100 ms mode being the majority. I could not find a bug in the grouping/lookup code (checked at global, per-scene, and single-scene-raw-histogram levels, always resolving `channel` via each record's own `calibrated_sensor`→`sensor` lookup per G2, never assumed). Treating this as a genuine property of this dataset's camera capture timing rather than a script defect, but flagging it because it affects any step that assumes a clean 12 Hz camera cadence (e.g. `ENTERING_HORIZON_S`, camera-adapter native-rate assumptions in step04c) — worth a second look when that step is built.
- Radar (~13.3 Hz) and LiDAR (~20.1 Hz) rates matched documented expectations closely enough that no further digging seemed warranted.

## Assumptions introduced (each must be in config with a status)
All in `src/ttcf/config.py`, none MEASURED (verified by test). Full table below, generated by `config.describe()` (used verbatim, per step00 §4):

| Name | Value | Status | Tunable on | Reason |
|---|---|---|---|---|
| `ACTIVE_CHANNELS` | ['LIDAR_TOP', 'RADAR_FRONT', 'RADAR_FRONT_LEFT', 'RADAR_FRONT_RIGHT', 'CAM_FRONT'] | ASSUMED | never | DEC-3, approved as proposed: forward corridor only (README_MASTER.md §6) — a forward-TTC/AEB design has no use for side/rear sensors. |
| `CANDIDATE_EXTRA_MARGIN_M` | 1.5 | ASSUMED | never | Extra margin (m) added to the corridor when deciding which detections compete as track candidates, so an object about to enter the corridor is not missed. |
| `CHI2_GATE` | 9.21034037197618 | ASSUMED | never | Chi-square quantile at 99% confidence, df=2 — computed from scipy, not typed, so it can never silently drift from the confidence level it claims. Replaces a fixed Euclidean gate per G7/lessons item A.4 (gate on the innovation covariance S = P_pred + R, including the incoming detection's own noise). The 0.99 confidence level itself is an assumption, not measured. |
| `CORRIDOR_HALF_WIDTH_M` | 1.2 | ASSUMED | never | Half-width (m) of the ego vehicle's forward path corridor used for the forward-path relevance filter (step05). |
| `CORRIDOR_MAX_RANGE_M` | 60 | ASSUMED | never | Maximum forward range (m) considered for path relevance — beyond this, TTC is not meaningfully actionable at typical urban/highway speeds. |
| `DEBOUNCE_N` | 3 | ASSUMED | tune_split | Number of consecutive real updates the danger condition must hold before an action tier fires (G16). Tune split only. |
| `DISTANCE_DEFINITION` | nearest_surface_to_ego | PLACEHOLDER | never | DEC-1 recommendation (README_MASTER.md §6, option B) is used as the working value here, but DEC-1 itself is explicitly listed as 'must be settled now' at stepD0_gt_event_table.md, not at step00 — this value is a placeholder pending that explicit confirmation, not yet an answered decision. |
| `EGO_POSE_STD_M` | 0.05 | ASSUMED | never | nuScenes ego_pose comes from a fused localization stack (GPS/IMU/wheel odometry), far more accurate than any tracked sensor — literature-typical ~5cm automotive localization std, not independently measured here. |
| `ENTERING_HORIZON_S` | 1.5 | ASSUMED | never | Time horizon (s) used to admit an object that is clearly about to enter the corridor even though it is not in it yet. |
| `EVICTION_GAP_S` | 0.3 | ASSUMED | never | ~3-4 inter-arrival gaps of the slowest active sensor; identical in all four runs (G11). Deliberately NOT v2's MAX_MISSED_SECONDS=1.5 — this project's tracks are short-lived by design (2-4 updates), so lessons-from-v1-v2.md item C.1 says the old eviction tradeoff does not carry over unexamined. |
| `MAX_ASSOC_DIST_M` | 2.0 | ASSUMED | never | Distance threshold (m) for Hungarian-assigning a system track to a GT object when diagnosing which GT object a false brake was about (step09). |
| `MAX_VEL_STD_MPS` | 1.5 | PLACEHOLDER | never | Velocity-trust limit (m/s) — a track's velocity estimate is only trusted below this std. Measured in step06_tracker.md. |
| `MIN_CLOSING_SPEED_MPS` | 0.5 | PLACEHOLDER | never | Closing speed <= this returns TTC = inf rather than an astronomically large but meaningless finite value (G8). Distinct from MIN_TRUSTED_SPEED_MPS above — this gates the RADIAL-to-ego component, not raw speed magnitude. Measured the same way as MIN_TRUSTED_SPEED_MPS. |
| `MIN_COUNT_FOR_RATE` | 10 | ASSUMED | never | Below this many events in a denominator, a reported rate is labelled 'indicative only' rather than a solid statistic (what-not-to-do.md §7). |
| `MIN_LIDAR_PTS` | 5 | ASSUMED | never | A GT object with fewer than this many LiDAR points inside its box is treated as outside LiDAR's effective view, not a tracker failure (what-not-to-do.md §6). |
| `MIN_RADAR_PTS` | 1 | ASSUMED | never | Same purpose as MIN_LIDAR_PTS, for radar's num_radar_pts. |
| `MIN_TRUSTED_SPEED_MPS` | 0.5 | PLACEHOLDER | never | Below this raw speed, a track's motion is treated as sensor/clustering jitter on a stationary object rather than real motion (G8). Measured on stationary GT objects once GT event table exists (stepD0 / later). |
| `MIN_UPDATES_FOR_TTC` | 2 | ASSUMED | never | Fixed by physics, not tunable: a rate (velocity) cannot be computed from fewer than 2 temporally-linked detections of the same object, regardless of sensor or algorithm (CLAUDE.md 'Does require'). |
| `MIN_VISIBILITY_LEVEL` | 2 | ASSUMED | never | Minimum nuScenes visibility level (of 4: 0-40%, 40-60%, 60-80%, 80-100%) for a GT object to count as observable at all, camera-side. |
| `SENSOR_R_CAMERA` | None | PLACEHOLDER | never | Camera R is range-dependent (ground-plane back-projection error grows with distance), not a fixed (sigma_long, sigma_lat) pair — the function itself is defined and measured in step04c_camera_adapter.md. |
| `SENSOR_R_LIDAR` | {'sigma_long_m': 0.3, 'sigma_lat_m': 0.3} | ASSUMED | never | Literature-typical LiDAR range accuracy. A direct measurement (matched-track residual vs. sample_annotation GT) was attempted for this exact sensor in the predecessor project and found untrustworthy: LiDAR's own track fragmentation there (4.1x vs. real object count) meant only ~86% of matched points were real inliers, well below the >97% a robust estimator needs, so the resulting number (~2.0m) was physically implausible for LiDAR (lessons-from-v1-v2.md item A.10 / 10). Re-measurement here is only justified once this project independently confirms low fragmentation under its own short-memory tracker (step06). |
| `SENSOR_R_RADAR` | {'sigma_long_m': 0.5, 'sigma_lat_m': 1.5} | ASSUMED | never | Literature-typical radar accuracy (good range resolution, poor angular/lateral resolution). Same non-measurability rationale as SENSOR_R_LIDAR above. |
| `SIGMA_A` | 2.0 | PLACEHOLDER | tune_split | Shared object process-noise std (m/s^2). Coincides numerically with v2's own PROCESS_NOISE_SIGMA_A, but NOT inherited from it — lessons-from-v1-v2.md item C.1 says this tradeoff genuinely changes once track lifetime drops to 2-4 updates, so this value is locked by a fresh pre-declared mean-score sweep across all four streams on the tune split (DEC-7, step11_runs_tuning_report.md). |
| `SIGMA_A_EGO` | 1.0 | ASSUMED | never | Ego vehicle's own process-noise std (m/s^2) for the shared KF used to derive ego velocity from ego_pose (G5/G6). Ego motion is smoother/less erratic than a tracked object's, hence lower than SIGMA_A, but not yet measured. |
| `TTC_AEB_S` | 1.2 | ASSUMED | tune_split | TTC at or below this triggers the AEB action tier. Tune split only. |
| `TTC_GRADUAL_S` | 2.5 | ASSUMED | tune_split | TTC at or below this triggers the GRADUAL action tier. Tune split only. |
| `V0_STD_MPS` | 10.0 | ASSUMED | never | Initial velocity std (m/s) used to initialise a new track's KF covariance — deliberately large/uninformative so the first real update dominates. |

## Flagged, not built (out-of-scope temptations)
- Nothing beyond step00's deliverables was built. `configs/frozen_config.json` was deliberately NOT written yet — `freeze()` exists and is tested (via `tmp_path`, not a committed file), but the actual frozen-config artifact belongs to step11 per the build order table, not step00.
- Did not pre-create `outputs/cache`, `outputs/figs`, `outputs/tables` — only `outputs/logs` (for the preflight log) exists so far, per README_MASTER §3 ("create as steps require; do not pre-create empty scaffolding").

## Open questions for Siva
1. The camera 10 Hz-vs-~12 Hz-nominal observation above — worth a deeper look now, or fine to revisit at step04c when the camera adapter actually needs a native-rate assumption?
2. `DISTANCE_DEFINITION` in config is currently a PLACEHOLDER carrying DEC-1's recommendation (nearest-surface-to-ego) as a working value, since DEC-1 itself is explicitly scoped to be settled at `stepD0_gt_event_table.md`, not step00 — confirming that's the right place to leave it rather than asking DEC-1 early.
3. `requirements.txt` currently pins only this project's core set (numpy/pandas/scipy/matplotlib/nuscenes-devkit/pyquaternion/pillow/pytest) plus `nuscenes-devkit`'s own transitive deps (`scikit-learn`, `opencv-python-headless`) — neither of which this project's own code imports yet (confirmed by grep across all step files). `step04a_lidar_adapter.md` will ask explicitly whether to use `sklearn.cluster.DBSCAN` for radar clustering when we get there.

Waiting for "approved" before proceeding to `step02_transform.md` (build order #2).
