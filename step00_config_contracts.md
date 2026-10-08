# STEP 00 — Dataset preflight, config, and shared contracts

**Build order #1.** Follow `README_MASTER.md` §4 (protocol) and §8 (templates).

## 1. Purpose (plain language)
Before writing any pipeline code we (a) confirm the dataset on disk is what we think it is — especially that native-rate `sweeps/` exist — (b) create the ONE place every constant lives, and (c) define the small data types every later step passes around, so the steps plug together.

## 2. Deliverables
- `scripts/preflight_dataset.py`
- `src/ttcf/config.py`
- `src/ttcf/types.py`
- `tests/test_config.py`
- `docs/decisions.md` (create, empty log with header)
- `reports/step00_report.md`, `docs/learning/step00_learning.md`

## 3. Preflight script (run first, before anything else)
Dataset root is `F:\TTC Fusion\archive`. It may be a Kaggle-style download with a nested folder, so **do not assume the layout — discover it.**

1. Print the top-level entries; search up to depth 3 for the folder containing `samples/`, `sweeps/`, `maps/` and the version folder (e.g. `v1.0-mini/` with `sample_data.json`). Report the exact `DATAROOT` and `VERSION` found.
2. Load with `NuScenes(version=..., dataroot=..., verbose=True)`. Report: number of scenes (expect 10 for mini), samples, sample_data records.
3. **Sweeps check (critical):** for every channel, count `sample_data` records with `is_key_frame == False`, and confirm that for a random sample of 20 records per channel the file exists on disk. **If sweeps are missing or files are absent, STOP and tell Siva** — native-rate processing is the core of this project (G3).
4. Print the installed devkit version and `inspect.signature(...)` for: `NuScenes.box_velocity`, `NuScenes.get_sample_data_path`, `nuscenes.utils.geometry_utils.transform_matrix`, `LidarPointCloud.from_file`, `RadarPointCloud.from_file`. Also print `nusc.list_categories()` and `nusc.list_attributes()` output. Put all of this in the report — later steps rely on the *installed* API, not memory.
5. Check native rates per channel (median timestamp gap per channel from `sample_data.timestamp`). Report Hz.
6. Report Python version, whether a GPU/torch is present (do NOT install torch).

## 4. Config module (`config.py`)
Single source of truth (G12). Each parameter is a `Param(value, status, reason, tunable_on)` where `status ∈ {MEASURED, ASSUMED, PLACEHOLDER}` and `tunable_on ∈ {"tune_split", "never"}`. `PLACEHOLDER` = a guess that a named later step must replace with a measurement (name the step in `reason`).

Required functions:
- `describe()` → a markdown table of every parameter with value/status/reason (used verbatim in report footnotes).
- `freeze(path)` → writes JSON + SHA-256 hash of all values (used at step 11).
- `DATAROOT` from `TTCF_DATAROOT` env var, else the discovered path.

**Proposed starting values — present these to Siva as DEC-9 before locking. All are ASSUMED/PLACEHOLDER, none measured:**

| Parameter | Start value | Status / note |
|---|---|---|
| `ACTIVE_CHANNELS` | LIDAR_TOP, RADAR_FRONT, RADAR_FRONT_LEFT, RADAR_FRONT_RIGHT, CAM_FRONT | DEC-3 |
| `SENSOR_R` (ego frame, σ_long, σ_lat in m) | LiDAR (0.3, 0.3); radar (0.5, 1.5); camera = range-dependent function (see step04c) | ASSUMED (literature-typical); measured later on tune split (step04a/b/c) only if inlier rate is trustworthy |
| `SIGMA_A` (object, m/s²) | 2.0 | PLACEHOLDER → step11 sweep |
| `SIGMA_A_EGO`, `EGO_POSE_STD_M` | 1.0, 0.05 | ASSUMED |
| `V0_STD_MPS` (initial velocity std) | 10.0 | ASSUMED |
| `CHI2_GATE` | `scipy.stats.chi2.ppf(0.99, df=2)` (≈9.21) — compute, don't type | ASSUMED confidence level |
| `EVICTION_GAP_S` | 0.3 (≈ 3–4 gaps of the slowest active sensor); identical in all four runs | ASSUMED |
| `MIN_UPDATES_FOR_TTC` | 2 | fixed by physics (a rate needs 2 points) |
| `MAX_VEL_STD_MPS` (velocity trust limit) | 1.5 | PLACEHOLDER → measure step06 |
| `MIN_TRUSTED_SPEED_MPS` | 0.5 | PLACEHOLDER → measure on stationary GT objects |
| `MIN_CLOSING_SPEED_MPS` | 0.5 | PLACEHOLDER → same |
| `CORRIDOR_HALF_WIDTH_M` / `CORRIDOR_MAX_RANGE_M` | 1.2 / 60 | ASSUMED |
| `CANDIDATE_EXTRA_MARGIN_M` / `ENTERING_HORIZON_S` | 1.5 / 1.5 | ASSUMED |
| `DEBOUNCE_N` | 3 | ASSUMED, tune split only |
| `TTC_GRADUAL_S` / `TTC_AEB_S` | 2.5 / 1.2 | ASSUMED, tune split only |
| `DISTANCE_DEFINITION` | per DEC-1 | decision |
| GT observability: `MIN_LIDAR_PTS`, `MIN_RADAR_PTS`, `MIN_VISIBILITY_LEVEL` | 5, 1, level ≥ 2 | ASSUMED |
| `MIN_COUNT_FOR_RATE` | 10 (below → "indicative only") | ASSUMED |
| `MAX_ASSOC_DIST_M` | 2.0 | ASSUMED |

Rule: every `R` entry must say measured-vs-assumed and why (lessons A.10). Config must contain no logic beyond derived values like the chi-square quantile.

## 5. Shared types (`types.py`)
Frozen dataclasses. **Timestamps are integer microseconds** (`t_us`, as in nuScenes); convert to float seconds only inside math.

- `RawEvent`: `t_us, channel, modality, sample_data_token, ego_pose_token, calibrated_sensor_token, filename, is_key_frame, scene_token`. (Tokens come from that record only — G2.)
- `Detection`: `t_us, channel, modality, xy_global (2,), R_global (2,2), ref_point_kind (DEC-1), cls: Optional[str], n_points: Optional[int], aux: dict` (aux holds e.g. radar Doppler; the tracker must ignore it — G17).
- `TrackSnapshot`: `track_id, t_us, x (4,) [x,y,vx,vy], P (4,4), n_updates, sensors_in_estimate: frozenset[str], first_t_us`.
- `TTCResult`: `t_us, track_id, distance_m, closing_speed_mps, ttc_s (may be inf), reason: TTCReason enum {OK, NOT_ELIGIBLE, SPEED_DEADBAND_ZEROED, CLOSING_DEADBAND, OPENING, BEHIND, OUT_OF_PATH}, closing_std_mps`.
- `ActionTier` enum `{NONE, GRADUAL, AEB}` and `ActionDecision(t_us, tier, driving_track_id, consecutive_count)`.

## 6. Do NOT
- Do not put any constant outside `config.py` (G12). Do not write a number in a docstring that could drift from config.
- Do not label any `R` as MEASURED yet.
- Do not write into `archive/`.
- Do not start building the transform or anything else.

## 7. Tests (`tests/test_config.py`)
- Every `Param` has a status and a non-empty reason.
- No `MEASURED` status exists yet.
- `freeze()` hash changes when any value changes; is stable otherwise.
- `describe()` renders every parameter.
- `t_us` fields are ints (type check on dataclasses).

## 8. Decisions to ask
DEC-3, DEC-9 (present the table in §4). Record answers in `docs/decisions.md`.

## 9. Learning-note topics
Why measured-vs-assumed labelling matters (lessons A.10); why timestamps are integer microseconds; what "sweeps vs samples" means and what the preflight found.

## 10. STOP
Report and wait for "approved".
