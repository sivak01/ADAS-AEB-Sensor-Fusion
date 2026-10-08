# STEP 04a — LiDAR detection adapter + first end-to-end run

**Build order #14.** Follow `README_MASTER.md` §4 and §8.
Prerequisites: all of steps 00–03, 05, 06, 07, 08, 09, 10, D0 approved.

## 1. Purpose (plain language)
nuScenes provides raw LiDAR point clouds, **not detections**. This adapter turns each LiDAR sweep into a short list of object candidates in the global frame, each with a position and a noise estimate `R`. It also builds the runner that pushes a single sensor's stream through the *whole* pipeline, so the first end-to-end result exists early — on the **tuning split only**.

## 2. Deliverables
- `src/ttcf/adapters/lidar.py`
- `scripts/run_pipeline.py` (single-stream version: `--stream lidar --split tune`; generalised in step 11)
- `scripts/measure_sensor_R.py` (see §5)
- `tests/test_lidar_adapter.py`
- figures in `outputs/figs/step04a/`, first four-column *lidar-only* result on the tune split
- report + learning note

## 3. Specification

Input: a `RawEvent` for `LIDAR_TOP`. Output: `list[Detection]` (step 00 type).

Pipeline for one sweep:
1. Load with `LidarPointCloud.from_file` (points are sensor-frame x, y, z, intensity).
2. Transform to the **ego frame** via `FrameContext.from_event(event)` (G2 — this sweep's own pose).
3. Coarse **ROI crop** in the ego frame (a superset of the candidate corridor, e.g. `0 < x < max_range + slack`, `|y| < candidate half-width + slack`, `z` within a plausible vehicle band). This is only for compute; the *decision* filter is step 05's candidate gate. State the ROI in config.
4. **Ground removal:** fit a plane (RANSAC on near-field low points) per sweep and drop points within a height threshold of it. Report the plane parameters per sweep on a few scenes (sanity), and how many points are removed. (Start simple; report limitations on slopes.)
5. **Clustering:** DBSCAN-style clustering (`eps`, `min_points` in config, ASSUMED). If scikit-learn is not installed, use a `scipy.spatial.cKDTree` connected-components implementation — **ask before installing sklearn**.
6. Per cluster → one `Detection`:
   - **Reference point per DEC-1.** If nearest-surface (recommended): use a robust estimate of the near face — e.g., the mean of the cluster points within ~0.3 m of the nearest point by range — not the single minimum point (too noisy). State the exact rule in the report. If centroid: state its bias.
   - `R_global` from `SENSOR_R[LIDAR]` given in ego-frame (σ_long, σ_lat), rotated to global with `rotate_cov_ego_to_global` (step 02).
   - `cls = None`; keep `n_points` and cluster extent in `aux` (not used for gating — DEC-2).
   - `xy_global` via `ego_to_global`.
7. Return detections stamped with **the event's own `t_us`** (G2).

## 4. Do NOT
- Do not apply the precise corridor here (that's step 05); only the coarse ROI.
- Do not classify objects with size heuristics (DEC-2).
- Do not use radar/camera/GT data for anything.
- Do not borrow another channel's timestamp/pose.
- Do not feed per-point returns to the tracker; only clusters.
- Do not evaluate on the eval split.

## 5. Measuring `R` (optional but valuable; tune split only)
`measure_sensor_R.py`: at keyframes where GT objects have high `num_lidar_pts`, compute residuals between the adapter's reference point and the GT reference point (DEC-1 definition), in the ego frame (longitudinal and lateral separately). Report **inlier rate** (residuals within a sane gate, e.g. < 2 m) and robust σ (MAD-based). **Label `R` as MEASURED only if the inlier rate is high and stable (state your threshold, e.g. > 70%);** otherwise keep ASSUMED and say why (lessons A.10: V1's LiDAR/radar `R` could not be measured because fragmentation corrupted the residuals). Never silently swap in a measured value.

## 6. Tests
1. Synthetic cloud: ground plane + one 4.5 × 1.8 m box → exactly one detection at the expected reference point.
2. Two boxes 3 m apart → two detections; two touching boxes → document the behaviour.
3. Ground-only cloud → zero detections.
4. Detection positions are in the **global** frame: a stationary synthetic object seen from two ego poses gives the same global position (G1).
5. Timestamp/pose come from the event's own record.
6. `R_global` is symmetric positive-definite; yaw rotation applied correctly.
7. Real data: on 3 keyframes, overlay detections vs GT boxes in BEV; report the number of detections vs in-path GT objects (recall/false-detection *by eye*, informational).

## 7. First end-to-end LiDAR-only run (tune split)
Run event stream → adapter → step 05 candidate gate → tracker → TTC → path relevance → debounce → step 09 → step 10 (**tune scenes only, `--final` NOT used**). Show:
- The metrics table (LiDAR-only row) with raw counts, both observability variants.
- `no_estimate_reason` breakdown.
- Tracker counters (matches / spawns / evictions).
- Any surprising number — investigate as a **likely bug first** (G13) and report the investigation. Do not tune to improve it in this step.

## 8. Decisions to ask
DEC-1 confirmation if not already recorded; whether to install scikit-learn.

## 9. Learning-note topics
Raw point cloud vs detection; ground removal; clustering; why nearest-surface is a better TTC reference than centroid (worked example); what "measured vs assumed R" means for this sensor.

## 10. STOP
Report with figures and the LiDAR-only row; wait for "approved".
