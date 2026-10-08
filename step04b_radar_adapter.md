# STEP 04b — Radar detection adapter (Doppler OFF)

**Build order #15.** Follow `README_MASTER.md` §4 and §8. Prerequisite: step 04a approved.

## 1. Purpose (plain language)
Radar returns several points per real object per scan (bumper, wheel well, panel). If those go into the tracker as separate detections, the filter treats them as repeated noisy measurements of one point and gets corrupted (G4). This adapter clusters per channel, per scan, and emits one detection per object. It runs with **Doppler disabled** in the tracker (G17) — a deliberate scope decision that must be disclosed wherever results appear.

## 2. Deliverables
- `src/ttcf/adapters/radar.py`
- extension of `scripts/measure_sensor_R.py` for radar
- `tests/test_radar_adapter.py`
- figures in `outputs/figs/step04b/`, radar-only result on the tune split
- report + learning note

## 3. Specification

Input: a `RawEvent` for one of the active radar channels (DEC-3: `RADAR_FRONT`, `RADAR_FRONT_LEFT`, `RADAR_FRONT_RIGHT`). Output: `list[Detection]`.

1. Load with `RadarPointCloud.from_file`. **Check what the devkit's default filters (invalid/ambiguous/dynamic-property states) already do** (see step 00 signature output; devkit exposes class-level filter lists and a way to disable them). Report which points survive and which filter settings you use. Do not duplicate filtering the devkit already offers.
2. Transform points → ego → global with **this radar event's own** `FrameContext` (G2 — never LIDAR_TOP's pose or timestamp; this exact bug was found twice).
3. **Cluster per channel, per scan** (distance-based; `eps` and `min_points` in config, ASSUMED; radar `eps` is larger than LiDAR's). Cluster in xy; ignore z (unreliable).
4. Per cluster → one `Detection`:
   - Reference point per DEC-1 (nearest-surface: robust near-range point of the cluster; centroid otherwise — state rule).
   - `R_global` from `SENSOR_R[RADAR]` (σ_long, σ_lat in the ego frame — radar is good in range, poor laterally), rotated to global.
   - `cls = None`.
   - `aux` = `{doppler_vx_comp, doppler_vy_comp, rcs, n_points, x_rms, y_rms}` — stored for a *future* ablation, **ignored by the tracker** (G17).
5. Stamp detections with **the radar event's own `t_us`**.

**Known radar issue to document (do not hack around silently):** stationary clutter (overhead signs, guard rails, manhole covers) can produce ghost in-path returns and hence phantom brakes. The forward-path filter, tracker, and debounce are the pipeline's only defences; report how many false brakes in the radar-only run trace to such returns (via step 10 attribution).

## 4. Do NOT
- No raw per-point returns to the tracker (what-not-to-do §2).
- No use of radar Doppler / `relative_speed` for TTC or in the KF in v1 (§3). Doppler as an ablation needs Siva's explicit approval (G17).
- No borrowed timestamp/pose from another channel.
- No classification, no LiDAR/camera information.
- No evaluation on the eval split.

## 5. Tests
1. Several synthetic blips on one 4.5 × 1.8 m footprint in one scan → **one** detection.
2. Two objects 5 m apart → two detections.
3. Detections from each radar channel carry that channel's own `t_us`/pose (assert on tokens).
4. **Doppler-ignored test:** tracker output is identical with `aux` stripped (re-uses step 06 test on radar-shaped data).
5. Global-frame check: stationary object from two ego poses → same global position.
6. Real data: for a keyframe, plot raw radar points + clusters + GT boxes; report raw-points-per-object vs clusters-per-object statistics on a few scenes.
7. `R_global` SPD and correctly rotated.

## 6. Measuring `R` (tune split only)
Same procedure as 04a using `num_radar_pts` to select well-observed GT objects. Label MEASURED only with a trustworthy inlier rate (state your threshold); otherwise ASSUMED with the reason.

## 7. Radar-only run (tune split)
Run the pipeline with `--stream radar --split tune`. Show the radar-only row with raw counts, both observability variants, and this sentence in the caption: **"Radar ran with Doppler disabled; this row reflects that configuration, not radar's real capability."** Investigate any surprising number as a bug first.

## 8. Learning-note topics
Why radar has multiple returns per object; what clustering fixes; why radar velocity (Doppler) is a huge potential advantage and why leaving it out makes this baseline conservative; the ghost-return problem.

## 9. STOP
Report and wait for "approved".
