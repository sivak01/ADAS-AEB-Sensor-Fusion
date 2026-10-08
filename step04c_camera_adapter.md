# STEP 04c — Camera detection adapter (genuinely single-sensor)

**Build order #16.** Follow `README_MASTER.md` §4 and §8. Prerequisite: step 04b approved.

## 1. Purpose (plain language)
A camera gives 2D boxes in an image, not distances. To get a distance we need depth — and it must come **from the camera alone** (G14). If any LiDAR/radar/GT information leaks into this path, the "camera-only" baseline is fake and the four-way comparison is meaningless (this is V1's `camera` vs `camera_mono` mistake).

## 2. Deliverables
- `src/ttcf/adapters/camera.py`
- `scripts/cache_camera_detections.py` (detections cached to disk; inference is slow)
- `tests/test_camera_adapter.py`, `tests/test_camera_isolation.py`
- figures in `outputs/figs/step04c/`, camera-only result on the tune split
- report + learning note

## 3. Specification

Input: a `RawEvent` for `CAM_FRONT` (DEC-3). Output: `list[Detection]`.

### 3.1 2D detector
- Use a pretrained off-the-shelf detector (e.g., torchvision Faster R-CNN or an Ultralytics YOLO model). **Ask Siva before installing torch/ultralytics, and report GPU/CPU availability from step 00.** ~2,400 CAM_FRONT frames across 10 scenes is feasible on CPU but slow — cache results in `outputs/cache/camera_dets/` keyed by `(model name+version, sample_data_token)`.
- Keep classes relevant to the path (car, truck, bus, person, bicycle, motorcycle) and record the class in `Detection.cls` — but class is **not used for gating in any run** (DEC-2).
- Discard boxes truncated at the bottom image edge (ground-contact point unknown) and count them in the report.

### 3.2 Monocular depth by ground-plane geometry (DEC-6 — confirm with Siva)
For each box, take the **bottom-centre pixel** `(u, v)` (ground contact of the near face):
1. Ray direction in the camera frame: `K⁻¹ [u, v, 1]ᵀ` using `camera_intrinsic` from this event's `calibrated_sensor`.
2. Rotate to the ego frame with the camera's extrinsics from the same record (this event's own — G2).
3. Intersect with the ground plane `z = GROUND_Z_EGO` (config; ASSUMED default 0; verify with a diagnostic, below). Camera height comes from the calibration translation — this is **calibration, not LiDAR data**.
4. That intersection point is the reference point (nearest surface — consistent with DEC-1 B). If DEC-1 = centroid, state the resulting bias.
5. Convert to the global frame with this event's own `FrameContext`.

Ground-plane assumption = flat road. Document as a limitation (fails on slopes/pitching; range error grows ~ range²).

### 3.3 Range-dependent noise `R`
`R` is a **function of the detection**, not a constant (the filter accepts per-call `R`):
- `σ_long ≈ Z² · σ_v / (f · h_cam)` (depth error from a pixel error `σ_v` in the box bottom edge; `f` focal length in pixels, `h_cam` camera height above ground, `Z` range).
- `σ_lat ≈ Z · σ_u / f`.
- `σ_u`, `σ_v` (pixels) are ASSUMED config values (start ~5–10 px). Derive and show the formulas in the learning note.
- Rotate to global with `rotate_cov_ego_to_global`.
Label `R` MEASURED only if measured on the tune split with a trustworthy inlier rate; else ASSUMED.

### 3.4 Diagnostic for the ground-plane constant (tune split; GT allowed **only in diagnostic scripts, never in the adapter**)
Compare adapter-derived ground contact positions against GT box bottoms at high-visibility keyframes to check `GROUND_Z_EGO` and report range-dependent bias. Any adjustment to the config constant is a *calibration* done on the tune split and logged in `docs/decisions.md`.

## 4. Do NOT
- **No LiDAR, radar, or GT-derived depth or positions anywhere in the camera path** (what-not-to-do §6).
- No learned depth network unless Siva changes DEC-6.
- No per-frame tuning of detector thresholds against GT on the eval split.
- No borrowed timestamp/pose.
- No class-based gating (DEC-2).

## 5. Tests
1. **Isolation test (`test_camera_isolation.py`):** statically scan `camera.py` and its imports for `LidarPointCloud`, `RadarPointCloud`, `LIDAR_TOP`, `RADAR_`, `sample_annotation`, `box_velocity`; fail if found. Also run the adapter with only camera records/files available (via dependency injection or a monkeypatched loader that raises on any non-camera file) — it must still produce detections.
2. Ground-plane geometry on synthetic camera: a known ground point projects to a pixel and back to the same point (round trip).
3. `σ_long` grows quadratically with range (numeric check against the formula).
4. Global-frame check: a stationary object across two ego poses → same global position.
5. Cache: identical detections on a second run; cache invalidates when the model version changes.
6. Truncated-bottom boxes are dropped and counted.
7. `R_global` SPD.
8. Real data: overlay projected reference points on the image and BEV vs GT; report typical range error by distance band.

## 6. Camera-only run (tune split)
Run `--stream camera --split tune`. Show the camera-only row with raw counts, both observability variants, and the depth-error-by-range table. Investigate surprising numbers as bugs first.

## 7. Decisions to ask
DEC-6 (depth method), permission to install detector dependencies.

## 8. Learning-note topics
Why depth is the hard part of camera-only TTC; ground-plane back-projection with a worked numeric example; why depth error grows with range and why `R` must reflect it; why the isolation test exists.

## 9. STOP
Report and wait for "approved".
