# Step 04c report — Camera adapter (genuinely single-sensor) + first camera-only run

## What was built (files, one line each)
- `src/ttcf/adapters/camera.py` — `process_camera_frame`/`camera_detections`: RawEvent(CAM_FRONT) → list[Detection]. Reads pre-cached 2D boxes → class/truncation filter → bottom-centre pixel → camera ray via intrinsics → ground-plane intersection (DEC-6) → global frame, range-dependent `R`.
- `scripts/cache_camera_detections.py` — bulk YOLOv8n inference over native-rate CAM_FRONT frames, cached to `outputs/cache/camera_dets/<model>/<sample_data_token>.json`.
- `scripts/measure_sensor_R.py` — generalized further with `--sensor camera` (visibility-based "well-observed" criterion, since camera has no point-count field).
- `scripts/camera_adapter_evidence.py` — real-data BEV overlays + depth-error-by-range-band table, 3 tune-split scenes.
- `scripts/run_pipeline.py` — `_stream_adapter` extended for `--stream camera`.
- `tests/test_camera_adapter.py` (6 tests) + `tests/test_camera_isolation.py` (3 tests) — all pass.
- `outputs/figs/step04c/bev_*.png` (3 scenes, tune split).
- New `config.py` params: `CAMERA_DETECTOR_MODEL`, `CAMERA_DETECTOR_CONF_THRESH`, `CAMERA_DETECTOR_IOU_THRESH`, `CAMERA_RELEVANT_COCO_CLASSES`, `CAMERA_TRUNCATED_BOTTOM_MARGIN_PX`, `GROUND_Z_EGO`, `CAMERA_PIXEL_SIGMA_U_PX`, `CAMERA_PIXEL_SIGMA_V_PX` (all ASSUMED). `SENSOR_R_CAMERA`'s reason text updated to point at the actual range-dependent function rather than a placeholder promise.
- `requirements.txt` refreshed (torch CPU wheel, torchvision, ultralytics + transitive deps); `.gitignore` updated for `outputs/models/`.

## Precedent check (V1/v2) — done before coding
- **V1's exact mistake, confirmed and avoided by design**: V1 had two camera pipelines feeding its comparison table — `camera` (Step 2.3.1) looked up the median LiDAR point inside each 2D box for depth, so its "camera-only" row was secretly LiDAR+camera fusion; `camera_mono` (Step 2.3.2, built later specifically to correct this) used genuinely LiDAR-free similar-triangles depth (assumed per-class object height ÷ pixel height × focal length). V2 confirms this explicitly in its own README: "The old pipeline's LiDAR-assisted `camera` baseline has no role anywhere in v2 — not excluded-but-referenced, simply not part of this design."
- **PORTED**: Ultralytics YOLOv8n, CPU inference, conf=0.35/iou=0.45, COCO classes [0,1,2,3,5,7] — V1's own tuned choice on this exact dataset/model, confirmed with Siva before installing (our GPU can't meaningfully accelerate a heavier model).
- **NOT PORTED**: V1's `camera` (LiDAR-assisted) depth — forbidden by design; structurally prevented by `tests/test_camera_isolation.py`.
- **NOT PORTED**: V1's `camera_mono` similar-triangles depth — DEC-6 chose ground-plane back-projection instead, which has **zero precedent anywhere in V1/V2** (checked directly); built entirely fresh. Different bias mechanism: similar-triangles is systematic per-instance (assumed height is often wrong for *this* object); ground-plane is systematic per-range/pitch (grows with range², documented and empirically confirmed below).
- **NOT PORTED**: V1's unified 6-camera global tracker (Hungarian assignment, `DIST_THRESHOLD`, `MAX_MISSED_FRAMES`) — MOT-scope machinery this project already rejects (G11); this adapter only needs per-event `Detection` output.
- No detection-caching precedent existed in V1 (results were just overwritten per run) — `cache_camera_detections.py` built fresh per step04c §2's own explicit requirement.

## Decisions taken before coding
- **Detector**: Ultralytics YOLOv8n, CPU inference. Our GPU (GeForce GT 710, 2GB VRAM) is confirmed too old/weak to meaningfully accelerate inference (`torch.cuda.is_available()` correctly returns `False` after a CPU-only torch install).
- **DEC-6**: ground-plane back-projection (bottom-centre pixel → camera ray via intrinsics → rotate to ego via extrinsics → intersect `z=GROUND_Z_EGO`), confirmed before coding — the spec's own proposed default, built fresh (no precedent to port).

## §1 — Dependency installation: a real conflict found and fixed
Installing `ultralytics` pulled in `numpy>=2`, breaking `nuscenes-devkit` (`<2.0.0`) and `scipy` (`<2.3`). **Investigated and fixed properly, not papered over**: pinned `numpy==1.26.4` back down, which then conflicted with the newer `opencv-python` `ultralytics` also wanted; pinning an older `opencv-python==4.10.0.84` resolved it. A second, subtler issue: `nuscenes-devkit` nominally depends on `opencv-python-headless` while `ultralytics` needs the GUI-capable `opencv-python` — having **both** installed simultaneously (briefly, mid-fix) silently corrupted the shared native `cv2` module (`cv2.__file__` became `None`, `cv2.imencode` disappeared) even though `pip check` reported nothing broken at that point. Fixed by force-reinstalling `opencv-python` alone and removing `-headless` entirely; verified `cv2` fully functional (`imencode` round-trip) and the **full 124-test suite (all prior steps) still passed** before writing any camera-specific code. `requirements.txt` documents this combination and why `pip check`'s remaining nominal complaint is safe to ignore.

## §2 — Ground-plane constant sanity check (`GROUND_Z_EGO`, step04c §3.4)
Direct GT-only check (no adapter needed): for 7,858 real GT objects (tune split, within 60m), computed each one's own true bottom height in the ego frame (`translation.z − size.height/2`, converted via `global_to_ego`). **Median = 0.049m, mean = 0.085m** — both very close to the `GROUND_Z_EGO=0.0` default, confirming it needs no recalibration. The spread (std=0.73m, 5th–95th percentile −1.07m to +1.32m) is consistent with real road slope/pitch variation across scenes — exactly the documented limitation, not a hidden one.

## §3 — Measuring R (`scripts/measure_sensor_R.py --sensor camera`, §3.3)
Scope: tune-split, in-path GT objects at `visibility_token >= 4` (fully visible — camera has no point-count field like LiDAR/radar, so nuScenes' own visibility level is the closest equivalent to "well-observed"). N=88 candidates, 31 total misses (no camera detection nearby at all), inlier rate (residual < 2.0m) = **27.3% (24/88)** — well below the 70% MEASURED threshold. Robust σ_long=1.070m, σ_lat=0.467m (informational only, not applied). **`SENSOR_R_CAMERA` stays ASSUMED** — unlike LiDAR/radar this was not a close call requiring a decision; monocular ground-plane depth is genuinely far less accurate, exactly as physically expected, and the measurement correctly reflects that rather than showing a suspiciously good number.

## §4 — Real-data evidence: depth error grows with range², empirically confirmed
`scripts/camera_adapter_evidence.py`, 3 tune-split scenes, 119 keyframes, nearest-GT-object error by range band:

| Range band | n | mean error | median error |
|---|---|---|---|
| 0–15m | 143 | 1.94m | 1.78m |
| 15–30m | 221 | 2.39m | 2.07m |
| 30–50m | 133 | 4.78m | 4.01m |
| 50–200m | 165 | 24.59m | 17.29m |

This is a direct, empirical confirmation of the range²-growth limitation step04c §3.2 names explicitly — median error roughly triples from the near band to 30-50m, then jumps an order of magnitude beyond 50m, exactly the shape the physics predicts (a fixed pixel error at the box's bottom edge maps to a rapidly growing ground-distance error as the viewing ray becomes more grazing at range).

## §5 — First camera-only run (tune split, `--stream camera --split tune`)
Command: `.venv\Scripts\python.exe scripts\run_pipeline.py --stream camera --split tune`. All 5 tune scenes, native-rate camera stream, scored against every tune-split keyframe with ≥1 in-path GT object (n_scored=79).

| Variant | n_scored | correct_action_rate | false_brake_rate | missed_brake_rate |
|---|---|---|---|---|
| all (unrestricted GT) | 79 | 94.9% (75/79) [0.88, 0.98] | 0.0% (0/75) [0.00, 0.05] | 100% (4/4) [0.51, 1.00] — indicative only |
| obs_camera (camera-observable GT only) | 79 | 94.9% (75/79) [0.88, 0.98] | 0.0% (0/75) [0.00, 0.05] | **100% (4/4) — identical to "all"** |

`no_estimate_reason_breakdown`: `NOT_ELIGIBLE`=2, `NONE_IN_PATH`=1, `STALE`=1.

Tracker counters per scene (matches/spawns/evictions):
| scene | matches | spawns | evictions |
|---|---|---|---|
| scene-0103 | 173 | 10 | 9 |
| scene-0061 | 138 | 20 | 19 |
| scene-0655 | 8 | 6 | 5 |
| scene-0796 | 30 | 7 | 5 |
| scene-1094 | 271 | 21 | 20 |

**Surprising number investigated (G13): unlike LiDAR/radar, both variants are IDENTICAL — the GT observability flag says camera COULD see all 4 danger events, yet the system still missed all 4.** This is a materially different story from LiDAR (0/4 observable, a genuine blind spot) and radar (1/4 observable). Traced directly by re-running the adapter on the exact 4 danger keyframes (same scene-0103 fast-closing car found in step04a/b):
- At the first 2 keyframes (`3950bd41`, `c5f58c19`), the detector **did** produce a plausible in-corridor "car" detection close to the real object's position (e.g. range=31.3m vs GT's 38.15m; range=21.7m vs GT's 30.31m — off by several metres, consistent with §4's own measured near/mid-range error, but correctly landing inside the candidate gate and corridor).
- At the last 2 keyframes (`747aa46b`, `f4f86af4`), **no detection anywhere near the real car's position exists at all** — a 2D-detector recall miss on that specific frame, not a depth-estimation failure.
- One box at `747aa46b` produced a **range of 1,696m** (car "detected" 1.7km away) — a concrete example of the ground-plane method's known numerical instability near the horizon: as a box's bottom pixel approaches the vanishing point, the ray becomes nearly parallel to the ground plane and the intersection distance blows up. Harmless in this specific case (the point lands far outside the corridor regardless), but a real, now-empirically-observed failure mode worth a numeric-stability guard in a later step (flagged, not built — out of scope here).

**Conclusion**: camera-only misses this event not because it's a genuine blind spot (unlike LiDAR/radar's story), but because monocular depth noise plus imperfect frame-to-frame 2D-detector recall meant the track never accumulated enough consistent, in-corridor real updates to become eligible before the danger window passed. This is an honest capability gap, not a bug — and exactly the kind of finding the observability-variant design exists to surface precisely rather than blur into a single "missed" number with no context.

**False-brake rate is a clean 0.0%** — no spurious triggers, consistent with LiDAR's and radar's own clean results.

No fusion run yet — all three single-sensor adapters now exist; the fused run and the full 4-run comparison table are step11's job.

## Tests
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
133 passed, 14 warnings in ~45s
```
All 9 of this step's tests pass (6 in `test_camera_adapter.py`, 3 in `test_camera_isolation.py`), alongside the 124 from prior steps — no regressions.

One test-writing mistake caught before it became a false pass: my first version of every geometry test used an identity rotation for the camera's own extrinsic calibration. A real camera's own frame (x-right, y-down, z-forward) is a *different axis convention* than ego's (x-forward, y-left, z-up) — an identity rotation isn't physically valid and made every ray point upward instead of toward the ground, so every test either returned zero detections or asserted a physically-wrong result. Investigated per G13 before "fixing" the assertions: traced it to the rotation, not the adapter, and fixed every synthetic test to reuse nuScenes' own real CAM_FRONT calibration quaternion instead of inventing an ad hoc one.

## Real-data figures
`outputs/figs/step04c/bev_scene-0103_5b03af7a.png`, `bev_scene-0061_b26e7915.png`, `bev_scene-1094_0e837c21.png` — GT boxes (green squares) vs. adapter detections (gray = outside the candidate gate, red = in-path candidate).

## Flagged, not built (out of scope here)
- A numeric-stability guard against near-horizon ground-plane blow-ups (§5's 1,696m example) — did not affect this step's results but is a real, now-observed failure mode worth addressing before any tuning step relies on camera range values directly.
- Fused run and the 4-run comparison table — needs step11.
- Any tuning of `CAMERA_DETECTOR_CONF_THRESH`/`GROUND_Z_EGO`/pixel-sigma values against outcomes — explicitly out of scope for this step, same rule as step04a/b.
