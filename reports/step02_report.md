# Step 02 report

## What was built (files, one line each)
- `src/ttcf/geometry/__init__.py` — package init.
- `src/ttcf/geometry/transforms.py` — `pose_matrix`, `FrameContext` (token-cached `from_event`), `sensor_to_ego`/`ego_to_sensor`/`ego_to_global`/`global_to_ego`/`sensor_to_global`/`global_to_sensor`, `ego_yaw`, `rotate_cov_ego_to_global`, `ego_pose_timestamp`.
- `tests/test_transforms.py` — 6 tests per step02 §5 (4 synthetic, 2 real-data).
- `outputs/figs/step02/ego_motion_evidence.png` — the required demonstration figure.
- `outputs/logs/step02_per_channel_independence.txt` — real-data per-channel timestamp/pose output.

## Precedent check (V1/v2), ported vs. rebuilt, with why
- **Not ported:** V1/v2's hand-rolled `transform_matrix` (`src/geometry.py`). Mathematically sound on inspection, but step02_transform.md explicitly forbids hand-rolling quaternion math, and neither V1 nor v2 (which imports V1's `point_to_global` unchanged) ever used the devkit's own `nuscenes.utils.geometry_utils.transform_matrix`. Built fresh against the devkit function directly, using the exact signature step00's preflight already captured.
- **Ported (technique, not code):** the batched-homogeneous-matrix-multiply pattern (`points_h = hstack([points, ones]).T`, one matrix multiply for all N points) from `src/geometry.py`'s `points_to_global` — pure efficiency technique, correctness-independent of MOT-vs-event-level scope.
- **Ported (idiom):** `Quaternion(ego_pose["rotation"]).yaw_pitch_roll[0]` for yaw extraction, found in V1's `tests/test_geometry_projection.py` — correct use of the tested library method, not hand-rolled trig.
- **Ported (testing methodology, not code):** that same test file's principle of cross-validating against the devkit's own independent implementation and deliberately picking a high-yaw-rate sample "since a straight-line drive can't distinguish a wrong quaternion convention from a correct one." Applied here by independently validating the transform against `nusc.get_sample_data()`'s own box (a second, real, independently-computed reference), not just against a hand-derived example.
- **Nothing to port for `FrameContext`'s token cache or `rotate_cov_ego_to_global`** — no precedent exists in V1/v2 for either; built fresh.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -v`
```
18 passed, 14 warnings in 2.21s
```
(12 from step00 + 6 from step02: identity, round-trip, hand-computed 90°, covariance rotation, ego-motion evidence, per-channel independence. The 14 warnings are all `matplotlib`/`pyparsing` deprecation notices, unrelated to this project's code.)

## Real-data verification (what you ran, what it showed, figures saved where)
- **Independent cross-check against the devkit's own transform** (not just the hand-derived test 3): took a real GT box from `nusc.get_sample_data()` (already in LIDAR_TOP's sensor frame, computed by the devkit's own internal logic), applied this project's `sensor_to_global`, and compared against `sample_annotation.translation` — nuScenes' own authoritative global-frame ground truth. Result: floating-point-exact match (diff ~1e-16) at two independent keyframes. This is stronger evidence than the hand-computed test alone, since it validates against a second, independently-computed reference on real data, not just an analytic example.
- **Test 5 (ego-motion evidence)**: chosen instance's parked-vehicle GT box, ego displaced 4.81 m between two consecutive keyframes. Raw LiDAR-local centroid of the in-box points moved 4.58 m (tracks ego motion almost exactly); global-frame centroid of the same points agreed within 0.40 m across the two keyframes. Figure: `outputs/figs/step02/ego_motion_evidence.png`.
- **Test 6 (per-channel independence)**: across 10 samples × 4 non-LiDAR channels (40 comparisons), `ego_pose_token` differed from `LIDAR_TOP`'s in every single case (`same_ego_pose_token=False`, 40/40). Timestamp deltas ranged ~0.5–40 ms; ego displacement between the differing poses ranged ~4.5 mm to ~330 mm depending on channel and instant. Full output: `outputs/logs/step02_per_channel_independence.txt`. Also asserted structurally (not just by printing) that a radar event's `FrameContext` resolves to that radar record's own `ego_pose`, never LIDAR_TOP's.

## Surprising or suspicious numbers (treat as bugs first)
**Found and fixed one, before it could hide inside a passing-looking test.** The first version of test 5's candidate search selected the pair with the largest ego displacement among annotations carrying the `vehicle.parked` attribute, without independently checking the GT box's own position. That selected an instance (`b0e4c63c...`) attributed `vehicle.parked` on **all 21** of its annotations, whose own GT `translation` nonetheless advances ~2.7 m every keyframe (~5.4 m/s) — a moving vehicle mislabeled parked in this data, most likely stemming from weakly-fit early boxes with `num_lidar_pts: 0`. This produced a 2.095 m global-centroid "disagreement" that looked like a transform bug.

Investigated per G13 before accepting or dismissing it:
1. Independently verified the transform against `sample_annotation.translation` (nuScenes' own authoritative GT) at both keyframes separately — exact match (see Real-data verification above). **Confirmed the transform code has no bug.**
2. Printed the full 21-annotation chain for that instance and found its translation moving steadily and monotonically the whole time — confirming it's genuinely a moving object, not sensor/transform noise.
3. Fixed the root cause in the test itself, not the tolerance: candidate selection now additionally requires the GT box's own translation to move less than 0.3 m between the two chosen keyframes (`_MAX_TRUE_GT_DRIFT_M`), i.e. it verifies "really stationary" independently of the attribute label, the same discipline `what-not-to-do.md` §2 asks for toward sensor-reported values, applied here to a dataset attribute instead. Re-ran: now selects a genuinely-stationary instance (ego displaced 4.81 m, global centroid agrees within 0.40 m), and the transform code itself was never touched.

## Assumptions introduced (each must be in config with a status)
None. Step02 introduces no new tunable constants; `_MAX_TRUE_GT_DRIFT_M = 0.3` and `min_lidar_pts = 15` are test-selection parameters (which real-data example to pick for a demonstration), not pipeline behavior, so they live in the test file, not `config.py`.

## Flagged, not built (out-of-scope temptations)
- Did not add camera-projection helpers (`project_point_to_camera`-equivalents) — not in step02's deliverable list; that belongs to step04c's camera adapter.
- Did not investigate further why some early annotations of a genuinely-moving vehicle get labeled `vehicle.parked` in nuScenes-mini — noted as a real data-quality fact for future GT-handling steps (stepD0, step04a/b's DEC-1 reference-point logic) to be aware of, not something to fix here.

## Open questions for Siva
1. Should the `vehicle.parked`-mislabeling finding above be carried forward as a standing caution for `stepD0_gt_event_table.md` (which will build the GT event table and may also lean on attributes to reason about expected motion)?
2. Devkit's ego-frame convention matched the assumed x-forward/y-left/z-up throughout (no discrepancy found) — nothing to report per §6's fallback instruction.

Waiting for "approved" before proceeding to `step03_kf_ego_state.md` (build order #3).
