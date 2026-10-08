# STEP 02 — Coordinate transforms (sensor → ego → global)

**Build order #2.** Follow `README_MASTER.md` §4 and §8. Prerequisite: step 00 approved.

## 1. Purpose (plain language)
Every sensor reports in its own frame. To compare across time and sensors, everything must be in one world (global) frame. If this is wrong, a parked car appears to move at the ego vehicle's own speed. This is the most load-bearing utility in the project, so it is built once, tested hard, and used by every adapter (G1, G2).

## 2. Deliverables
- `src/ttcf/geometry/transforms.py`
- `tests/test_transforms.py`
- `outputs/figs/step02/` (evidence figure, see §5)
- report + learning note

## 3. Specification
Use the devkit's `transform_matrix` (verify its import path against the signature captured in step 00) and `pyquaternion`. **Do not hand-roll quaternion math.**

Functions (names may vary, semantics must not):
- `pose_matrix(translation, rotation) -> 4x4`
- `FrameContext.from_event(nusc, event: RawEvent)` → holds *that event's own* `calibrated_sensor` and `ego_pose` records, looked up **from `event.calibrated_sensor_token` and `event.ego_pose_token` only**. Cache by token.
- `sensor_to_ego(points, ctx)`, `ego_to_global(points, ctx)`, `sensor_to_global(points, ctx)`, `global_to_ego(points, ctx)` — accept (N,2) or (N,3).
- `ego_yaw(ctx) -> float` (rad).
- `rotate_cov_ego_to_global(R_ego (2,2), yaw) -> R_global (2,2)`  (`R_g = Rz · R_e · Rzᵀ`) — the adapters use this so per-sensor noise (given as σ_long, σ_lat in the ego frame) becomes a global-frame covariance.
- `ego_pose_timestamp(ctx)` — return the `ego_pose` record's own `timestamp`.

**Structural rule (G2):** no function may accept a bare token or a "sample" and quietly pick another channel's pose. The only way to get a transform is through an event's own record. There must be no function argument such as `channel=` that changes which pose is used.

Tracking works in global **xy** (bird's-eye view); keep `z` only where an adapter needs it (e.g., LiDAR ground removal).

## 4. Do NOT
- Reuse LIDAR_TOP's `ego_pose`/timestamp for radar or camera (what-not-to-do §2).
- Parse timestamps from filenames.
- Compare raw sensor-frame coordinates across time.
- Write per-adapter copies of transform code.

## 5. Tests
1. Identity pose → points unchanged.
2. Round trip sensor→global→sensor recovers points (tolerance 1e-9).
3. Known 90° yaw + translation example computed by hand.
4. Covariance rotation: 90° yaw swaps σ_long² and σ_lat².
5. **Ego-motion evidence (real data):** choose a GT instance with attribute `vehicle.parked` visible in ≥ 2 consecutive keyframes; take LiDAR points inside its GT box (devkit box-point helper) at each keyframe; transform to global. Assert the global centroid agrees within ~0.5 m across keyframes **while the raw sensor-frame centroid differs by roughly the ego displacement**. Save a figure showing both. This test *demonstrates the bug the utility prevents.*
6. **Per-channel independence (real data):** for several samples, show that `ego_pose_token` and `timestamp` of LIDAR_TOP, each RADAR and CAM_FRONT `sample_data` records differ; print the differences in ms and the vehicle displacement between them. Assert `FrameContext` for a radar event uses the radar record's own pose.

## 6. Decisions to ask
None. (If the ego-frame convention in the installed devkit differs from x-forward/y-left/z-up, report it.)

## 7. Learning-note topics
The three frames and what each transform does; why a stationary object looks like it is moving without ego compensation (worked numeric example: ego at 10 m/s over 0.1 s = 1 m apparent shift); why each channel has its own pose.

## 8. STOP
Report with the figure from test 5 and wait for "approved".
