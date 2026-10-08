# STEP 03 — Shared Kalman filter and ego-state estimator

**Build order #3.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 00, 02 approved.

## 1. Purpose (plain language)
One constant-velocity Kalman filter class is the only velocity estimator in the project (G5, G6). It is used twice: for tracked objects (step 06) and for the ego vehicle's own velocity (this step) — because `ego_pose` has no velocity field, ego velocity must be *derived*, with the same discipline as any other velocity.

## 2. Deliverables
- `src/ttcf/filtering/kalman.py`
- `src/ttcf/filtering/ego_state.py`
- `tests/test_kalman.py`, `tests/test_ego_state.py`
- figures in `outputs/figs/step03/`
- report + learning note

## 3. Kalman filter spec (`ConstantVelocityKF`)
State `x = [x, y, vx, vy]` in the **global frame**. Measurement = position `[x, y]`, `H = [[1,0,0,0],[0,1,0,0]]`.

- Constructor takes **only** `sigma_a` (process noise; a property of the object's motion, not of any sensor).
- `init(t_us, z, R, v0_std)`: position from `z`, velocity 0, covariance = `R` for position and `v0_std²` for velocity.
- `predict(dt)`: `F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]`; process noise from the white-noise-acceleration model, per axis:
  `Q_axis = σ_a² · [[dt⁴/4, dt³/2],[dt³/2, dt²]]` (assemble into the 4×4 by axis).
- `update(z, R)`: **R is an argument of the call**, not stored in the object. Use the Joseph-form covariance update; re-symmetrise `P` after each step.
- `state_at(t_us)` → returns a **copy** predicted to time `t` without mutating (used for gating and for extrapolation later).
- `innovation_cov(R)` → `S = H·P·Hᵀ + R` and `mahalanobis_sq(z, R)` → `(z−Hx)ᵀ S⁻¹ (z−Hx)` on the *predicted* state. This is what the tracker gate will use (G7).
- The class must contain **zero references** to sensor names, modalities, or channels.

## 4. Ego-state estimator spec (`EgoStateEstimator`)
- Built from `ego_pose` records of the active channels' `sample_data` records: de-duplicate by `ego_pose_token`, sort by the **`ego_pose` record's own `timestamp`**.
- Feeds `(x, y)` from `ego_pose.translation` into the KF with `R = EGO_POSE_STD_M² · I` (from config).
- `state_at(t_us)` is **causal (G15)**: it uses only poses with timestamp ≤ `t_us`, predicts forward to `t_us`, and returns position, velocity, speed, and heading (yaw from the latest pose quaternion).
- The GT builder (step D0) uses this same estimator so both sides share one ego-velocity definition (G9).
- One estimator per scene; never carry state across scenes.

## 5. Do NOT
- Compute any velocity as `(p2−p1)/dt` anywhere (what-not-to-do §3).
- Store `R` in the filter, or let it know which sensor is reporting.
- Use a fixed Euclidean gate helper — only `mahalanobis_sq`.
- Look at poses in the future of `t_us` (G15).

## 6. Tests
**Kalman (synthetic):**
1. Noise-free constant-velocity input → velocity converges to the truth.
2. Noisy input (σ known) → velocity error shrinks with updates; report mean error after 2, 5, 20 updates.
3. Predict-only grows position variance; update shrinks it.
4. Update with huge `R` ≈ no change; with tiny `R` ≈ snaps to the measurement.
5. `P` stays symmetric and positive semi-definite after 1,000 steps.
6. **S = P + R test:** reproduce the worked example — prediction 20.0 m, P = 0.09 (σ 0.3), detection at 21.0 m with R = 0.64 (σ 0.8): gating on `P` alone rejects (d² ≈ 11.1 vs 1-D limit 6.63), gating on `S = P + R` accepts (d² ≈ 1.37). Assert both.
7. **Consistency (NEES):** over many Monte-Carlo runs the average normalised estimation error squared ≈ state dimension (4). Print it; this validates that `Q` and `R` are self-consistent.
8. `state_at` does not mutate the filter.

**Ego state (real data):**
9. For each of the 10 scenes, plot ego speed vs time; expect smooth, plausible values (urban: roughly 0–15 m/s). Identify any scene where the ego is (near) stationary and show speed ≈ 0 with jitter below `MIN_TRUSTED_SPEED_MPS`.
10. Causality test: the output at time `t` is identical if all poses after `t` are removed.
11. Report the lag: KF velocity vs a smoothed reference during a braking or accelerating segment (informational).
12. If a CAN-bus expansion exists in `archive` (check; likely absent), compare to its speed; otherwise state that no independent check exists.

## 7. Decisions to ask
None. Report the chosen numeric `SIGMA_A_EGO` behaviour (do the plots look over/under-smoothed?) and propose adjustments; do not silently tune.

## 8. Learning-note topics
What a Kalman filter does (predict/update, P as uncertainty, R and Q as trust knobs); why a raw two-point difference amplifies noise (worked example: 5 cm position noise over 0.05 s → 1 m/s velocity noise); why `R` is passed per update; what `S = P + R` means.

## 9. STOP
Report, include NEES output and ego-speed plots, wait for "approved".
