# Step 03 report

## What was built (files, one line each)
- `src/ttcf/filtering/kalman.py` — `ConstantVelocityKF`: `init`, `predict(dt)`, `update(z, R)` (Joseph form + re-symmetrise), `state_at(t_us)` (non-mutating peek), `innovation_cov(R)`, `mahalanobis_sq(z, R)`.
- `src/ttcf/filtering/ego_state.py` — `EgoStateEstimator`: one per scene, causal `state_at(t_us)`, native-rate ego_pose collection across all active channels, incrementally cached.
- `tests/test_kalman.py` — 8 synthetic tests (step03 §6, items 1-8).
- `tests/test_ego_state.py` — 4 real-data tests (items 9-12).
- `outputs/figs/step03/ego_speed_all_scenes.png` — speed-vs-time for all 10 scenes.

## Precedent check (V1/v2), ported vs. rebuilt, with why
See the full breakdown posted before building; summary:
- **Ported directly**: the F/Q matrix formulas (standard CV-KF math); R-per-`update()`-call / `sigma_a`-fixed-at-construction (v2's `kalman_track.py` already had this exactly right); the `S = P_pred + R` gating formula, validated in v2 by a real measured finding (omitting R from the gate inflated LiDAR/radar/camera_mono track counts by +99%/+58%/+1%, tracking exactly with each sensor's R); one ego-state KF per scene, reset on scene-token change (v2's `ego_pose_lookup()`).
- **Deliberately not ported**: v2's simple covariance update (this class uses Joseph form + re-symmetrisation per spec); v2's constructor baking in the initial position (this class separates bare construction from `init()` so a track's initial covariance reflects whichever sensor's real R started it); v2's `ego_pose_lookup()` only supporting lookup at existing 2Hz sample instants (this project reads native-rate `sweeps/`, so `state_at(t_us)` supports causal prediction to any timestamp); v2's single shared `sigma_a` for ego and objects (this project's `config.py` already deliberately split `SIGMA_A_EGO` from `SIGMA_A` at step00/DEC-9).

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -v`
```
30 passed, 14 warnings in 42.15s
```
All 8 synthetic Kalman tests + 4 real-data ego-state tests pass, plus the 18 from step00/step02.

Notable synthetic outputs:
- Test 2 (noisy input): mean velocity error by update count: `{2: 2.521, 5: 0.881, 20: 0.174}` m/s — shrinks monotonically as designed.
- Test 6 (worked example): `d²` on P alone = 11.11 (rejects vs. 1-D chi-square limit 6.635); `d²` on `S = P+R` = 1.37 (accepts) — exact match to the spec's worked numbers.
- Test 7 (NEES): mean NEES over 300 Monte-Carlo runs = **3.964** (state dimension = 4) — strong evidence the `Q`/`R`/update-equation implementation is internally self-consistent, not just "runs without crashing."

## Real-data verification (what you ran, what it showed, figures saved where)
- **Ego speed vs. time, all 10 scenes** (`outputs/figs/step03/ego_speed_all_scenes.png`): smooth curves throughout, all within urban plausibility (max observed well under 25 m/s). `scene-0553` is flat at exactly 0.000 m/s for all 398 samples — investigated (see below) and confirmed correct, not a bug. `scene-0757` and `scene-1100` show the vehicle decelerating to and holding near zero, with jitter below `MIN_TRUSTED_SPEED_MPS` once stopped.
- **Causality** (test 10): confirmed structurally — an estimator built from only the past-half of a scene's poses gives identical `state_at(t_mid)` output to the full-scene estimator queried at the same instant.
- **Lag report** (test 11, informational): scene-1094 (largest speed range, 12.6 m/s) — KF velocity vs. a raw finite-difference reference showed ~0.000 s lag at the native-rate (~50 ms) sampling resolution used for the check.
- **CAN-bus check** (test 12): no `can_bus` expansion present in `archive/` — documented as a fact; no independent vehicle-speed ground truth exists for this dataset, so the ego-speed checks above rely on `ego_pose`-derived KF velocity only.

## Surprising or suspicious numbers (treat as bugs first)
**Found and fixed a real bug during test development, not a flaky test.** The first version of `EgoStateEstimator.state_at()` failed on real-data tests with `predict(dt=-1.2e-06)` — `dt` coming out as a tiny *negative* number when `state_at()` was called with a `t_us` exactly equal to the last processed pose's own timestamp. Root-caused rather than papered over:
1. Traced it to the KF's internal clock being tracked as accumulated *float seconds* (`self._t_s += dt` on every `predict()` call) — after enough additions, floating-point rounding drifted the accumulated value by a few hundred nanoseconds away from the true value, so a `state_at()` call landing exactly on a real pose's timestamp could compute a spuriously negative `dt`.
2. Fixed the root cause: the internal clock is now kept as an **integer microseconds** field (`self._t_us`), matching this project's own timestamp convention (`t_us: int`, step00 `types.py`), with `predict()` advancing it via `round(dt * 1e6)` rather than accumulating float error. Also added a small tolerance (`dt >= -1e-6` clamped to 0) as a second line of defense against any residual floating-point noise, while still raising on a *meaningfully* negative `dt` (a real chronology bug).
3. Confirmed the fix against all 8 synthetic Kalman tests (still passing, including the NEES/Joseph-form tests) before trusting it on real data.

**Second finding, a performance bug rather than a correctness one**: the first version of `EgoStateEstimator.state_at()` rebuilt the entire KF chain from scratch on every call. Querying ~400 timestamps per scene against ~1000+ collected native-rate poses per scene (5 active channels) made the real-data test suite stall past a 3-minute background timeout. Fixed with an incremental cache (`_cache_kf`/`_cache_progressed_idx`) that only folds in poses newer than the last call for the common forward-sweeping access pattern, falling back to a from-scratch rebuild (still exactly correct, just slower) only for an out-of-order/backward query. Real-data suite runtime dropped from 3+ minutes (didn't finish) to 43 seconds.

## Assumptions introduced (each must be in config with a status)
None new. This step consumes `SIGMA_A_EGO`, `EGO_POSE_STD_M`, `V0_STD_MPS`, `CHI2_GATE`, `ACTIVE_CHANNELS` — all already in `config.py` with status from step00/DEC-9.

Requested by step03 §7 (report the chosen `SIGMA_A_EGO` behaviour, don't silently tune): the speed-vs-time plots look neither obviously over-smoothed (fast accelerations like scene-1077/scene-1094's ~0→13 m/s ramps are tracked without visible lag) nor under-smoothed (no visible jaggedness/noise amplification) at `SIGMA_A_EGO=1.0`. No adjustment proposed at this time — flagging for a second look once step06's tracker and step11's tuning give more scenes' worth of comparison.

## Flagged, not built (out-of-scope temptations)
- Did not build any trusted-velocity/eligibility gating (`MIN_TRUSTED_SPEED_MPS`, `MIN_UPDATES_FOR_TTC`) into `ConstantVelocityKF` itself — v2 baked a `trusted_velocity()` method directly onto its KF class; this project's step03 spec keeps the KF class free of any such policy ("zero references to sensor names... "), and that eligibility logic belongs to step06's tracker (which wraps a KF per track), not the bare filter.
- Did not further optimize `_collect_ego_poses`'s O(all sample_data) per-scene scan (still the dominant cost in the 43s real-data suite) — acceptable at this step's scale (10 scenes, test-only usage); revisit only if a later step's actual pipeline usage pattern makes it a real bottleneck.

## Open questions for Siva
1. `SIGMA_A_EGO=1.0` behavior reported above (neither obviously over- nor under-smoothed) — confirm no adjustment wanted before it's used downstream by stepD0's GT builder and later steps.
2. The lag-report test (informational) came out at ~0.000s at ~50ms sampling resolution — fine to leave as a coarse informational check, or worth a finer-resolution rerun later?

Waiting for "approved" before proceeding to `step07_ttc.md` **Part A** (build order #4).
