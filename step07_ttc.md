# STEP 07 — Time-to-Collision

Built in **two parts at different points in the build order** (see `README_MASTER.md` §7).
- **Part A (order #4):** pure TTC math + synthetic tests. Needed early because the GT table uses the exact same function.
- **Part B (order #10):** track-level wrapper, eligibility, and critical-object selection. Needs the tracker.

Follow `README_MASTER.md` §4 and §8.

## 1. Purpose (plain language)
TTC = distance ÷ closing speed. Closing speed must be the speed at which the gap is shrinking *relative to the ego vehicle* — not the object's raw speed. Every subtle mistake in this project's history lives here, so this module is pure, small, and heavily tested.

---

# PART A — Pure math (`src/ttcf/ttc/ttc_math.py`)

## A1. Deliverables
`ttc_math.py`, `tests/test_ttc_math.py`, report + learning note (Part A).

## A2. Specification
Single pure function (no I/O, no state), used by the tracker path AND the GT builder:

```
ttc_from_state(obj_ref_xy, obj_v_xy, ego_xy, ego_v_xy, cfg) -> TTCMathResult
```
Steps:
1. `dx, dy = obj_ref_xy − ego_xy`; `distance = hypot(dx, dy)` — using the reference point defined by **DEC-1** (same definition for tracker and GT — G9).
2. **Trusted-speed deadband:** if the object's absolute speed `|v_obj| < MIN_TRUSTED_SPEED_MPS`, set `v_obj := 0` (treat as jitter/stationary). Record reason `SPEED_DEADBAND_ZEROED`. (Note: a stationary object still has a real closing speed if the ego approaches it.)
3. `closing = ((ego_vx − vx)·dx + (ego_vy − vy)·dy) / distance` — `closing` is positive when the gap shrinks (G8). [Corrected 2026-09-20: the previous `(vx − ego_vx)` ordering, combined with step 1's `dx = obj − ego`, gives the NEGATIVE of closing speed — verified against this file's own worked example (A4 item 1: stationary object 30 m ahead, ego closing at 10 m/s must give closing=+10, not −10). This ordering matches that example and v2/ttc.py's already-validated formula.]
4. **Closing deadband:** if `closing <= MIN_CLOSING_SPEED_MPS` → `ttc = inf` (reason `CLOSING_DEADBAND`, or `OPENING` if `closing < 0`). Not just `<= 0`.
5. Else `ttc = distance / closing`.
6. Return `distance, closing, ttc, reason`. Also accept an optional velocity covariance block and return `closing_std = sqrt(uᵀ·P_vv·u)` where `u = (dx,dy)/distance`, for the uncertainty diagnostic (DEC-10).

`MIN_TRUSTED_SPEED` and `MIN_CLOSING_SPEED` are **separate checks on separate quantities** and must be separate config values (what-not-to-do §5).

## A3. Do NOT
- Use the object's absolute velocity as closing speed.
- Use `<= 0` as the only guard.
- Merge the two deadbands.
- Read config inside the function body implicitly — pass `cfg` in, so tests can vary it.

## A4. Tests (all must pass; these target mistakes already made once)
1. Stationary object 30 m ahead, ego 10 m/s toward it → closing 10, TTC 3.0 s.
2. Two vehicles co-travelling at 15 m/s, 20 m apart → TTC = ∞.
3. Near-tangential relative motion (closing just below `MIN_CLOSING_SPEED`) → `inf` with reason `CLOSING_DEADBAND`.
4. Opening gap → `inf`, reason `OPENING`.
5. Object speed 0.2 m/s (< trusted) → treated as 0; result equals the stationary-object case.
6. Independence of deadbands: object with high raw speed moving purely tangentially passes the trusted-speed check but hits the closing deadband.
7. Frame invariance: rotate/translate the whole scene → identical TTC.
8. Scaling: doubling distance at fixed closing doubles TTC.
9. Object behind ego (dot product with ego heading < 0) is handled by the caller via the corridor (Part B), but assert the function returns a sensible result, not a crash.

## A5. Learning-note topics (Part A)
Worked example for tests 1–3 in numbers; why ignoring ego velocity is dangerous on highways; why `<= MIN_CLOSING_SPEED` beats `<= 0` (a TTC of 40,000 s is a nonsense signal, functionally as bad as a missing one).

## A6. STOP after Part A. Wait for "approved".

---

# PART B — Track-level estimator (`src/ttcf/ttc/estimator.py`)

Prerequisites: steps 06 (tracker) and 05 Part B (path relevance) approved.

## B1. Deliverables
`estimator.py`, `tests/test_ttc_estimator.py`, report + learning note (Part B).

## B2. Specification
- `TTCEstimator.estimate(snapshot: TrackSnapshot, ego: EgoState) -> TTCResult`.
- **Eligibility:** `n_updates >= MIN_UPDATES_FOR_TTC` **and** velocity std (from `snapshot.P`) `<= MAX_VEL_STD_MPS`; otherwise `reason = NOT_ELIGIBLE`, `ttc = inf` (and flagged, so the harness can distinguish "no estimate" from "safe"). Ego state is taken at the snapshot's own time (causal — G15).
- Calls `ttc_from_state` from Part A unchanged.
- Fills `closing_std_mps`. Optionally provide `ttc_conservative = distance / (closing + k·closing_std)` as a **diagnostic only** (DEC-10).
- `critical_object(results, relevance)` → among tracks the step-05 Part B relevance function marks path-relevant, pick the smallest finite TTC; return the id and result, or `None`.

## B3. Do NOT
- Do not compute velocity anywhere except from the tracker's KF state (G5).
- Do not treat `NOT_ELIGIBLE` as "safe": it must be visible downstream.
- Do not let class gate relevance (DEC-2).

## B4. Tests
1. Track with 1 update → `NOT_ELIGIBLE`.
2. Track with wide velocity covariance → `NOT_ELIGIBLE` even with many updates.
3. Critical-object selection picks the lowest TTC among in-path tracks and ignores out-of-path ones.
4. Ego velocity used is the value at the snapshot's timestamp, not the latest (causality test).
5. Synthetic closing scenario end-to-end (tracker → estimator) yields TTC within tolerance of the analytic value after eligibility.

## B5. Decisions to ask
DEC-10 (point estimate vs conservative bound for action).

## B6. STOP after Part B. Wait for "approved".
