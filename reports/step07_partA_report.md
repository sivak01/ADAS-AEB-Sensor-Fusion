# Step 07 Part A report

## What was built (files, one line each)
- `src/ttcf/ttc/ttc_math.py` — `TTCMathConfig`, `TTCMathResult`, `ttc_from_state()`: pure distance/closing-speed/TTC math, no I/O, no state.
- `tests/test_ttc_math.py` — 10 tests (9 from step07 §A4 + 1 extra covering the `closing_std` diagnostic from A2 step 6).

## A spec inconsistency found and resolved before writing code
step07_ttc.md's own prose, read completely literally, contradicts its own worked test. A2 step 1 defines `dx,dy = obj_ref_xy − ego_xy`; A2 step 3 defines `closing = ((vx−ego_vx)·dx + (vy−ego_vy)·dy) / distance` using the **object's** velocity for `vx,vy`. Implementing that literally against A4 test 1's own scenario (stationary object 30 m ahead, ego closing at 10 m/s) gives **closing = −10**, not the **+10** the test explicitly requires — verified numerically before writing any code (shown in-conversation). This is not a real design ambiguity: only one sign convention (a) reproduces the spec's own +10, (b) matches the physical definition of closing speed (positive when distance shrinks), and (c) matches v2's already-validated `ttc.py` (which uses `dx = ego − obj`, mathematically equivalent to what's implemented here). Implemented with the position vector unchanged (`dx,dy = obj − ego`, exactly as A2 step 1 states) and the relative velocity as **ego's velocity minus the object's** (the flip needed to correct the sign) — documented at the top of `ttc_math.py` with the exact numeric contradiction, so a future reader hitting the same confusion finds the resolution immediately rather than re-deriving it.

## Precedent check (V1/v2), ported vs. rebuilt, with why
- **Ported (sign-corrected)**: the core closing-speed formula and the `MIN_CLOSING_SPEED` deadband. v2's `ttc.py` already found and fixed, on real data, the exact "closing speed technically positive but negligible → astronomically large finite TTC" bug that A4 test 3 targets (v2 observed up to ~190 million seconds before its fix). Its module docstring's distinction between `MIN_TRUSTED_SPEED` (raw magnitude) and `MIN_CLOSING_SPEED` (radial component) is the same distinction step07 A2 draws — cited directly in this project's `config.py` at step00 already.
- **Not ported — genuinely new structure**: v2's `compute_ttc()` returns a plain 3-tuple with no `reason` enum, no trusted-speed zeroing step (that lived separately in `kalman_track.py`'s `trusted_velocity()`), and no `closing_std` uncertainty diagnostic. step07 Part A fuses all of this into one pure function returning a structured `TTCMathResult` — built fresh, no precedent for that shape.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -v`
```
40 passed, 14 warnings in 39.68s
```
All 10 of this step's tests pass (9 from §A4 + the `closing_std` extra), alongside the 30 from step00/02/03.

One test-only issue, not a code bug: `test_speed_below_trusted_equals_stationary_case` initially failed on a bare dataclass `==` comparison — both results legitimately have `closing_std_mps = NaN` (no covariance supplied to either), and `NaN != NaN` by IEEE754, so the equality check spuriously failed even though every field matched. Fixed by comparing fields explicitly (with `math.isnan()` for the NaN field) rather than relying on dataclass equality — `ttc_math.py` itself was never touched for this.

## Real-data verification
Not applicable to Part A — step07 §A1 lists only synthetic tests as this part's deliverable (real-data use of this function arrives via stepD0's GT builder and step07 Part B's tracker wrapper, both later in the build order).

## Surprising or suspicious numbers (treat as bugs first)
The sign-convention finding above is the headline item for this step — investigated by direct numeric verification against the spec's own worked example before writing any implementation code, not discovered after the fact by a failing test. Resolution is documented both here and at the top of `ttc_math.py`.

## Assumptions introduced (each must be in config with a status)
None. `TTCMathConfig` carries only `min_trusted_speed_mps`/`min_closing_speed_mps`, both already `PLACEHOLDER` in `config.py` since step00 (`MIN_TRUSTED_SPEED_MPS`, `MIN_CLOSING_SPEED_MPS`) — this function takes them as parameters (A3: "pass `cfg` in"), it doesn't define or duplicate them.

## Flagged, not built (out-of-scope temptations)
- No `TrackSnapshot`/`EgoState` wiring, no eligibility check (`MIN_UPDATES_FOR_TTC`, `MAX_VEL_STD_MPS`), no `critical_object()` selection — all explicitly Part B (step07 §B2), which needs the tracker (step06) first per the build order.
- No `BEHIND`/`OUT_OF_PATH` reason handling — A4 test 9 explicitly scopes that to the corridor filter (step05 Part B), not this pure function; verified only that an object behind ego doesn't crash and returns a sensible (finite distance, non-NaN closing, non-negative ttc) result.

## Open questions for Siva
1. ~~The sign-convention resolution above — confirm this matches your intent, or would you rather the step07_ttc.md prose itself be corrected~~ **Resolved:** `step07_ttc.md` A2 step 3 corrected on 2026-09-20 to `closing = ((ego_vx − vx)·dx + (ego_vy − vy)·dy) / distance`, with an inline note explaining why, at your request.

Waiting for "approved" before proceeding to `step05_forward_path_filter.md` **Part A** (build order #5).
