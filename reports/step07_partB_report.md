# Step 07 Part B report

## What was built (files, one line each)
- `src/ttcf/ttc/estimator.py` — `TTCEstimator.estimate()`, `TTCEstimator.conservative_ttc()` (diagnostic only), `critical_object()`.
- `tests/test_ttc_estimator.py` — 6 tests (5 from step07 §B4 + 1 extra covering `critical_object`'s "nothing qualifies" case).

## Decisions taken (DEC ids + Siva's answers)
- **DEC-10** — TTC point estimate drives the action decision (step08); the conservative bound (`distance / (closing + k·closing_std)`) is computed and exposed via `conservative_ttc()` but is diagnostic only, never acted on, for v1. Per README_MASTER.md §6 recommendation.

## Precedent check (V1/v2)
Searched v2 for any "critical object" / "driving track" selection logic — found nothing (one match on "critical" turned out to be an unrelated chi-square docstring, `"Chi-square critical value..."`). This makes sense: V1/v2 evaluated *every* track against ground truth (an MOT-style all-tracks comparison); neither ever needed to pick a single object to act on, since that's an AEB-decision-layer concept this project introduces, not something the predecessor's scope required. Built entirely fresh — `is_velocity_eligible` (step05 Part B) and `ttc_from_state` (step07 Part A) are reused unchanged, exactly as the spec requires ("Calls ttc_from_state from Part A unchanged").

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
78 passed, 14 warnings in 43.60s
```
All 6 of this step's tests pass, alongside the 72 from prior steps. All passed on the first implementation.

Test 4 (causality) is worth spelling out precisely: it doesn't just check that *some* ego state was passed — it constructs two different ego states (one correctly at the snapshot's own timestamp, one from a materially later, faster, closer ego) and confirms they produce *different* distance/closing-speed answers (30m/moderate-closing vs. 10m/faster-closing). This proves the causality obligation actually matters for the result, not just that the API accepts a timestamp-shaped argument.

## Real-data verification
Not applicable — step07 Part B's own spec lists only synthetic tests (§B4); real-data use arrives once the sensor adapters (steps 14-16) feed real tracks through this same estimator.

## Surprising or suspicious numbers (treat as bugs first)
None from the estimator logic itself. One non-issue investigated before being dismissed: the test file's first run took 20.66s (vs. ~1-2s for other synthetic-only test files) — re-ran immediately and it dropped to 1.43s, confirming this was one-time Python import/bytecode-cache overhead from newly-added modules, not a real performance problem in the estimator or its tests.

## Assumptions introduced (each must be in config with a status)
None new. This step consumes `MIN_UPDATES_FOR_TTC`, `MAX_VEL_STD_MPS`, `MIN_TRUSTED_SPEED_MPS`, `MIN_CLOSING_SPEED_MPS` — all already in `config.py`.

## Flagged, not built (out-of-scope temptations)
- `conservative_ttc()` is implemented (since DEC-10 explicitly wants it available as a diagnostic) but nothing consumes it yet — step08 will decide what, if anything, ever displays or logs it; it does not feed the action decision itself, per DEC-10.

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step08_debounce_action.md` (build order #11).
