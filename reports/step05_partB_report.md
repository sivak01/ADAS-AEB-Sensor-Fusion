# Step 05 Part B report

## What was built (files, one line each)
- `src/ttcf/tracking/relevance.py` — `path_relevance(snapshot, ego, cfg)`, `is_velocity_eligible()` (written once for reuse by step07 Part B), `Relevance` enum, `RelevanceResult`.
- `tests/test_relevance.py` — 6 tests (step05 §B4).

## Decisions taken (DEC ids + Siva's answers)
None — Part B has no "Decisions to ask" section (DEC-1/DEC-9 already settled at Part A).

## Precedent check (V1/v2)
Already confirmed at Part A: no corridor/forward-path code exists anywhere in V1/v2. That finding extends here — nothing to port for the motion-aware "entering" test either. Built entirely fresh.

## Design choices made explicit (not fully pinned down by the spec text)
- **Velocity std for eligibility**: taken as the *worse* of the two axis stds (`max(sqrt(P[2,2]), sqrt(P[3,3]))`), not a combined magnitude — a track isn't eligible unless *both* velocity components are individually well-determined. Written once in `is_velocity_eligible()` specifically so step07 Part B's `TTCEstimator` (which needs the identical check) imports this rather than re-deriving it.
- **"Approaching longitudinally"**: implemented as a separate, explicit condition (`rel_v_ego[0] < 0`) alongside — not instead of — extrapolating the *full* position (both x and y) through `corridor_contains`. This matters concretely: test 4 (stationary roadside object) has the ego closing the longitudinal gap by definition (any stationary object is being "approached" as the ego drives past), but the object's lateral position never moves — extrapolating only x would have wrongly let the longitudinal-closing condition alone produce `ENTERING`; extrapolating the full position correctly keeps it `OUT` because the object's y never crosses into the corridor band.
- **Reused `corridor_contains` for the extrapolated check** rather than writing a parallel "will it be in range" test — same "one function, two/three callers" principle already established at Part A (detections, GT footprints, and now track extrapolation all go through the identical membership function).
- **Rotating velocity vectors correctly**: velocities are free vectors, not points — rotating one through `global_to_ego` (built for points, with a translation baked in) would have been wrong. Added a small dedicated `_rotate_2d()` helper (rotation only, no translation) rather than misusing the point-transform machinery.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
72 passed, 14 warnings in 41.68s
```
All 6 of this step's tests pass, alongside the 66 from prior steps — all passed on the first implementation, no debugging narrative this time.

## Real-data verification
Not applicable — Part B operates on `TrackSnapshot`/`EgoState`, which only exist once the tracker (step06, synthetic-only) and real sensor adapters (steps 14–16) are running; this step's own tests are synthetic per its own spec.

## Surprising or suspicious numbers (treat as bugs first)
None. All 6 tests passed cleanly.

## Assumptions introduced (each must be in config with a status)
None new — this step only consumes existing `config.py` values (`CORRIDOR_HALF_WIDTH_M`, `CORRIDOR_MAX_RANGE_M`, `CANDIDATE_EXTRA_MARGIN_M`, `ENTERING_HORIZON_S`, `MIN_UPDATES_FOR_TTC`, `MAX_VEL_STD_MPS`).

## Flagged, not built (out-of-scope temptations)
- No lane maps or curved-path prediction — explicitly out of scope (B3), consistent with Part A's own documented straight-corridor limitation.
- No extrapolation beyond `ENTERING_HORIZON_S` — enforced structurally (the extrapolation is a single fixed-horizon step, not a search over multiple horizons).

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step07_ttc.md` **Part B** (build order #10).
