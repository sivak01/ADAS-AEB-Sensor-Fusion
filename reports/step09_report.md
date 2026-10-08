# Step 09 report

## What was built (files, one line each)
- `src/ttcf/evaluation/extrapolate.py` — `extrapolate_track`, `system_view_at`, `associate_tracks_to_gt`, `SystemView`/`TrackExtrapolation`.
- `scripts/run_pipeline.py` — synthetic end-to-end runner, writes the pipeline log schema (`track_log.csv`, `action_log.csv`, `association_log.csv`) under `outputs/runs/<run_name>/`.
- `tests/test_extrapolate.py` — 10 tests (8 from step09 §5 + 2 extra covering `associate_tracks_to_gt`'s empty-input cases and `system_view_at`'s multi-track integration).

## Decisions taken (DEC ids + Siva's answers)
None — step09 has no "Decisions to ask" section.

## Precedent check (V1/v2), ported vs. rebuilt, with why
v2's `evaluate.py` has `position_at()` and `match_track_to_ground_truth()`, directly relevant precedent — and step09's own purpose statement names this exact file's history: *"Two bugs found in V2 live here: extrapolating without a bound, and extrapolating backwards before a track existed."* Reading v2's actual (already-fixed) code confirmed both guards already exist there, with the same reasoning this step asks for:
- **Ported directly**: the two bounding guards — an annotation before a track's first snapshot is a clean no-match (`NO_TRACK`), never a backward extrapolation; one past the eviction cutoff is `STALE`, since the track would already be evicted by then. Same reasoning, using this project's own (much shorter) `EVICTION_GAP_S` in place of v2's `MAX_MISSED_SECONDS`.
- **Deliberately not ported**: v2's `position_at()` is plain arithmetic (`pos + v*dt`) with **no covariance growth** — its own docstring explains this was a deliberate simplification, since v2 only ever compared extrapolated position for MAE/RMSE and never needed P at the extrapolated instant. This project's eligibility check (`is_velocity_eligible`, step05 Part B) depends on P, and a track extrapolated a long time without a real update *should* become progressively less trusted — so `extrapolate_track` reconstructs a throwaway `ConstantVelocityKF` from the snapshot and calls its own, already-tested `state_at()` instead of reimplementing the arithmetic by hand. This reuses one implementation of "predict forward" (step03) rather than adding a second, and gets correct P growth for free.
- **Also deliberately not ported**: v2's association is a **locked identity** — once a track matches a GT instance, it keeps using that same instance for its whole life, mirroring the old late-fusion pipeline's design. This is explicitly MOT-style. step09 §3.3 wants a *fresh* per-instant Hungarian match every time, consistent with this project's no-persistent-identity philosophy (G11) — built fresh.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
98 passed, 14 warnings in 44.74s
```
All 10 of this step's tests pass, alongside the 88 from prior steps — all passed on the first implementation.

## Real-data verification
Not applicable in the sense of real sensor data (no adapters exist yet — steps 14-16). However, `scripts/run_pipeline.py` was run end-to-end on a synthetic closing scenario to confirm the full log schema materializes correctly and stays internally consistent with step06/07/08's already-verified behavior:
- Early rows (few updates) correctly show `ttc_reason=NOT_ELIGIBLE`, `is_critical_object=False` (an ineligible track can never be selected as critical — step07 Part B's own rule, `critical_object()` requires a finite TTC).
- Once eligible, distance shrinks smoothly (~27m → ~21m) as the synthetic ego closes at 10 m/s on a stationary object at (30,0) — matches the scenario's own physics.
- The action log's debounce timeline matches step08's already-tested behavior exactly: `NONE` with a growing `consecutive_count` (0,1,2) as TTC first crosses below `TTC_GRADUAL_S`, firing `GRADUAL` at exactly the 3rd consecutive qualifying update.

## Surprising or suspicious numbers (treat as bugs first)
None — this step's tests and the pipeline-log demo all matched expectations on the first implementation, consistent with the already-verified behavior of the steps it composes (06, 07, 08).

## Assumptions introduced (each must be in config with a status)
None new. This step consumes `EVICTION_GAP_S`, `MAX_ASSOC_DIST_M`, `SIGMA_A` — all already in `config.py`.

## A design choice made explicit (not fully pinned down by the spec text)
`SystemView`'s top-level `no_estimate_reason` (when no critical object is found) needed a priority order across the four possible values (`NO_TRACK`/`STALE`/`NOT_ELIGIBLE`/`NONE_IN_PATH`) the spec names but doesn't rank. Implemented as: all tracks `NO_TRACK` → `NO_TRACK`; all `NO_TRACK`/`STALE` → `STALE`; all of those plus `NOT_ELIGIBLE` → `NOT_ELIGIBLE`; anything else (some track has a usable, eligible estimate but none are path-relevant) → `NONE_IN_PATH`. Documented directly in the code (`_derive_top_level_reason`) so the reasoning is visible, not just the conclusion.

## Flagged, not built (out-of-scope temptations)
- `scripts/run_pipeline.py` deliberately stays minimal — a synthetic-only demonstration of the log schema, not a general-purpose runner. It will need real extension once sensor adapters (steps 14-16) exist; not built ahead of that need.
- No parquet output — CSV only, consistent with this project's existing pattern (`stepD0`'s `gt_events.csv`) and simpler to inspect by hand; the spec's "parquet/CSV" wording explicitly allows this.

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step10_metrics.md` (build order #13).
