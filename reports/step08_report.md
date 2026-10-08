# Step 08 report

## What was built (files, one line each)
- `src/ttcf/ttc/action.py` — `ActionTierDebouncer` (`process()`, `action_at()`), `ActionRecord`, `LatencyRecord`.
- `tests/test_action.py` — 10 tests (9 from step08 §5 + the required timeline figure).
- `outputs/figs/step08/timeline.png` — TTC trace + debounced action decision.

## Decisions taken (DEC ids + Siva's answers)
None new — DEC-9 (`DEBOUNCE_N=3`, `TTC_GRADUAL_S=2.5`, `TTC_AEB_S=1.2`) already approved at step00; DEC-10 (point estimate drives action) already confirmed at step07 Part B and directly determines this step's design (the debouncer consumes `TTCResult.ttc_s`, the point estimate, not the conservative bound).

## Precedent check (V1/v2)
Searched v2 for any debounce/action-tier logic — found nothing (both grep matches were unrelated uses of the word "consecutive" describing ego-pose ordering). Neither V1 nor v2 ever built an actual decision/action layer — both only tracked and evaluated positions, never made a brake/warn decision. Built entirely fresh.

## A design choice made explicit (not fully pinned down by the spec text)
`ActionDecision` (the step00-approved shared type) has no field distinguishing "confidently safe" from "no usable estimate right now" — but step08 §3 explicitly requires that distinction ("NOT_ELIGIBLE results... are recorded as NO_ESTIMATE," "so evaluation can attribute later misses to coverage rather than to logic"). Rather than modify the already-approved step00 contract, this step returns its own `ActionRecord` — a strict superset of `ActionDecision` (same fields, plus `no_estimate: bool`) — composition instead of redefining a shared type other steps may already depend on.

`consecutive_count`'s exact meaning also wasn't fully pinned down by the spec (only tested at the moment a tier fires, where it must equal `DEBOUNCE_N`). Implemented as the current *in-progress* danger streak (whichever tier's condition currently holds), visible incrementally (1, 2, ..., N) rather than staying 0 until the exact instant of firing — this was in fact caught as a real bug during testing (see below), not simply assumed correct from the start.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
88 passed, 14 warnings in 42.37s
```
All 10 of this step's tests pass, alongside the 78 from prior steps.

**Two real logic bugs found and fixed, not test-only mistakes:**
1. `consecutive_count` was implemented as "the streak of the *already-fired* tier" — meaning it silently reported 0 for every in-progress update right up until the exact instant a tier fired, even though a real streak was accumulating. Caught by the eviction-recovery test (§5 item 7's follow-on check): after eviction, the very next dangerous update should show `consecutive_count == 1`, and it reported 0 instead. Fixed by reporting the in-progress streak (max across tiers) regardless of whether a tier has fired yet, not just an already-active tier's count.
2. The latency test's first version assumed exactly one `LatencyRecord` would be produced when a debounce sequence fires — it failed with 2. Investigated before "fixing" the test blindly: the synthetic scenario used a TTC value (`AEB_S - 0.1`) that satisfies *both* the AEB and GRADUAL gates simultaneously, so both tiers legitimately reach `DEBOUNCE_N` on the same update and both correctly get a latency record — genuine, correct behaviour the test's own assumption hadn't accounted for. Fixed the test's expectation, not the code.

**One test-scenario bug in the figure-generation test**: the first synthetic V-shaped TTC profile spent only 2 consecutive updates below `TTC_AEB_S` — one short of `DEBOUNCE_N=3` — so AEB never fired at all in the demonstration figure. Fixed by widening the dip (adding an explicit hold phase at the bottom) rather than lowering `DEBOUNCE_N` to make the test pass.

## Real-data verification
Not applicable — step08 is explicitly synthetic-streams-only (header: "Scope: synthetic streams only").

## Surprising or suspicious numbers (treat as bugs first)
Both action.py bugs above were caught by tests failing as designed, investigated per G13 before either the code or the test was changed — in each case confirming which one was actually wrong before touching anything.

## Assumptions introduced (each must be in config with a status)
None new. This step consumes `DEBOUNCE_N`, `TTC_GRADUAL_S`, `TTC_AEB_S`, `EVICTION_GAP_S` — all already in `config.py`.

## Release rule, stated as the step file asks
Release is symmetric: a tier drops after `DEBOUNCE_N` consecutive real updates where its condition is false, mirroring exactly how it was triggered. Verified directly in `test_release_requires_n_consecutive_safe_updates` (the tier stays active through `N-1` safe updates and only releases on the `N`-th). Alternative designs considered but **not built**, per the step file's own instruction to propose without building: an asymmetric release (e.g., a shorter release debounce than trigger debounce, biasing toward staying cautious longer) — this would trade a longer "stuck in AEB" tail for faster re-arming, a real tradeoff worth revisiting only if step11's tuning shows the symmetric rule causing a problem.

## Flagged, not built (out-of-scope temptations)
- No steering logic — explicitly out of scope (§4: "no GT to score it against in v1").
- No asymmetric release variant — proposed above, not built.

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step09_extrapolation.md` (build order #12).
