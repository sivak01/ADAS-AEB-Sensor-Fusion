# Step 06 report

## What was built (files, one line each)
- `src/ttcf/tracking/tracker.py` — `ShortMemoryTracker.process_scan()`, `AssociationLogEntry`, counters (`n_matches`, `n_spawns`, `n_evictions`, `multi_sensor_fraction`).
- `scripts/synthetic_scenarios.py` — reusable constant-velocity/alternating-sensor scan generators, for this step and later ones.
- `tests/test_tracker.py` — 10 tests (step06 §5).
- `outputs/figs/step06/test1_trajectory.png` — true vs. tracked trajectory for test 1.

## Decisions taken (DEC ids + Siva's answers)
- **DEC-5** — confirmed: `sensors_in_estimate` is the sensors of only the last `MAX_MEMORY_UPDATES=4` real updates (new `config.py` param, ASSUMED), a bounded recent-window set, not a lifetime-growing one. Eligibility (later, step07 Part B) uses `n_updates >= MIN_UPDATES_FOR_TTC` and velocity std `<= MAX_VEL_STD_MPS`; eviction uses `EVICTION_GAP_S`; no hard lifetime cap on a track's age.

## Precedent check (V1/v2), ported vs. rebuilt, with why
- **Ported**: predict-then-gate ordering; eviction *before* gating (an already-expired track can never steal a scan's detections — v2's own reasoning, carried over exactly); innovation-covariance gating `S = P_pred + R` (via this project's own `ConstantVelocityKF.mahalanobis_sq`, built at step03); the early-fusion architecture itself (one tracker, fed identically regardless of which/how many sensors are active).
- **Deliberately not ported** (G11 / what-not-to-do.md §1): v2's `MAX_MISSED_SECONDS`-scale eviction (~1.5s; this project uses the much shorter `EVICTION_GAP_S=0.3s`); v2's scene-boundary reset as its own subsystem (a tracker instance here is simply given one scene's events at a time — the concern dissolves rather than needing a mechanism); v2's lifetime-growing `sensors_seen` set and unbounded `Track.history` — replaced by the `MAX_MEMORY_UPDATES`-bounded window (DEC-5).
- **Genuine complexity increase over v2**: v2's `process_event` always handled exactly one detection (trivial 1×N assignment). `process_scan` here handles a *list* of detections per scan, needing real N×M Hungarian assignment with an infeasible-cost sentinel for below-gate pairs (technique borrowed from v2's `gating.py`, applied to a genuinely harder problem than v2 ever solved).
- **A literal spec detail, implemented precisely**: an unmatched track is "untouched" this scan — its internal KF state is *not* mutated (v2 mutates every track's predict every event, matched or not). Mathematically equivalent for a linear CV-KF (`predict(a)` then `predict(b)` == `predict(a+b)`) and cheaper; the tracker still returns a same-instant, predicted-to-`t_us` snapshot for every live track via the non-mutating `state_at` peek.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
66 passed, 14 warnings in 41.85s
```
All 10 of this step's tests pass, alongside the 56 from prior steps.

**Test 2 (S = P + R regression)** — the worked numbers, generated from an actual 2-real-update track (not a hand-picked hypothetical): after two tight (σ=0.1) updates, `P_pred = 0.0493`. A subsequent σ=1.5 detection offset by 1.2m gives **d² (S=P+R) = 0.626** (well inside the gate, 9.21 → accepted) vs. **d² (P alone) = 29.195** (nearly 3× the gate → would be wrongly rejected) — the tracker (which always uses S=P+R) correctly matches it; the parametrised "wrong" comparison, computed alongside for contrast, demonstrably would not have.

**Test 4 (crowd conflation) — a richer finding than initially expected, worth reading in full.** The first version of this test (both objects present from scan 0, 0.8m apart) did **not** reproduce conflation — Hungarian's global-optimum assignment handled two simultaneously-spawned, cleanly-separated tracks correctly. Investigated *why* before accepting a "no conflation observed" result as the final answer, since the step file's own expectation was "likely one merged track": redesigned the scenario to match the actual failure mode lessons-from-v1-v2.md describes — object 1 gets an established track first, then object 2 debuts 0.8m away in a scan where object 1 has no competing detection (simulating a missed detection that instant). This reproduced something more interesting than simple merging: **track identity hijacking**. The original track (`trk_000000`, which started on object 1) absorbed object 2's debut detection, and by the end of the scenario sat 1.16m from object 1's true path but only 0.36m from object 2's — it had been captured by the wrong object. Track *count* stayed at 2 throughout, which means count alone would never have surfaced this — only checking each track's actual position against both true trajectories did. Documented as the real observed failure mode, per what-not-to-do.md §4 and lessons-from-v1-v2.md item C.3; **not fixed**, per the step file's explicit instruction.

## Real-data verification
Not applicable — step06 is explicitly synthetic-data-only (§header: "Scope of this step: synthetic data only. No real detections yet."). Real detections arrive via the sensor adapters (steps 14–16), which feed this same tracker unchanged.

## Surprising or suspicious numbers (treat as bugs first)
The crowd-conflation investigation above is the main finding this step surfaced — not a bug, but a real limitation whose *actual* failure signature (identity hijack, not just track-count reduction) took a redesigned test to properly demonstrate. Nothing else unexpected; the other 9 tests all passed on first implementation.

## Assumptions introduced (each must be in config with a status)
- `MAX_MEMORY_UPDATES` — new `config.py` parameter (ASSUMED, 4), per DEC-5.

## Flagged, not built (out-of-scope temptations)
- No fix or mitigation for crowd conflation — explicitly named as an accepted, open limitation (step06 §5.4: "do not 'fix' it").
- No `sigma_a` re-tuning to avoid all over-merging across a scene — explicitly deferred to step11 (step06 §4's own instruction, lessons-from-v1-v2.md item D).
- No appearance re-ID — explicitly out of scope (what-not-to-do.md §1).

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step05_forward_path_filter.md` **Part B** (build order #9).
