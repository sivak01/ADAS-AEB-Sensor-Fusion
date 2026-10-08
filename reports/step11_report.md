# Step 11 report — Four runs, tuning, freeze, final evaluation

This is the step's own build report. The two required deep-dive documents
this step produces — **`reports/final_report.md`** (the four-run result,
investigation, limitations, config table) and **`docs/knowledge_share.md`**
(the portfolio-ready teaching write-up) — carry the actual findings in
full; this report summarizes what was built and how, without duplicating
them.

## What was built
- `scripts/run_pipeline.py` generalized: `--stream {lidar,radar,camera,fused}`
  (a new per-channel adapter dispatch table, so "fused" routes each event
  to its own real adapter while all writing into one shared tracker —
  early fusion, G10) and `--split {tune,eval} --final` (wired to
  `config.current_config_hash()` against `configs/frozen_config.json` via
  `guard_eval_scoring`, reused rather than re-implemented).
- `scripts/tune.py` — the SIGMA_A grid sweep (all 4 streams, tune split,
  DEC-7 criterion) and direct measurement of the three remaining
  PLACEHOLDER thresholds. Caches each stream's own adapter output once
  per scene and reuses it across every SIGMA_A grid point (adapter
  detections don't depend on SIGMA_A, only the tracker's process noise
  does), avoiding redundant expensive LiDAR RANSAC/camera-geometry calls.
- `scripts/generate_final_table.py` — produces the official
  `outputs/tables/four_run_table.*` from the accepted eval-split `--final`
  computation.
- `tests/test_run_equivalence.py` (6 tests) — run-equivalence (shared
  config hash unaffected by per-sensor params, changed by truly shared
  ones), the fused stream is exactly the union of the three single-sensor
  channel sets, the frozen-config hash matches `freeze()`'s own output,
  the four-run table generator's positive path (step10's own tests only
  covered its two failure paths), and full-pipeline determinism (two
  identical real runs give identical tracker counts and confusion
  matrices).
- `config.py`: `shared_config_hash()` and `current_config_hash()` added.
  Every remaining PLACEHOLDER resolved: `SIGMA_A=0.5`, `MAX_VEL_STD_MPS=3.90`,
  `MIN_TRUSTED_SPEED_MPS=1.99`, `MIN_CLOSING_SPEED_MPS=1.79` — full
  reasoning (including three G13 bugs found and fixed in the measurement
  methodology itself) in each param's own reason field and in
  `docs/decisions.md`.
- `configs/frozen_config.json` — SHA-256
  `58709abded7796ead9a7b086f82484e02ef7527aae165a6e1fae7e6fe96926c5`,
  approved by Siva before the eval split was touched.

## Decisions
- **DEC-7** (tuning selection criterion) — pre-declared *before* the
  sweep ran: mean `correct_action_rate` across all four streams,
  `missed_brake_rate` as tie-breaker (not needed in practice — SIGMA_A=0.5
  won outright).
- **Freeze approved** — Siva reviewed the tuned config and gave explicit
  approval before the eval split was scored.

## Precedent check
No V1/V2 precedent exists for a four-way single-vs-fused comparison, a
pre-declared tuning criterion, or a code-enforced eval-split guard — all
built fresh, consistent with what-not-to-do.md §7's own tuning-honesty
rules and this project's step10 guard mechanism (reused, not duplicated).

## Tests
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
139 passed
```
133 prior + 6 this step's own. Two pre-existing tests
(`test_ttc_estimator.py::test_wide_velocity_covariance_is_not_eligible`,
`test_relevance.py::test_ineligible_track_is_provisional_never_entering`)
needed a fixture update after `MAX_VEL_STD_MPS` moved from its old
placeholder (1.5) to the newly-measured 3.90 — their own "loose velocity
covariance" test fixture no longer exceeded the new, larger threshold.
Not a code bug: the test's *intent* (a wide velocity covariance should be
ineligible) was still completely valid; only the specific magic number
needed to still exceed the new, correctly-measured bound.

## G13 investigations (full detail in `docs/decisions.md` and `reports/final_report.md`)
1. **Three real bugs found and fixed in the tuning-threshold measurement
   methodology itself**, before any of the three values were trusted:
   under-converged-track contamination, a crude diagnostic association
   picking up unrelated nearby tracks in crowded scenes, and a closing-
   speed formula conflating real ego motion with sensor jitter. All three
   documented in full in `config.py`'s own reason fields.
2. **Fused's elevated eval-split false-brake rate**, investigated and
   traced: half the instances came from one scene that is a textbook
   crowd-conflation scenario (a busy parking lot), an already-documented,
   already-accepted limitation, not a new defect.
3. **A process error**: the eval-split camera cache was never built
   (only tune-split caching had been run), causing the first `--final`
   attempt for camera/fused to crash before any real scoring happened.
   Confirmed via the eval-split guard's own log that no score was ever
   produced the first time; fixed by building the missing cache and
   re-attempting only the two affected streams, recorded plainly per
   step11 §5.4's own spirit even though no completed score was actually
   redone.

## STOP
Final report and knowledge-share document delivered
(`reports/final_report.md`, `docs/knowledge_share.md`). Waiting for
Siva's review, per step11 §11.
