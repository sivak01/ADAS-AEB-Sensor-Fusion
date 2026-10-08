# Step 10 report

## What was built (files, one line each)
- `src/ttcf/evaluation/metrics.py` — `score_run`, `keyframe_gt_tier`, `wilson_score_interval`, `guard_eval_scoring`, `KeyframeGT`/`KeyframeSystem`/`RunMetrics`/`RateResult`.
- `src/ttcf/evaluation/report.py` — `build_footnotes`, `generate_four_run_table`.
- `tests/test_metrics.py` — 12 tests (7 from step10 §5 + 5 extra covering report-generator/split-guard edge cases).

## Decisions taken (DEC ids + Siva's answers)
None — step10 has no "Decisions to ask" section. DEC-1, DEC-2, DEC-4, DEC-9 (`MIN_COUNT_FOR_RATE`) already settled and consumed here as specified.

## Precedent check (V1/v2)
v2's `evaluate.py` (`compute_sensor_metrics`, `compute_length_bucket_metrics`, `compute_multi_sensor_composition`) is entirely full-track MAE/RMSE and lifetime-match-rate based — exactly what what-not-to-do.md §1 and this step's own §4 forbid ("No IDF1/MOTA/ID-switch/MT-PT-ML, no full-track MAE/RMSE"). No event-level scoring, confusion matrix, or Wilson-score-interval precedent exists anywhere in V1/v2. This entire step is built fresh. The one thing worth carrying over in *spirit*, not code: v2 always reports a raw count (`n_matched_pairs`) alongside its MAE rate — this project's own "always report `n/a` over a hidden 0, always show raw counts" rule (§3.2) is the same honesty principle, taken further (Wilson intervals on top of raw counts, not just the counts).

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
110 passed, 14 warnings in 43.66s
```
All 12 of this step's tests pass, alongside the 98 from prior steps — all passed on the first implementation, no debugging narrative this time.

Wilson interval implementation (test 3) checked against **published reference values**, not just internal consistency: 50/100 → [0.404, 0.596], 0/10 → [0.0, 0.2775] — both standard textbook examples, matched to 3 decimal places.

## Real-data verification
Not applicable — no real per-run results exist yet (adapters arrive at steps 14-16; the actual four-way comparison is step11's job). Ran `generate_four_run_table` end-to-end with dummy 4-run data to confirm the file-writing pipeline (`.md`/`.csv`/`.json`) works correctly, then **deleted the dummy output** from `outputs/tables/` afterward so it can't be mistaken for step11's real results later.

## Surprising or suspicious numbers (treat as bugs first)
One near-miss, resolved before it became confusing: the demo report's Markdown file displayed as `wo§7` → `Â§7` when viewed through a PowerShell console `Get-Content` call. Checked the actual file bytes directly (via the file-reading tool, not the terminal) before assuming a real encoding bug — confirmed the file itself is correctly UTF-8 (`§` intact); the corruption was only in how PowerShell's console rendered it. Not a code issue.

## Assumptions introduced (each must be in config with a status)
None new. This step consumes `MIN_COUNT_FOR_RATE`, and (at report-generation time) whatever `config.describe()` reports for each sensor's `R` status.

## Design choices made explicit (not fully pinned down by the spec text)
- **`false_brake_incl_phantom_rate`'s exact formula** wasn't given as a literal equation in the spec (§3.1 only says to report it "as a combined line"). Implemented as `(false-brakes + phantom-brakes) / (GT=NONE keyframes in E + empty-corridor keyframes)` — combining both the numerator and denominator of the two underlying rates, verified directly in test 7.
- **Split-guard logging**: "every eval-split invocation is appended" is implemented to log **both** successful and refused attempts (written before the exception is raised on failure), so the audit trail captures denied attempts too, not just approved ones — confirmed by test 6 checking `REFUSED` appears in the log.
- **`configs/frozen_config.json` doesn't exist yet** (it's a step11 deliverable) — `guard_eval_scoring` handles this gracefully today (treats a missing frozen-config file as "no valid hash to match," always refusing eval scoring until step11 actually freezes one), rather than crashing on a missing file.

## Flagged, not built (out-of-scope temptations)
- No actual per-run scoring was performed — there is no real pipeline output to score yet. This step only builds and tests the scoring/reporting *machinery*; step11 is where it gets pointed at real four-way results.
- No tuning logic anywhere in this step, per §4's explicit rule ("do not tune anything here to improve a row").

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step04a_lidar_adapter.md` (build order #14) — the first step that touches real sensor data.
