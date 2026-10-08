# STEP 11 — Four runs, tuning, freeze, ONE final evaluation, write-up

**Build order #17.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 00–10 and 04a/b/c approved.

## 1. Purpose (plain language)
Run the *identical* pipeline four times — LiDAR-only, radar-only, camera-only, fused — tune shared settings on the tuning scenes, freeze everything, then score the held-out scenes **once**. Fusion's justification is a claim to test, not assume (G13; lessons A.11). If fusion does not beat the best single sensor, that is a valid, reportable result.

## 2. Deliverables
- `scripts/run_pipeline.py` generalised: `--stream {lidar,radar,camera,fused} --split {tune,eval}`
- `scripts/tune.py`, `outputs/tuning_log.csv`
- `configs/frozen_config.json` (with SHA-256)
- `outputs/tables/four_run_table.*` (step 10 generator)
- `reports/final_report.md`
- `docs/knowledge_share.md` (portfolio-ready write-up)
- `tests/test_run_equivalence.py`
- report + learning note

## 3. Stream switch — same code path
The four runs differ **only** by which adapters/channels are enabled. Everything else — event stream, transforms, tracker, gate, TTC, debounce, extrapolation, metrics, config — is the same code and the same config values (except unavoidable per-sensor adapter parameters and `R`). Test: hash the config excluding `enabled_streams` and per-sensor adapter blocks; assert equal across the four runs. Also assert the fused run enables exactly the union of the three single-sensor adapters.

## 4. Tuning protocol (tune split only)
1. **What may be tuned** (all shared across the four runs unless noted): `SIGMA_A`, `DEBOUNCE_N`, `TTC_GRADUAL_S`, `TTC_AEB_S`, corridor margins, `MIN_TRUSTED_SPEED_MPS`, `MIN_CLOSING_SPEED_MPS`, `MAX_VEL_STD_MPS`, `EVICTION_GAP_S`; per-sensor: adapter clustering params and `R` (only if MEASURED per its step's rule).
2. **Pre-declare the selection criterion before running the sweep** (DEC-7): record it in `docs/decisions.md`. Recommended: the mean of the correct-action rate across the four streams (with missed-brake rate as a tie-breaker), so that selection is not biased toward fusion.
3. **`SIGMA_A` re-measurement (do not inherit V1's number — what-not-to-do §4).** Sweep a log-spaced grid. For each stream and value report: gate acceptance rate, spawn-to-match ratio (fragmentation), the velocity jitter on stationary GT objects, and the event-level metrics on the tuning split. Show the interaction with the ~75× `R` range across sensors (tight-R sensors under-gate vs loose-R sensors over-merge). Choose one shared value by the pre-declared criterion.
4. Log **every** run in `outputs/tuning_log.csv` (params, stream, split, metrics, counts) — including bad ones.
5. Coarse grids or one-at-a-time sweeps are fine; there are only a handful of tuning scenes, and the log must say how few danger events the tuning set contains.
6. Replace every `PLACEHOLDER` in config with a measured value (with evidence) or an explicitly justified ASSUMED value. Update statuses honestly.

## 5. Freeze and final evaluation
1. Write `configs/frozen_config.json` + hash. Siva reviews and says "freeze approved".
2. Run the four streams on the **eval split with `--final`**, exactly once. The split guard (step 10) logs it.
3. Generate `four_run_table.*` with raw counts, Wilson intervals, both observability variants, latency, `no_estimate_reason` breakdown, false-brake attribution, and the mandatory footnotes.
4. **If the result surprises you** (e.g., camera beating fused, or a row all zeros), investigate as a likely bug: check frames, timestamps, gate, denominators, observability flags. Report the investigation. **Do not re-run the eval split** to fix it; if a genuine bug is found, log it, fix it, and record that the eval split was touched more than once and why. Never quietly re-tune.

## 6. Final report (`reports/final_report.md`) must contain
- The four-row table (lidar / radar / camera / fused) — never fused alone.
- Statement of whether fusion beat the best single sensor, in plain words, with counts and intervals.
- Named limitations: crowd conflation (same-class objects close together, e.g., pedestrians at a crosswalk) — safety-relevant and open (lessons C.3); curved-path corridor; flat-ground camera depth; radar Doppler disabled; ghost radar returns; tiny number of danger events in nuScenes-mini; ego-velocity KF lag.
- Table of config parameters with MEASURED/ASSUMED/PLACEHOLDER status.
- "Flagged, not built": Doppler-in-KF ablation, camera-class gating ablation, appearance re-ID, learned depth, steering scoring, curved-path prediction.
- Reproduction instructions (exact commands, config hash, dataset path variable).

## 7. `docs/knowledge_share.md` (for Siva to teach from)
Portfolio-ready, plain language, ≤ ~6 pages: mission and rescope from MOT; the architecture diagram; the ten most important design decisions and the mistake each prevents; the four-run result and how to read it honestly; what was measured vs assumed; limitations; what you'd do next. Include the worked numeric examples from the learning notes (S = P + R; ego-compensation; closing-speed deadband; debounce latency).

## 8. Do NOT
- Do not tune on eval scenes, or re-run the eval split to improve numbers.
- Do not tune any parameter specifically for fusion.
- Do not report fused in isolation, or rates without counts.
- Do not drop the mandatory footnotes.
- Do not declare an `R` measured without evidence.
- Do not add MOT metrics or long-memory features "for robustness".

## 9. Tests
1. Run-equivalence test (§3).
2. Frozen-config hash matches at final-run time.
3. Eval-split guard blocks non-`--final` scoring.
4. Table generator produces four rows and all footnotes.
5. Re-running a tune-split run with the same config reproduces identical counts (determinism).

## 10. Learning-note topics
Why single-sensor baselines are the control experiment; how the tuning/eval split protects the claim; how to read a table with tiny counts; what an honest "fusion did not win" looks like.

## 11. STOP
Deliver the final report and knowledge-share document. Wait for Siva's review.
