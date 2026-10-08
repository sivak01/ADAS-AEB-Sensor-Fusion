# STEP 10 — Event-level metrics and the four-run report

**Build order #13.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 08, 09, D0 approved.

## 1. Purpose (plain language)
Score the system the way an AEB engineer would: at each moment something real is in the path, did the system take the right action? No MOT metrics, no full-track MAE/RMSE (G11, G16, what-not-to-do §1). This step builds the scorer and the report generator, and enforces the "held-out data is touched once" rule.

## 2. Deliverables
- `src/ttcf/evaluation/metrics.py`, `src/ttcf/evaluation/report.py`
- `tests/test_metrics.py`
- guard for the eval split (see §3.5)
- report + learning note

## 3. Specification

### 3.1 Unit and labels (DEC-4)
- **Primary set E:** keyframes with ≥ 1 in-path GT object (from D0). GT action per keyframe = most severe tier among its in-path GT objects.
- System action per keyframe = `system_view_at(t_gt).action` (step 09).
- **Correct** = system tier equals GT tier.
- **Missed-brake** = GT ≠ NONE and system = NONE.
- **Under-brake** = GT tier > system tier > NONE (report separately).
- **False-brake** = GT = NONE and system ≠ NONE (within E).
- **Over-brake** = GT ≠ NONE and system tier > GT tier (report separately).
- **Phantom brake** = keyframe with **empty corridor** where system ≠ NONE. Reported separately, and also as a combined "false-brake incl. phantom" line, clearly labelled.

### 3.2 Rates — always with raw counts
- correct-action rate = correct / |E|
- false-brake rate = false-brakes / #keyframes in E whose GT = NONE
- missed-brake rate = missed / #keyframes in E whose GT ≠ NONE
- If a denominator is 0, report `n/a` — never 0 or NaN silently.
- Any rate with denominator `< MIN_COUNT_FOR_RATE` is labelled **"indicative only"**.
- Add **Wilson score 95% intervals** next to every rate.
- Also output the full 3×3 confusion matrix (GT tier × system tier) per run.

### 3.3 Observability variants (what-not-to-do §6)
For each single-sensor run, compute the metrics twice:
- **All GT events**, and
- **Observable-restricted:** recompute GT tiers using only GT objects that sensor could plausibly have seen (`obs_lidar / obs_radar / obs_camera` flags; for fused use `obs_any`).
Both variants appear in the table. A sensor's coverage gap must never read as a tracker failure.

### 3.4 Diagnostics (not headline metrics)
- Latency: time from GT danger onset (first keyframe with tier ≠ NONE for an object) to system action firing.
- `no_estimate_reason` breakdown for every missed-brake (`NO_TRACK/STALE/NOT_ELIGIBLE/NONE_IN_PATH`).
- False-brake attribution: for each false-brake, the category/attribute of the nearest GT object within `MAX_ASSOC_DIST_M`, or `ghost` if none (DEC-2 diagnostic).
- Fraction of fused estimates with ≥ 2 sensors in `sensors_in_estimate` (dilution check).

### 3.5 Split guard
- `metrics.py` refuses to score `eval` scenes unless invoked with `--final` **and** the config hash equals `configs/frozen_config.json`. Every eval-split invocation is appended to `outputs/eval_runs.log` (timestamp, config hash, run name). Scoring on the tune split is unrestricted.

### 3.6 Report generator (`report.py`)
Produces `outputs/tables/four_run_table.md/.csv/.json`: four rows (lidar / radar / camera / fused), every metric with raw counts, both observability variants. **The footnotes are generated from config and code and cannot be omitted; the generator must fail if any is missing:**
1. Radar-only ran with **Doppler disabled** → this reflects this configuration, not radar's real capability (what-not-to-do §7).
2. Which `R` values are MEASURED vs ASSUMED (from `config.describe()`).
3. Which rows fall under `MIN_COUNT_FOR_RATE` ("indicative only").
4. The class policy used (DEC-2).
5. The split used and that tuning was done on the tuning split only.
6. The distance definition (DEC-1).
7. Number of GT events and the statement that nuScenes-mini has few danger events.

## 4. Do NOT
- No IDF1/MOTA/ID-switch/MT-PT-ML, no full-track MAE/RMSE.
- Do not report fused numbers without the other three rows.
- Do not report rates without raw counts.
- Do not hide `n/a` or "indicative only" cells.
- Do not tune anything here to improve a row.

## 5. Tests
1. Hand-built toy logs with known GT/system tiers → exact expected counts and rates.
2. Zero-denominator cells → `n/a`.
3. Wilson interval matches a reference implementation on known inputs.
4. Observable-restricted variant removes exactly the unobservable GT objects.
5. Report generator raises if a footnote source is missing.
6. Split guard: scoring eval without `--final` fails; with a mismatched hash fails; every eval call is logged.
7. Phantom-brake counting on an empty-corridor toy keyframe.

## 6. Learning-note topics
Event-level vs trajectory metrics (why an AEB decision doesn't need a long-lived ID); what false vs missed brake mean in safety terms and why they trade off; why tiny denominators need counts and intervals; why the unobservable-GT variant exists.

## 7. STOP
Report and wait for "approved".
