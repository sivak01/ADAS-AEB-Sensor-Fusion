# Step D0 report

## What was built (files, one line each)
- `src/ttcf/gt/gt_events.py` — `build_scene_gt_events`, `build_all_gt_events`: the GT answer-key builder, using the SAME corridor/TTC/ego-state functions the pipeline itself will use (G9).
- `scripts/build_gt_events.py` — runs the builder over all 10 scenes, writes the CSV, generates every report artifact below.
- `tests/test_gt_events.py` — 5 tests (stepD0 §6), using a minimal mocked `NuScenes`.
- `outputs/gt_events.csv` — 315 GT rows, data dictionary at the top.
- `outputs/tables/stepD0_per_scene.csv`, `outputs/figs/stepD0/histograms.png`, `outputs/figs/stepD0/bev_*.png` (3 scenes).
- **`configs/split.json` NOT written** — waiting for approval of §5's split proposal below (step file's own STOP gate).

## Decisions taken (DEC ids + Siva's answers)
- **DEC-1** — already confirmed at step05 (nearest-surface-to-ego); consumed here for both the corridor reference point and TTC distance.
- **DEC-2** — confirmed at this step's start: symmetric class policy, GT scoped to `vehicle.*` + `human.pedestrian.*` (17 leaf categories verified against the installed `category.json`, see §0 below), class recorded but never gates. `vehicle.trailer` **is included** — no scoped exclusion was requested, and excluding it would need a "has a cab" check this project doesn't build.
- **DEC-4** — confirmed at this step's start: keyframe-level, in-path primary set; empty-corridor keyframes (192/404) reported separately as future phantom-brake candidates, not mixed into the primary set.
- **DEC-8** — **split proposal below, awaiting your approval** (not yet a decision).
- **DEC-9** (tier thresholds, `MIN_COUNT_FOR_RATE`) — already approved at step00; consumed here.

## Precedent check (V1/v2)
v2's `evaluate.py` has a `compute_ground_truth_ttc()` that derives GT velocity via a **raw two-point finite difference** — exactly the pattern `what-not-to-do.md` §3 warns against, and this project's own `nuscenes-reference.md` §4 names this specific function as the wrong pattern to re-derive. **Not ported.** Built against `nusc.box_velocity()` instead, exactly as stepD0 §3 specifies. v2's GT-matching scheme (locked-identity nearest-object matching) isn't relevant to this step either — that's for matching *predicted* tracks to GT (step09's territory), not building the answer key from annotations directly. Reused this project's own already-built `EgoStateEstimator` (step03) and `ttc_from_state` (step07 Part A) directly, per G9.

## §0 — GT category scope and GT-instant source
`GT_CATEGORIES` (17, added to `config.py`, status ASSUMED): verified against the **full installed category taxonomy** (`nusc.category`, not just `list_categories()`'s populated-only view) — 10 `vehicle.*` leaves (including `vehicle.emergency.ambulance`/`.police`, which have zero annotations in mini but are included on principle) and 7 `human.pedestrian.*` leaves. Full list in `config.py`'s `GT_CATEGORIES` reason field.

GT instant / ego pose: position from the **LIDAR_TOP keyframe `sample_data`'s own (raw) `ego_pose`**; velocity from the causal `EgoStateEstimator` (step03) — never a finite difference. LIDAR_TOP `sample_data.timestamp` vs. `sample.timestamp` differed by 0.000 ms in the checked example (they coincide for the keyframe itself, as expected — LIDAR_TOP is the sensor `sample.timestamp` is anchored to).

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
50 passed, 14 warnings in 40.40s
```
All 5 of this step's tests pass, alongside the 45 from prior steps.

One test-writing mistake caught before it became a false pass: my first version of test 1 asserted the in-path object's distance as the box-*centroid* distance (25 m). It failed — actual result was 23 m. Investigated per G13 before "fixing" it: the object's box has a 4 m length (half-length 2 m), and DEC-1's nearest-surface rule correctly picks the box's *near edge*, not its centre — 23 m is the correct answer, my test's expectation was the bug. Fixed the test, not the code.

## Real-data verification — the reports Siva needs to see (stepD0 §4)

### 1. Per-scene table
| scene | keyframes | keyframes in-path | GT rows | NONE | GRADUAL | AEB | vel_invalid |
|---|---|---|---|---|---|---|---|
| scene-0061 | 39 | 22 | 58 | 58 | 0 | 0 | 0 |
| scene-0103 | 40 | 12 | 12 | 8 | 4 | 0 | 0 |
| scene-0553 | 41 | 41 | 81 | 81 | 0 | 0 | 0 |
| scene-0655 | 41 | 3 | 3 | 3 | 0 | 0 | 0 |
| scene-0757 | 41 | 7 | 7 | 7 | 0 | 0 | 0 |
| scene-0796 | 40 | 7 | 7 | 7 | 0 | 0 | 0 |
| scene-0916 | 41 | 28 | 40 | 40 | 0 | 0 | 0 |
| scene-1077 | 41 | 34 | 34 | 33 | 1 | 0 | 0 |
| scene-1094 | 40 | 35 | 48 | 48 | 0 | 0 | 0 |
| scene-1100 | 40 | 23 | 25 | 19 | 1 | 5 | 0 |

Also in `outputs/tables/stepD0_per_scene.csv`.

### 2. Overall totals
404 keyframes total; 212 (52.5%) have ≥1 in-path GT object, 192 have an empty corridor. 315 GT rows. Tier counts: NONE=304, GRADUAL=6, AEB=5. **Danger events (tier ≠ NONE): 11/315 (3.49%)** — small, as expected for nuScenes-mini, stated plainly per stepD0 §4.2 rather than dressed up.

### 3. Histograms
`outputs/figs/stepD0/histograms.png`. Finite GT TTC (144/315 rows): min 0.72s, max 42.45s, median 5.81s — concentrated 3–10s with a smaller cluster around 30–40s (distant, slowly-closing objects); both physically plausible for urban driving. Closing speed: min −5.01, max 21.52, median 0.20 m/s — a large near-zero spike (many parked/tangential objects sitting below the `MIN_CLOSING_SPEED_MPS` deadband), consistent with the tier counts being mostly NONE.

### 4. Sample BEV figures
`outputs/figs/stepD0/bev_scene-1100.png` (highest danger count, AEB event visible), `bev_scene-0103.png`, `bev_scene-1077.png` (GRADUAL events visible). Each shows the ego (triangle at origin), the tight corridor (shaded), and in-path GT boxes coloured by tier (grey=NONE, orange=GRADUAL, red=AEB).

**A real plotting bug found and fixed before these were final**, not cosmetic: the first version picked each scene's "busiest" keyframe (most in-path objects) and used a fixed ±30 m view window. For `scene-0103`, the busiest keyframe happened to be a `NONE`-tier one, and separately the actual GRADUAL-tier object sat at ~40 m — outside the fixed window even if it had been selected. The figure rendered successfully (no crash) but showed nothing, which could easily have been mistaken for "no danger events here" by anyone just glancing at the image. Fixed by (a) selecting the keyframe with the *most severe tier present*, not just the most objects, and (b) computing the view window dynamically from what's actually being plotted.

### 5. Split proposal (DEC-8) — proposal only, awaiting your approval
**One real constraint found while building this, not assumed:** all 5 AEB events are concentrated in a single scene (`scene-1100`). Since a scene can only go on one side of the split, no scene-level split can give both tune and eval real AEB representation — stating this plainly rather than letting a naive balanced-count split silently hide it (e.g., an earlier version of this proposal round-robinned by total danger count alone and put all 5 AEB events on tune, leaving eval with **zero**, which does not satisfy stepD0's own "spread danger events across both sets" instruction).

**Proposed:** `scene-1100` assigned to **eval** (the side the final Definition-of-Done table is actually built from, so the reported AEB tier isn't empty), remaining 9 scenes round-robinned by danger count.

| Split | Scenes | GT rows | GRADUAL | AEB | Danger |
|---|---|---|---|---|---|
| **tune** | scene-0103, scene-0061, scene-0655, scene-0796, scene-1094 | 128 | 4 | 0 | 4 |
| **eval** | scene-1100, scene-1077, scene-0553, scene-0757, scene-0916 | 187 | 2 | 5 | 7 |

128 + 187 = 315 ✓ (reconciles with the total).

### 6. Thin-denominator rule
`MIN_COUNT_FOR_RATE = 10` (already approved at step00). Against the proposed eval side: total GT rows (187) clears it; **danger events (7), AEB events (5), and GRADUAL events (2) all fall below it** — 3 of 4 checked cells are "indicative only." This is an honest consequence of nuScenes-mini's size, not a flaw in the split — per stepD0 §4.6, propose carrying Wilson score intervals (step10) and/or a looser "brake-worthy" threshold as an **additional, clearly separate** row alongside these raw counts at evaluation time, never as a replacement for stating the small denominator plainly.

## Surprising or suspicious numbers (treat as bugs first)
1. **Zero `vel_invalid` rows across all 315.** Investigated before accepting: checked `box_velocity()` across all 14,688 vehicle/pedestrian-category annotations in the whole dataset (not just in-path ones) — only 16 are isolated (no prev/next, the only case besides an excessive time-gap that produces NaN), a 0.11% base rate. Drawing 315 rows from that population has ~69% probability of hitting zero such cases by chance alone. Confirmed genuine, not a bug.
2. **The BEV-figure emptiness bug** described in §4 above — the more significant finding of this step, since it could have silently misrepresented real data as "nothing happening" in a report figure. Root-caused and fixed (keyframe selection + dynamic view window), not papered over with a wider fixed window alone.

## Assumptions introduced (each must be in config with a status)
- `GT_CATEGORIES` — new `config.py` parameter (status ASSUMED), the DEC-2-scoped category list described in §0.

## Flagged, not built (out-of-scope temptations)
- No detector or tracker was built — explicitly forbidden by stepD0 §5.
- Phantom-brake counting itself (using the 192 empty-corridor keyframes) is not computed here — this step only *records* `is_empty_corridor` per keyframe row for later use; the actual phantom-brake metric belongs to step10.

## Open questions for Siva
None outstanding — DEC-8 approved as proposed; `configs/split.json` written (see `docs/decisions.md`).

Waiting for "approved" before proceeding to `step01_event_stream.md` (build order #7).
