# STEP D0 — Ground-truth event table, split proposal, thin-denominator rule

**Build order #6.** Follow `README_MASTER.md` §4 and §8.
Prerequisites approved: step 00, 02, 03, 07 Part A, 05 Part A.

## 1. Purpose (plain language)
Before any detector or tracker exists, build the answer key: for every 2 Hz keyframe in every scene, which real objects are in the ego vehicle's path, what their true TTC is, and what the correct action would be. This fixes the metric's denominators *before any results can tempt us*, and lets us split scenes into tuning and evaluation sets fairly (what-not-to-do §7).

## 2. Deliverables
- `src/ttcf/gt/gt_events.py`, `scripts/build_gt_events.py`
- `outputs/gt_events.csv` (+ a short data dictionary at the top of the report)
- `configs/split.json` (only after Siva approves)
- `tests/test_gt_events.py`
- figures in `outputs/figs/stepD0/`
- report + learning note

## 3. Specification

**What a GT row is:** one *(keyframe, in-path GT object)* pair.

For each scene → each keyframe `sample`:
1. GT instant = the keyframe's `sample.timestamp` (annotations live at this instant). Ego pose for this instant: use the LIDAR_TOP keyframe `sample_data` record's own `ego_pose` (state in the report which you used and the ms difference between candidates).
2. For each `sample_annotation`:
   - Category filter: `vehicle.*` and `human.pedestrian.*` (DEC-2). **Verify the exact list against the installed `category.json` / `list_categories()`**, print the resulting list, and store it in config (not hard-coded in the script). Note in the report whether `vehicle.trailer` is included and why.
   - Get box centre, size, yaw; convert to the ego frame with the step-02 utility.
   - In-path decision via the **same `corridor_contains` / `footprint_intersects_corridor` as step 05** (one function, two callers).
   - Reference distance per **DEC-1**, identical to what the tracker will emit (G9).
   - GT velocity: `nusc.box_velocity(ann_token)` (verify signature from step 00 report). It may return NaN when neighbours are missing — record `vel_valid` and **exclude those rows from TTC-based labels but count them** in the report.
   - Ego velocity at that instant from the step-03 `EgoStateEstimator` (same estimator as the pipeline, causal).
   - GT TTC = `ttc_from_state` from step 07 Part A (same function as the pipeline; different inputs).
   - GT action label from GT TTC using `TTC_GRADUAL_S` and `TTC_AEB_S`: `NONE / GRADUAL / AEB`.
3. Columns to store: scene, sample token, `t_us`, instance token, category, attributes (moving/parked/...), visibility level, `num_lidar_pts`, `num_radar_pts`, distance, closing speed, GT TTC, GT action, `vel_valid`, and per-sensor **observability flags**: `obs_lidar = num_lidar_pts >= MIN_LIDAR_PTS`, `obs_radar = num_radar_pts >= MIN_RADAR_PTS`, `obs_camera = visibility >= MIN_VISIBILITY_LEVEL`, `obs_any`.
4. **Keyframe-level table** (DEC-4): for each keyframe with ≥1 in-path GT object, the keyframe's GT action = the most severe tier among its in-path objects. Also record keyframes with an **empty corridor** (needed for phantom-brake counting later).

## 4. Reports Siva needs to see (the point of this step)
1. Per-scene table: #keyframes, #keyframes with in-path objects, #GT rows, and counts of GT action tiers (NONE/GRADUAL/AEB), plus #`vel_valid=False`.
2. Overall totals and how many events are **danger** events (tier ≠ NONE). Expect this to be small in nuScenes-mini — say so plainly.
3. Histogram of GT TTC (finite values) and of closing speed; check they look physical.
4. Sample BEV figures (ego frame) with corridor drawn and GT boxes coloured by tier, for ≥ 3 scenes including one with the highest danger count.
5. **Split proposal (DEC-8):** propose a tuning/eval scene assignment that spreads danger events across both sets, showing the counts on each side. Do not write `split.json` until Siva approves. Once committed it is fixed.
6. **Thin-denominator rule:** state the count below which a rate is labelled "indicative only" (`MIN_COUNT_FOR_RATE`, DEC-9) and how many of the eventual table's cells will fall below it *given the counts on the eval side*. Propose adding Wilson score intervals (step 10) and any looser "brake-worthy" evaluation threshold as an **additional, clearly separate** row — never as a replacement.

## 5. Do NOT
- Do not tune thresholds using anything but the tuning split; here just count.
- Do not use different distance definitions for GT and pipeline (what-not-to-do §5).
- Do not hard-code the category list from memory.
- Do not silently drop rows with invalid velocity.
- Do not build any detector or tracker.

## 6. Tests
1. A known synthetic annotation set (mocked) → expected in-path membership, TTC, and tier.
2. GT TTC for a parked GT object with moving ego equals distance/ego-speed (± tolerance).
3. In-path membership is identical when computed via the step-05 function vs a fresh call (regression: one implementation).
4. `vel_valid=False` rows appear in the counts and are excluded from tier labels.
5. Row counts reconcile: sum over scenes = total.

## 7. Decisions to ask
DEC-1 (must be settled now), DEC-2, DEC-4, DEC-8, DEC-9 (tier thresholds; `MIN_COUNT_FOR_RATE`).

## 8. Learning-note topics
Why the answer key is built first; what "event" means here; why nuScenes-mini has few danger events and what that does to the statistics; what observability flags are for.

## 9. STOP
Report with the counts tables and figures, wait for "approved". Only then write `configs/split.json`.
