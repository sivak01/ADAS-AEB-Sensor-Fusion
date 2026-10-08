# STEP 09 — Bounded, causal extrapolation to ground-truth timestamps

**Build order #12.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 06, 07, 08, D0 approved.

## 1. Purpose (plain language)
Ground truth exists only at 2 Hz keyframe instants; the pipeline updates at native rate. To compare them fairly we ask: "what did the *system* believe at exactly the GT instant?" — using only information available up to that instant, extrapolated a bounded distance. Two bugs found in V2 live here: extrapolating without a bound, and extrapolating backwards before a track existed.

## 2. Deliverables
- `src/ttcf/evaluation/extrapolate.py`
- `scripts/run_pipeline.py` writes a **pipeline log** this step consumes (define the log schema here; see §3.1)
- `tests/test_extrapolate.py`
- report + learning note

## 3. Specification

### 3.1 Pipeline log (schema — produced by the runner, consumed here)
A time-ordered record per real update: `t_us, track_id, TrackSnapshot fields, TTCResult, path relevance verdict, critical-object flag`, plus the action-decision timeline from step 08 and the tracker decision log. Store as parquet/CSV under `outputs/runs/<run_name>/`.

### 3.2 `system_view_at(log, ego, t_gt_us) -> SystemView`
For each track that exists at `t_gt`:
- Take the **last snapshot with `t ≤ t_gt`** (causal — never a snapshot from the future, G15).
- If `t_gt < track.first_t_us` → the track does not exist yet: **no extrapolation backward**; reason `NO_TRACK`.
- Δ = `t_gt − t_snapshot`. If `Δ > EVICTION_GAP_S` → reason `STALE` (do not stretch a dying track). Otherwise extrapolate position with the snapshot's own velocity: `pos + v·Δ`; recompute TTC with the ego state **at `t_gt`** using `ttc_from_state` (same function as GT).
- If the snapshot is `NOT_ELIGIBLE` → `NOT_ELIGIBLE` (visible, not "safe").
- The system's **action at `t_gt`** = `action_at(t_gt)` from step 08.
- Return per-track extrapolated state, TTC, path relevance at `t_gt`, the action, and a `no_estimate_reason` when nothing is usable (`NO_TRACK / STALE / NOT_ELIGIBLE / NONE_IN_PATH`).

### 3.3 Diagnostic association (for false-brake attribution only)
Match each system track to GT objects at `t_gt` by Hungarian assignment on distance between reference points (DEC-1 definition), threshold `MAX_ASSOC_DIST_M`. The primary metric (step 10) does **not** need this; it exists to say *which* GT object (or none — a ghost) a false brake was about.

## 4. Do NOT
- Do not compare a track's nearest update to GT (what-not-to-do §6).
- Do not extrapolate without bound, or backward before the first snapshot.
- Do not use any snapshot with timestamp > `t_gt`.
- Do not silently treat "no estimate" as "no danger" — always carry the reason.
- Do not use a different distance definition than D0 (G9).

## 5. Tests
1. Exact extrapolation: known-velocity track, GT time between updates → predicted position matches the analytic value.
2. `Δ > EVICTION_GAP_S` → `STALE` (regression for the unbounded-extrapolation bug).
3. `t_gt` before first snapshot → `NO_TRACK` (regression for the backward-extrapolation bug).
4. **No-lookahead:** results at `t_gt` are identical if all log entries after `t_gt` are deleted.
5. `NOT_ELIGIBLE` propagates.
6. Ego state used is at `t_gt`, not at the snapshot time.
7. Association threshold behaviour on toy cases.
8. Proportion check: with 2–4 updates of memory, mid-eviction/not-yet-started cases are relatively common — the tests must include them.

## 6. Learning-note topics
Why GT at 2 Hz vs tracker at ~20 Hz needs alignment; causality (what a real vehicle could know at that instant); why "no estimate" must be distinguished from "safe".

## 7. STOP
Report and wait for "approved".
