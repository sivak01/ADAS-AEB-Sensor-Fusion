# STEP 05 — Forward-path relevance filter

Built in **two parts** (see `README_MASTER.md` §7).
- **Part A (order #5):** pure corridor geometry + pre-tracker candidate gate. Needed early because the GT table uses the same corridor function.
- **Part B (order #9):** track-level relevance using velocity ("about to enter"). Needs the tracker.

Follow `README_MASTER.md` §4 and §8.

## 1. Purpose (plain language)
Only objects in, or clearly about to enter, the ego vehicle's path deserve tracking attention. But at detection time we only have a *position* (no velocity yet), so "about to enter" can only be approximated by a wider margin. Once a track has a velocity we can apply the stricter, motion-aware test. The same corridor definition must be used for detections, tracks, and ground truth so the comparison is fair (G9).

---

# PART A — Corridor geometry and candidate gate

## A1. Deliverables
`src/ttcf/geometry/corridor.py`, `tests/test_corridor.py`, report + learning note (Part A).

## A2. Specification
All computations are in the **ego frame at the detection's own time** (x forward, y left). Convert with the step-02 utility (G1, G2).

- `corridor_contains(ref_xy_ego, half_width, max_range) -> bool`: `0 < x <= max_range` and `|y| <= half_width`.
- `candidate_gate(det, ctx, cfg) -> bool`: `corridor_contains` with `half_width = CORRIDOR_HALF_WIDTH_M + CANDIDATE_EXTRA_MARGIN_M`. This is the generous, position-only pre-tracker filter.
- `footprint_intersects_corridor(box_corners_ego, ...)`: for ground-truth boxes (D0 uses this). Uses the same corridor parameters. If DEC-1 = nearest-surface, membership is decided on the nearest point of the footprint; state precisely which rule you implemented.
- The corridor is **straight in the ego frame**. Curved paths (turns, intersections) are a *known limitation* — record it in the report; do not build path prediction.
- Range, half-widths and margins come only from config.

## A3. Do NOT
- Do not use class to gate (DEC-2).
- Do not apply this to anything except position; velocity-based logic belongs to Part B.
- Do not write a second corridor implementation for GT (one function, two callers).

## A4. Tests
1. Points just inside/outside each edge (front, rear, left, right, max range).
2. Candidate gate is a strict superset of the tight corridor.
3. A point behind the ego is never in the corridor.
4. Rotating the ego frame does not change membership of the same physical point (transform round trip).
5. GT footprint partly overlapping the corridor edge is treated per the stated rule.

## A5. Decisions to ask
DEC-1 (if not yet answered), DEC-9 (corridor values).

## A6. Learning-note topics (Part A)
Why filter early (compute and false alarms); why a position-only filter must be generous; the curved-road limitation and why it is documented, not solved.

## A7. STOP after Part A. Wait for "approved".

---

# PART B — Track-level relevance

Prerequisite: step 06 tracker approved.

## B1. Deliverables
Add to `corridor.py` (or `tracking/relevance.py`): `path_relevance(snapshot, ego, cfg)`; tests; report + learning note (Part B).

## B2. Specification
Returns `IN_PATH`, `ENTERING`, or `OUT`:
- `IN_PATH` if the reference point (ego frame, at the snapshot time) is in the strict corridor and in front of the ego.
- `ENTERING` if it is currently outside but its **lateral position relative to the ego**, extrapolated by `ENTERING_HORIZON_S` using the track's *relative velocity* (object velocity minus ego velocity, rotated to the ego frame), falls inside the corridor **and** it is approaching longitudinally.
- Else `OUT`.
- Uses relative velocity (G8's ego term applies here too). Requires the track to be velocity-eligible; otherwise fall back to the candidate-gate result and mark the verdict `PROVISIONAL`.

## B3. Do NOT
- Do not extrapolate more than `ENTERING_HORIZON_S`.
- Do not use absolute (global) velocity for the lateral test.
- Do not add lane maps or path prediction (out of scope).

## B4. Tests
1. Object in the corridor moving with ego → `IN_PATH`.
2. Object laterally outside but crossing toward the corridor fast enough → `ENTERING`.
3. Object outside and moving away laterally → `OUT`.
4. Stationary roadside object (e.g., parked car beside the corridor) with ego passing → `OUT`.
5. Ego-frame rotation invariance.
6. Ineligible-velocity track → `PROVISIONAL`, never `ENTERING`.

## B5. Learning-note topics (Part B)
Relative vs absolute velocity for lateral motion; why "entering" needs a velocity and how that ties into the two-update minimum.

## B6. STOP after Part B. Wait for "approved".
