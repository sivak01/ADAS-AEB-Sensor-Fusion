# Step 05 Part A report

## What was built (files, one line each)
- `src/ttcf/geometry/corridor.py` — `corridor_contains`, `candidate_gate`, `nearest_point_on_footprint`, `footprint_intersects_corridor`.
- `tests/test_corridor.py` — 5 tests (step05 §A4).

## Decisions taken (DEC ids + Siva's answers)
- **DEC-1** — Distance/reference-point definition: **B, nearest-surface-to-ego**, confirmed (README_MASTER.md §6 recommendation). Needed now because `footprint_intersects_corridor`'s exact membership rule depends on it. Recorded in `docs/decisions.md`; `config.DISTANCE_DEFINITION` updated from `PLACEHOLDER` to `ASSUMED` accordingly.
- **DEC-9** (corridor values) — already approved as part of the full step00 table; not re-asked, just consumed (`CORRIDOR_HALF_WIDTH_M=1.2`, `CORRIDOR_MAX_RANGE_M=60`, `CANDIDATE_EXTRA_MARGIN_M=1.5`).

## Precedent check (V1/v2)
Searched both V1 and v2 for any corridor/forward-path/lane-relevance code — **found nothing**. This is expected, not a gap in the search: V1/v2 were MOT-scoped and tracked every object in the scene, not just a forward corridor (`lessons-from-v1-v2.md`: *"this project has no reason to track something off to the side"*). Built entirely fresh; nothing to port or reject.

## Which precise rule was implemented for `footprint_intersects_corridor` (step05 §A2 asks this explicitly)
Per DEC-1 (nearest-surface-to-ego): the function computes the **exact nearest point on the GT footprint's boundary (or interior, if the ego origin falls inside it) to the ego origin** — not the footprint's centroid, and not a full polygon-vs-rectangle overlap test — then tests only that single nearest point against `corridor_contains`. Implemented as closest-point-on-a-convex-quadrilateral: closest point on each of the 4 edge segments to the origin, minimum taken across all 4 (with a point-in-polygon check first, for the degenerate case where the ego origin is inside the footprint). This means a large object whose nearest edge sits inside the corridor counts as intersecting even when most of its footprint (and its centroid) lies outside — verified directly in `test_footprint_intersects_corridor_per_nearest_point_rule`'s "straddling" case, where a box centred 1.5 m off the corridor's centreline (outside the 1.2 m half-width) still registers as intersecting because its near edge reaches to y≈0.55 m, inside the band.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
45 passed, 14 warnings in 39.19s
```
All 5 of this step's tests pass, alongside the 40 from prior steps.

## Real-data verification
Not applicable to Part A — step05 §A1 lists only synthetic tests as this part's deliverable. Real-data use arrives via stepD0's GT builder (which calls `footprint_intersects_corridor` on real annotation boxes) and step01/06 (which call `candidate_gate` on real detections), both later in the build order.

## Surprising or suspicious numbers (treat as bugs first)
None. All 5 tests passed on the first implementation; no debugging narrative for this step.

## Assumptions introduced (each must be in config with a status)
None new. `config.DISTANCE_DEFINITION` transitioned PLACEHOLDER → ASSUMED per DEC-1 above (already an existing parameter, not a new one).

## Flagged, not built (out-of-scope temptations)
- **Curved-path handling.** The corridor is straight in the ego frame; a turn or intersection is a known limitation per step05 §A2's own instruction ("do not build path prediction"). Not built, and not silently worked around — an object correctly in a vehicle's real curving path could be judged out-of-corridor by this straight-line rule, and vice versa. This is a named, accepted limitation, not a bug to fix later in this project's scope.
- Did not implement anything velocity-based ("about to enter" via motion) — explicitly Part B, needs the tracker (step06) first.

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `stepD0_gt_event_table.md` (build order #6).
