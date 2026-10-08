# STEP 12 — BEV visualization (static keyframe PNGs, then per-scene video)

**Build order #18 — new scope, post step11** (not in README_MASTER.md §7's
original 17-step table). Prerequisites: steps 00–11 approved; the post-step11
G17 Doppler ablation / NIS check are done and recorded in `docs/decisions.md`.

## 1. Purpose (plain language)
A bird's-eye-view (BEV) rendering of the fused stream's own real objects —
raw detections, the live tracker's `TrackSnapshot`s, the candidate-gate
corridor, and matching GT boxes — color-coded by TTC urgency, ego-centered.
This is a sanity-check/diagnostic and portfolio artifact, not a new metric:
it visualizes outputs this project already computes, nothing new is scored.
Build order: static per-keyframe PNG first (fast to get the overlay/scale
logic right and inspect by eye), then extend to a per-scene video at native
rate (closest to the predecessor project's own `Step_8_BEV_Fusion_Video.ipynb`,
rebuilt fresh — V1's own BEV code is not portable: file-based per-step
handoff, per-sensor trackers merged after the fact, different frame/overlay
conventions. None of its code is reused; only a few of its *ideas* are:
per-scene video (never one combined file, since nuScenes-mini's scenes are
genuinely discontinuous clips), fps measured from real timestamp deltas
(never assumed), and filtering GT boxes to `vehicle.*`/`human.pedestrian.*`
so barrier/cone GT doesn't swamp the panel (same classes DEC-2 already
scopes GT events to).

## 2. Deliverables
- `src/ttcf/viz/bev.py` — pure drawing functions: ego marker, detections,
  tracks (colored by TTC urgency), corridor polygon, GT boxes. No
  pipeline-running logic here — takes already-computed objects in.
- `scripts/render_bev.py` — CLI: runs `run_pipeline.run_real_stream` for
  one scene (fused stream only, per Siva's 2026-10-08 scope choice),
  iterates its own per-event history, calls `bev.py`'s drawing functions,
  writes `outputs/figs/bev/<scene_token short>/keyframe_<sample_token short>.png`
  (Part A) or `outputs/figs/bev/<scene_token short>.mp4` (Part B).
- `tests/test_bev.py` — synthetic-scenario tests only (no real data needed
  to verify the drawing math itself): ego marker always at a fixed
  screen position, a track's screen position matches its known global
  position after the heading-up rotation, TTC color bins match config
  thresholds exactly at the boundary, corridor polygon vertices match
  `candidate_gate`'s own geometry.
- report + learning note (`reports/step12_report.md`, `docs/learning/step12_learning.md`).

## 3. What gets drawn (Siva confirmed, 2026-10-08, via direct question — not
re-opened as a DEC below)
- Tracks, colored by that instant's `TTCResult` urgency.
- Raw detections this scan (pre-gate, pre-track) — visual debugging of the
  adapter/gate, drawn smaller/fainter than tracks so they don't dominate.
- The candidate-gate corridor polygon (`candidate_gate`'s own
  `CORRIDOR_HALF_WIDTH_M`/`CORRIDOR_MAX_RANGE_M`, drawn once per frame from
  config — not re-derived).
- GT boxes, filtered to `vehicle.*` / `human.pedestrian.*` (same filter
  DEC-2 already uses for GT event scoping — one source of truth, not a
  second ad hoc filter).
- Stream: fused only (one shared tracker, matches the step11 frozen-config
  headline result).

## 4. Decisions to confirm (DEC-11 through DEC-14)

| ID | Decision | Options | Recommendation |
|---|---|---|---|
| DEC-11 | BEV orientation | (A) ego-centered, heading-up (frame rotates each instant so ego's own heading always points up) — V1's own convention; (B) ego-centered, north-up (global axes fixed, ego marker rotates in place) | **A.** Heading-up reads naturally for a braking/TTC context ("what's ahead" is always "up"); matches the one piece of V1's BEV worth keeping. |
| DEC-12 | Viewport extent | (A) fixed window every frame, sized to `CORRIDOR_MAX_RANGE_M` forward / `CORRIDOR_HALF_WIDTH_M`×~3 lateral; (B) auto-fit to whatever is visible this frame | **A.** A fixed scale is what makes TTC-urgency color meaningful frame to frame — auto-fit would make a far object look exactly as "big" as a near one and silently distort the sense of urgency. |
| DEC-13 | TTC color-bin source | (A) hardcode V1's own bins (red<3s/orange<6s/yellow<10s/green); (B) derive bins from this project's own `config.TTC_AEB_S`/`TTC_GRADUAL_S` | **B.** G12 (config is the single source of truth) — V3 already tuned these thresholds in step11; a visualization using a second, different set of numbers would mislabel exactly the cases the thresholds were tuned to get right. |
| DEC-14 | Which scenes may be rendered | (A) tune-split scenes only, ever; (B) any scene via explicit `--scene`, defaulting to tune split, with eval-split frames visibly labeled "EVAL" in the rendered output | **B.** Nothing here is scored or tuned from watching a video (no G13 risk), and a portfolio reader will want to see the actual frozen-config eval result, not only tune-split scenes — but the label prevents anyone later mistaking an eval-scene screenshot for a tuning input. |

## 5. Do NOT
- Do not compute any new metric here — this step visualizes existing
  `TrackSnapshot`/`TTCResult`/`ActionDecision`/GT outputs, nothing new.
- Do not use a second GT class filter different from DEC-2's.
- Do not let viewport auto-fit (ruled out by DEC-12) sneak in as a
  "nicer-looking" default.
- Do not port any V1 BEV code directly — rebuild from this project's own
  types (module docstring already states why: incompatible architecture).

## 6. Tests
1. Ego marker renders at a fixed screen position every frame (heading-up
   convention, DEC-11).
2. A synthetic track at a known global position/heading maps to the
   expected screen position after rotation — hand-computed, not just
   "runs without error."
3. TTC color bin boundaries match `config.TTC_AEB_S`/`TTC_GRADUAL_S`
   exactly (DEC-13) — e.g. ttc_s just above/below `TTC_AEB_S` lands in
   different bins.
4. Corridor polygon vertices match `candidate_gate`'s own corridor
   geometry (no second, drifted copy of the same numbers).
5. GT-box class filter matches DEC-2's filter exactly (shared helper, not
   a duplicated list).

## 7. Learning-note topics
Why a fixed viewport (not auto-fit) matters for reading urgency honestly;
heading-up vs. north-up and why the choice matters for interpretability;
reusing config thresholds instead of a second hardcoded set of numbers
(G12 in a visualization context, not just in code).

## 8. STOP
Static per-keyframe PNGs first — show Siva, confirm the overlay/scale look
right, **then** build the per-scene video. Wait for "approved" at each of
those two points before continuing.
