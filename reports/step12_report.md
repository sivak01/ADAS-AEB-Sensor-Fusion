# Step 12 report — BEV visualization (Parts A and B, both approved)

**Scope note:** step12 is new scope beyond README_MASTER.md §7's original
17-step build order (build order #18), requested by Siva post-step11.
Full step protocol used per Siva's explicit choice. Part A (static
keyframe PNGs) approved 2026-10-08 ("ok"); Part B (per-scene native-rate
video) built and reported in the same session, same approval.

## What was built
- `step12_bev_visualization.md` — step spec (this project's own
  `stepXX_*.md` format), covering both Part A and Part B.
- `src/ttcf/viz/bev.py` — pure BEV drawing primitives: ego-frame →
  heading-up display mapping (DEC-11), fixed viewport (DEC-12), TTC
  color bins derived from `config.TTC_AEB_S`/`TTC_GRADUAL_S` (DEC-13),
  candidate-gate corridor polygon, GT category filter reusing DEC-2's
  own `GT_CATEGORIES`, and draw functions for ego/corridor/detections/
  tracks/GT boxes/legend. No NuScenes dependency — pure math/matplotlib.
- `scripts/render_bev.py` — runs the fused stream once per scene
  (Siva's 2026-10-08 scope choice), causally extrapolates every track to
  each GT keyframe (reusing `evaluation/extrapolate.py`'s own
  `_extrapolate_snapshot`), and writes one PNG per keyframe to
  `outputs/figs/bev/<scene short>/keyframe_<sample short>.png`.
  `--scene <token>` (any scene, labelled "[EVAL]" if eval-split) or
  `--split {tune,eval}` (default tune, every scene in the split).
- `src/ttcf/geometry/transforms.py`: added `global_to_ego_vector` (the
  missing inverse of the existing `ego_to_global_vector` — needed to
  rotate a track's global-frame KF velocity into ego frame for the
  display's velocity arrows; none of the G17 Doppler work needed this
  direction, only this step does).
- `tests/test_bev.py` (8 tests, synthetic only) + 1 new transform test
  (`global_to_ego_vector` round trip).

## Decisions taken
DEC-11 through DEC-14, all with Claude's recommendation, approved by
Siva 2026-10-08 ("ok with ur recommendation") — full detail in
`docs/decisions.md`:
- DEC-11: heading-up orientation.
- DEC-12: fixed viewport (ego-frame x∈[-5,65]m, y∈[-15,15]m — forward
  extent derived from `CORRIDOR_MAX_RANGE_M`).
- DEC-13: TTC color bins derived from `config.TTC_AEB_S`/`TTC_GRADUAL_S`.
- DEC-14: any scene renderable via `--scene`, eval-split frames labelled.

## Tests
```
python -m pytest tests/ -q --ignore=tests/test_run_equivalence.py
150 passed, 1 warning in 30.48s
```
(141 prior + 8 new `test_bev.py` + 1 new `test_transforms.py` test.)

## Real-data verification
Ran `python scripts/render_bev.py --scene fcbccedd61424f1b85dcbf8f897f9754`
(scene-0103, tune split) — 40 keyframe PNGs written. Inspected
`keyframe_3950bd41.png` (a real GRADUAL-tier GT event): the orange
(GRADUAL-colored) track sits directly inside the candidate corridor,
overlapping its matching GT box — the color-coding and corridor/GT/track
geometry all agree visually with what step11's own metrics already say
this scene contains. Detections, GT boxes (purple, vehicle/pedestrian
only), corridor (dashed), and the heading-up ego triangle all render at
consistent scale across the viewport.

## Surprising or suspicious numbers (investigated as bugs first, G13)
While adding `global_to_ego_vector`, an initial round-trip test using an
arbitrary 3D "tumbling" quaternion (axis=[1,2,3]) failed by ~37% —
investigated before accepting or dismissing it. Root cause: `_apply_vector`
truncates a vector's z-component after rotation (by design — it's a 2D
vector API), which is lossless only when the rotation preserves the xy-
plane (pure yaw). Real nuScenes ego poses are always near-level (yaw,
with only tiny real pitch/roll), so this never shows up on real data —
confirmed NOT a code bug, fixed by rewriting the test with a realistic
yaw-only rotation (matching the project's other genuine vector round-trip
test, `test_vector_transform_rotates_not_translates`). No code changed
as a result of this investigation, only the test's own input.

## Assumptions introduced
None new in `config.py` — DEC-12's viewport extent and the ego-marker
drawing size are purely cosmetic (don't affect any computed result), so
they're local constants in `bev.py`, the same precedent the existing
`scripts/*_adapter_evidence.py` plotting scripts already use for their
own fixed plot extents.

## Part B — native-rate per-scene video

Built `render_scene_video()` in `scripts/render_bev.py` (`--video` flag),
reusing every Part A drawing/causality helper. Each of a scene's merged
native-rate events (all channels, chronological — 1401 events for
scene-0103 over 19.5s real time) becomes one frame, using the LIVE
tracker/TTC state at that exact event (not causal extrapolation to a
future GT time, unlike Part A's keyframe snapshots — there's no "future"
here, every frame already IS a real processed instant).

**Format: animated GIF, not MP4.** No ffmpeg binary is installed in this
environment; installing one is a new dependency for a cosmetic format
choice (README_MASTER.md §2's "ask before installing anything heavy"),
so PIL (already present) writes the GIF directly. GIF's own per-frame
`duration` list gives TRUE native-rate playback — each frame held for
exactly its own real inter-event gap (clipped to [30ms, 500ms] for
readability) — a more literal version of "fps measured from real deltas"
(step12 §1's own kept idea from V1) than a single averaged fps would be.

**Per-event detections, not a merged "latest known."** Each frame shows
ONLY that one event's own channel's raw detections (e.g. a
RADAR_FRONT_RIGHT frame shows only that radar's blips) — a deliberate
difference from Part A's keyframe PNGs (which show every channel's most
recent scan together). This is the more honest reading of "native rate":
the real system genuinely processes one sensor's scan at a time: a frame
showing the actual single scan that just arrived, not a synthesized
composite.

**GT boxes**: drawn only on the exact event that IS a keyframe's own
LIDAR_TOP sample (same t_us as a real `sample.timestamp`) — never
interpolated or held over between keyframes, so the video never implies
GT precision the dataset doesn't actually have.

**Real-data verification**: `python scripts/render_bev.py --scene
fcbccedd... --video` → 1401-frame GIF, `outputs/figs/bev/fcbccedd/fcbccedd_bev.gif`,
16.5MB, 42.4s total playback (real scene = 19.5s, so ~2.2x slower than
real-time — expected, since most real inter-event gaps were well under
the 30ms readability floor and got stretched up to it). Spot-checked
frame 0 (CAM_FRONT) and frame 700 (RADAR_FRONT_RIGHT, mid-scene) — both
show exactly that event's own detections, correctly colored live tracks,
and the fixed viewport/corridor consistent with Part A.

Tested with `--max-frames 40` and `--max-frames 150` first (27s for 150
frames including NuScenes load) before committing to the full ~3.5-minute
render, run in the background.

## Flagged, not built
- Selectable `--stream` (lidar/radar/camera) — scoped to fused only per
  Siva's 2026-10-08 choice; would be a small, mechanical extension later
  if wanted (reuses `_channel_adapter_dispatch` exactly like
  `run_pipeline.py` already does for every other stream).
- Action-tier/debounce-state text overlay — not requested in step12 §3's
  "what gets drawn" list; left out to avoid scope creep.

## Open questions for Siva
1. Both Parts A and B are built, tested, and verified against real data
   — any changes wanted before closing out step12 (e.g. render the
   remaining 4 tune-split scenes, or a specific eval-split scene for a
   portfolio shot)?
2. GIF file size (~16.5MB/scene) is fine locally but would bloat git if
   ever committed — confirm `outputs/figs/bev/` should be added to
   `.gitignore` alongside `outputs/cache/` (README_MASTER.md §2 already
   covers the general pattern; this step just has a concrete case of it).
