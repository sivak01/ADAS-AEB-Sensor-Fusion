# instant-ttc-fusion

## Mission — read this before touching any design decision

Give the ego vehicle a trustworthy, continuously-updated Time-to-Collision
estimate for whatever is currently in its path — using fusion to make that
estimate better than any single sensor could give alone — so the vehicle
(or driver) can act on it: steer, apply AEB, brake gradually, or cut power
to the accelerator.

**MOT asks:** "Is this still car #7, 100 frames later?"
**This project asks:** "Is the thing in front of me closing fast enough,
right now, that I need to act?"

The second question only ever needs a short, continuously-refreshed
memory — not a name that has to survive a whole clip.

## This is a deliberate rescope, not a fresh idea — know the history

A prior project (`multimodal-perception-ttc`, and its `v2/` async
rebuild) built a full Multi-Object Tracking (MOT) system on the same
nuScenes data: persistent object identity across an entire 20-second
scene, long occlusion tolerance (`MAX_MISSED_SECONDS`), scene-boundary
resets, and evaluation via full-track MAE/RMSE averaged over an object's
whole lifetime. That work surfaced real, well-diagnosed engineering
lessons (Mahalanobis gating over fixed thresholds, per-sensor measurement
noise, one shared Kalman filter implementation) — reuse those lessons.

**Do not reuse its scope.** A design that's excellent by MOT standards
(surviving a crowd of pedestrians correctly identified for 100+ frames)
was solving a harder, different problem than this project needs, and was
absorbing real risk (e.g. a same-class object cluster silently merging
into one track over a long lifetime) that barely matters here, because
this project never asks an identity to survive that long in the first
place.

## Does require

- Distance to an object, right now.
- Closing speed, right now — needs at least two temporally-linked
  detections of the SAME object. This minimal link is the only
  irreducible "tracking" requirement — a rate cannot be computed from
  one snapshot, ever, regardless of sensor or algorithm.
- Combining LiDAR + radar + camera so the fused estimate is more
  trustworthy than any one sensor's own noise/blind spots, at each
  instant.
- This repeated continuously, frame after frame, for as long as
  something is in the vehicle's path.

## Does NOT require — explicit non-goals

- An object keeping one persistent ID for an entire scene.
- Surviving long occlusion gaps (seconds) under one identity.
- Telling apart every individual object in a crowd for that object's
  entire time on screen.
- Full-track accuracy metrics (MAE/RMSE averaged over a track's whole
  lifetime), MOT-style metrics (IDF1, MOTA, ID-switch counts), or any
  metric that scores identity persistence over time.

## Additional scope — single-sensor TTC baselines, for comparison

Fusion's whole justification is that it beats any single sensor alone.
That's a claim to prove, not assume — V1/V2 only became trustworthy once
single-sensor baselines existed to check fusion against (see
`lessons-from-v1-v2.md`, item A.11). This project needs the same
discipline, scoped to its own short-memory/event-level design:

- Run the SAME event-level TTC pipeline three more times, each fed only
  one sensor's stream — LiDAR-only, radar-only, camera-only — using the
  identical short-memory tracker, gating, and TTC logic as the fused run.
  Same class, different input, exactly as V2 did for its own single-sensor
  runs — this is what makes the comparison trustworthy rather than
  apples-to-oranges.
- Report all four (lidar / radar / camera / fused) side by side on the
  same event-level metric (correct-action rate, false-brake rate,
  missed-brake rate — see Evaluation below). Do not report fusion's
  number alone.
- If fusion doesn't beat the best single sensor, report that plainly,
  the same way V1/V2 were required to. A result that isn't a clean win
  is still a real, reportable finding — don't tune toward a win.
- Camera's baseline must be genuinely single-sensor — no borrowed depth
  from LiDAR (V1's `camera` vs `camera_mono` distinction). If monocular
  depth estimation is used, that's the camera baseline; don't quietly
  let it use LiDAR-assisted depth and call it a fair comparison.

## Design contract

- **Short rolling memory only.** A track needs to persist only long
  enough to produce one trustworthy velocity estimate — target 2-4 real
  updates of working memory, not a multi-second eviction timer. If a
  design choice only matters for an object surviving more than about a
  second of missed detections, or for disambiguating identity across
  dozens of frames, that's out of scope — flag it, don't silently build
  it.
- **No scene-boundary reset logic needed as a distinct feature.** Given
  the short memory window above, a track's natural lifetime is already
  far shorter than a scene — this concern mostly dissolves rather than
  needing its own mechanism.
- **Forward-path relevance filter.** Only objects in, or clearly about
  to enter, the ego vehicle's path should compete for tracking
  attention at all — this project has no reason to track something off
  to the side that will never matter for a driving decision.
- **Decision debouncing, not single-frame triggers.** Don't act (brake/
  steer/etc.) on one frame's TTC alone. Require the danger condition to
  hold for N consecutive real updates (start with N=2-3) before
  triggering an action — this is standard real-AEB practice, and it's
  the cheap, well-precedented answer to "what if one frame's
  same-object match was briefly wrong," instead of chasing long-horizon
  identity robustness to protect against the same risk.
- **Fusion stays per-sensor-R-weighted**, evaluated fresh at each
  instant an object is being assessed — not accumulated as a
  lifetime-spanning multi-sensor identity property.

## Reusable primitives — carry the tool over, not the scope

From `multimodal-perception-ttc`'s `v2/` rebuild, these concepts are
proven and worth reusing in spirit (do not copy files wholesale — they
were built for a longer-memory design):
- Constant-velocity Kalman filter, sensor-agnostic (R supplied per
  update call, not baked into the filter).
- Mahalanobis-distance gating with a chi-square threshold, over a fixed
  Euclidean gate.
- Single source of truth for per-sensor measurement noise (`R`),
  documented as measured vs. assumed, never duplicated across files.

## Evaluation — this is the part that changes most

Do NOT evaluate with full-track MAE/RMSE or any MOT metric. Instead,
evaluate at the event level: for each moment a real object was in the
ego vehicle's path (per nuScenes ground truth), was the derived
TTC/action (brake, no-brake, steer) correct compared to what ground
truth implies at that instant? Report this as an event-level
precision/recall-style result (correct-action rate, false-brake rate,
missed-brake rate), not as a trajectory-accuracy number.

Report this metric for all four runs from the single-sensor-baselines
scope above — lidar / radar / camera / fused — as one table, every time.
Fusion's number in isolation is not a complete result.

## Working style

- Ask before expanding scope beyond what's stated above.
- If something from the MOT-scoped predecessor looks tempting to add
  "for robustness," check it against the Design contract section first
  — if it only helps long-horizon identity, it's out of scope here.
