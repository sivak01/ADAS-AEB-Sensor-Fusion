# Step 12 — learning note (Part A: static BEV)

## The idea in plain language
A bird's-eye-view (BEV) image is a top-down map of what the system
"believes" at one instant: where the ego car is, what raw sensor blips it
saw, what the tracker thinks is out there (colored by how urgent the
braking decision is), the forward corridor it cares about, and the real
ground-truth boxes for comparison. It's a sanity check you can see with
your eyes, not a new number — everything drawn already exists elsewhere
in the pipeline (tracks, TTC, corridor, GT); this step only draws it.
The camera convention is "heading-up": the ego car's own forward
direction always points to the top of the image, like a driver's view,
not a fixed compass direction — so a car approaching from ahead always
appears to be coming from the top of the frame, consistent frame to frame
even as the real ego vehicle turns.

## Tiny worked numeric example
Ego is driving north; in ego's own frame, "forward" is always +x and
"left" is always +y (regardless of compass heading — the transform
pipeline already rotated everything into ego's own frame before this
step ever sees it). A detection sits 10m ahead and 5m to the ego's left:
ego-frame `(x=10, y=5)`. To draw it heading-up: `display_x = -y = -5`,
`display_y = x = 10` — it lands 5 units left of center, 10 units up.
If that same detection's TTC estimate comes out to 1.0s, and
`config.TTC_AEB_S = 1.2s`, then `1.0 <= 1.2` → it's drawn **red** (AEB
tier) — using the project's own already-tuned threshold, not a separate
hardcoded number picked just for the picture.

## Why it is designed this way
- **Fixed viewport, not auto-fit (DEC-12).** If the view rescaled to fit
  whatever's visible each frame, a far-away red (urgent) object could
  look exactly as "big" as a near one — the picture would visually lie
  about urgency. A fixed scale keeps "how close does this look" honest.
- **Color bins pull from `config.TTC_AEB_S`/`TTC_GRADUAL_S` (DEC-13),
  never a second hardcoded set.** This is G12 applied to a
  visualization: V1's own BEV code hardcoded its own color thresholds,
  separate from whatever thresholds its actual decision logic used —
  so its picture and its real behavior could literally disagree. One
  source of truth prevents that class of bug here.
- **GT box filter reuses DEC-2's `GT_CATEGORIES` exactly.** A second,
  slightly different hardcoded category list for "what counts as a
  pedestrian/vehicle" is exactly the kind of silent drift this project's
  own config discipline (G12) exists to prevent.
- **Rebuilt fresh, not ported from V1's own BEV notebook.** V1's BEV
  code assumes per-step file handoff and per-sensor trackers merged
  after the fact — neither exists in this project's single shared,
  in-memory, early-fusion tracker. Only a few of V1's *ideas* survived
  (per-scene output, GT class filtering) — none of its code.

## How to explain this in an interview
- "The visualization draws nothing new — it's a window onto outputs the
  pipeline already computes, so if the picture and the metrics table
  ever disagreed, that itself would be a bug to chase, not two
  independent sources of truth to reconcile."
- "I deliberately chose a fixed-scale viewport over an auto-fit one,
  because auto-fit would silently distort how urgent something looks —
  a small design choice with a real correctness implication for a safety
  visualization."
- "The TTC color thresholds in the picture are the literal same config
  values the braking decision uses — not a separate set chosen to look
  good, which is a mistake I found and deliberately avoided."

## Part B addendum — native-rate video
The static PNGs (Part A) show "what the system believed at a GT
instant," using causal extrapolation to get there. The video (Part B) is
simpler in one sense: every frame IS a real, already-processed instant,
so it just shows the live tracker state directly — no extrapolation
needed. But it's also more literal about what "native rate" really
means: each frame shows only the ONE sensor scan that triggered it (a
radar frame shows radar's own blips, not a composite of every sensor's
last known scan), because that's genuinely how the real system processes
events — one at a time, not as a synchronized multi-sensor snapshot.
Frame *timing* is also genuinely native: GIFs can hold each frame for
its own duration, so the video reproduces the real, uneven rhythm of
sensors firing at different rates (LiDAR ~20Hz, 3 radars, camera
~12Hz) rather than smoothing it into one constant frame rate.

## Common misunderstandings
- "Heading-up" does not mean the image rotates as the car turns in real
  time within one frame — each static frame is drawn at one instant,
  already in that instant's own ego frame; there's no animation yet
  (that's Part B).
- GT boxes shown are **not** restricted to "in-path" objects the way the
  official metrics are — every vehicle/pedestrian GT box in the viewport
  is drawn, so you can see objects the corridor filter correctly
  excludes, too. That's intentional (a visual check the metrics alone
  can't give), not a bug.
- "Raw detections" means pre-candidate-gate, pre-tracker points — some
  of what you see as a faint blue "x" will correctly never become a
  track at all (expected, not noise to be filtered out of the picture).
