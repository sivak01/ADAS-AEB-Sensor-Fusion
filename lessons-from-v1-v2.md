# Lessons carried forward from V1 / V2 — filtered for instant-ttc-fusion

V1 (`multimodal-perception-ttc`) and its `v2/` async rebuild solved MOT
(scene-long identity persistence). This project solves a narrower
problem. Everything below is sorted by whether that difference matters.

---

## A. Carry over directly — correctness bugs that apply regardless of memory horizon

These aren't MOT-specific. They're just correct engineering, and would
bite this project exactly as hard if ignored.

1. **Every sensor's data must be in the global (ego-motion-compensated)
   frame before any comparison across time or sensors.** Comparing raw
   sensor-local coordinates makes a stationary object look like it moved
   by the ego vehicle's own displacement. This matters *more*, not less,
   here — a short-memory system has fewer data points to average this
   error out over.
2. **Each sensor CHANNEL has its own native timestamp and its own
   `ego_pose`, distinct from any other channel's** — even within one
   nuScenes "sample." Two separate bugs were found and fixed from
   reusing LIDAR_TOP's timestamp/pose for radar. Any new adapter must
   look up each channel's own values, never borrow another's.
3. **Radar needs per-scan clustering before tracking.** A single real
   object commonly returns multiple radar blips (bumper, wheel well,
   panel) in one scan. Feeding them in as separate near-simultaneous
   detections corrupts a filter update — it looks like repeated noisy
   measurement of one point when it's actually several points on one
   surface. Cluster per-channel-per-scan (simple proximity/DBSCAN)
   before anything downstream sees the data.
4. **Mahalanobis gating must include the incoming detection's own
   measurement noise (R), not just the track's predicted covariance.**
   The correct quantity is the innovation covariance `S = P_pred + R`.
   Gating on `P_pred` alone silently produces a gate that's too tight
   right after a low-noise sensor (e.g. LiDAR) updates a track — a real
   bug found and fixed in V2, and it would reproduce identically here if
   skipped.
5. **TTC needs the object's velocity relative to the EGO vehicle's own
   velocity, not the object's raw absolute velocity.** Track state is
   global-frame, so `(vx, vy)` is absolute. Closing speed is
   `((vx-ego_vx)*dx + (vy-ego_vy)*dy) / distance`. Skipping the ego term
   silently reports large closing speed for two vehicles co-traveling at
   the same speed — a common, safety-relevant highway scenario.
6. **TTC needs a closing-speed deadband, separate from any raw-speed
   deadband.** `closing_speed <= MIN_CLOSING_SPEED` (not `<= 0`) before
   returning `inf`. Near-zero-but-positive closing speed (near-tangential
   relative motion) otherwise produces astronomically large but
   technically finite TTC values that are meaningless as a signal — and
   for an AEB-style system, a nonsensical large TTC is functionally the
   same failure as a missing one.
7. **Never derive velocity (object's or ego's own) from a raw 2-point
   finite difference.** Run positions through a constant-velocity Kalman
   filter instead — same primitive, used for tracked objects and for the
   ego vehicle's own velocity (nuScenes `ego_pose` has no velocity field;
   it must be derived, and the same discipline applies to deriving it).
8. **One sensor-agnostic Kalman filter class**, with measurement noise
   (`R`) supplied per `update()` call (known only at update time — which
   sensor actually matched) and process noise (`sigma_a`) fixed once at
   construction (a property of the *object's* motion uncertainty, not of
   whichever sensor is currently reporting on it). Keep the filter with
   zero knowledge of sensor identity.
9. **Single source of truth for every sensor's `R`, imported everywhere,
   never duplicated.** A flat, un-tuned `R` sat unused next to correctly
   per-sensor-tuned values in V1 for a long time before being caught —
   costly, avoidable with one config file.
10. **State explicitly, per sensor, whether an `R` value is measured or
    assumed** — and why, if it couldn't be measured (V1's LiDAR/radar `R`
    couldn't be empirically measured because their own fragmentation
    corrupted the ground-truth-residual estimate below a trustworthy
    inlier rate). Don't let an assumed number quietly read as measured.
11. **Treat "fusion is best" as the claim under test, not a given.**
    Every fusion result in V1 that looked wrong turned out to have a real
    cause (flat R, bad gate, borrowed depth). Assume the same discipline
    here: an unexpected number is a bug to find, not a result to report
    as-is.

## B. Carry over — architecture that fits this goal *better* than it fit MOT

These weren't built for this project, but they turn out to be a more
natural match for it than for the system they were built for.

1. **Measurement-level (early) fusion — one shared track set that every
   sensor writes into directly, no separate per-sensor-track-then-merge
   step.** This was built to fix MOT's dilution problem (only ~35% of
   "fused" output ever being a real multi-sensor merge). For a
   short-memory, single-current-object system, it's arguably an even
   better fit: there's no long-lived per-sensor track history to
   reconcile after the fact — every detection should compete for the
   current best estimate the instant it arrives.
2. **Asynchronous, native-rate event processing** (each sensor's own real
   timestamp, not a synchronized 2Hz frame). For "give me the freshest
   trustworthy TTC," waiting for a synchronized frame is actively
   counterproductive — you want to react to whichever sensor has new
   information right now, at whatever rate it arrives.

## C. Carry over, but the tradeoff genuinely changes here — don't just reapply the old conclusion

1. **The shared-`sigma_a` dilemma (tight-R sensors under-gate, loose-R
   sensors risk over-merging) was diagnosed against LONG-lived tracks.**
   The over-merging failure mode there (a pedestrian-cluster track
   silently drifting across dozens of detections) needed a long lifetime
   to become severe. In a design where a track only needs to survive
   2-4 updates, the *window* for that drift to accumulate is much
   smaller — this constraint likely binds less hard here. Worth
   re-measuring in this project's own context rather than assuming V1's
   numeric conclusion (which value of sigma_a "won") still applies —
   the right value may be different once track lifetime is short by
   design, not just by eviction tuning.
2. **A new implication worth stating explicitly, not present in V1's
   framing at all: gating precision on the FIRST 1-2 matches now matters
   proportionally more than it did in MOT.** In a long-lived track, one
   early wrong match gets diluted by dozens of later correct ones when
   averaged into a lifetime metric. Here, there is no long tail to dilute
   into — if the first couple of matches are wrong, that's most or all
   of the track's short life. This means the gate/R correctness work
   (item A.4 above) isn't just "still relevant," it's *more* load-bearing
   than it was in the system it was originally fixed in.
3. **The same-class crowd-conflation failure (position+class alone can't
   tell apart multiple people standing close together) is NOT safely
   smaller here just because tracks are short-lived — it may be more
   safety-relevant, not less.** The scenario where this bites hardest —
   several pedestrians close together — is exactly a crosswalk, exactly
   the situation an AEB system most needs to get right, and exactly
   where a short window doesn't help, because the conflation can happen
   within that first short window just as easily as it did over a long
   one. Keep this as a known, named limitation (see item D), not
   something the rescope quietly resolves.

## D. Explicitly leave behind — do not carry these into this project

State these plainly so a future session doesn't rebuild them "for
robustness" without noticing they're solving MOT's problem, not this
project's.

- Long eviction windows (`MAX_MISSED_SECONDS` in the 1-2 second range) —
  this project's tracks are short-lived by design; eviction only needs
  to ask "did I get an update in roughly the last expected inter-arrival
  gap," not survive a multi-second gap.
- Scene-boundary reset as its own subsystem — dissolves naturally once
  tracks don't live long enough to plausibly cross a scene boundary.
- `sensors_seen`-over-a-track's-whole-lifetime bookkeeping, and any
  "multi-sensor composition" metric computed the same way — reframe as
  "which sensors contributed to THIS current estimate," not a lifetime
  property.
- Full-track MAE/RMSE, and any MOT-style metric (IDF1, MOTA, ID-switch
  counts, MT/PT/ML) — these score identity persistence over time, which
  this project doesn't need. Evaluate at the event level instead (see
  the project's own CLAUDE.md / project knowledge for the replacement
  metric).
- Chasing a single shared `sigma_a` that avoids ALL over-merging across
  an entire scene — not this project's problem to solve at that scope
  (see item C.1).
- Appearance-based re-ID as something to build now — still correctly
  named as future work (item C.3's limitation is real), not as an
  immediate requirement.

## E. Minor, low-stakes facts worth keeping, not acting on

- nuScenes-mini has 10 scenes, not 6.
- `samples/` = 2Hz keyframes; `sweeps/` = each sensor's true native rate
  (~12-20Hz). This project should default to reading `sweeps/` from the
  start, unlike V1/V2 which deferred it — "freshest possible detection"
  is the whole point here, and samples-only cadence was already shown
  (in V2's sigma_a investigation) to distort filter tuning in ways that
  wouldn't occur at true native rate.
