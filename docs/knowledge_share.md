# instant-ttc-fusion — knowledge share

*A plain-language walkthrough of what this project is, what it found, and
what it's worth remembering. Written to teach from, not just to record.*

## 1. Mission, and the rescope from MOT

The goal: estimate **Time-to-Collision (TTC)** — how many seconds until
the ego vehicle would hit the nearest real object ahead of it — in real
time, from LiDAR, radar, and camera, on the nuScenes-mini dataset, and
honestly test whether *fusing* those three sensors actually beats using
any one of them alone.

The predecessor project (V1, and its async rebuild V2) was scoped as a
general **Multi-Object Tracker (MOT)**: track every object in the whole
scene, for its whole lifetime, and evaluate with MOT metrics (ID
switches, track length, MOTA/MOTP-style scores). This project deliberately
does **not** do that. A collision-avoidance system doesn't need to know
that "track #47 is the same car it saw 30 seconds ago three blocks back" —
it needs to know, right now, whether *anything* in front of it is closing
fast enough to require braking. Rescoping away from MOT let this project
drop an entire category of complexity (long-horizon identity, appearance
re-identification, track-length metrics) that was never actually needed
for the real question being asked.

## 2. The architecture, in one picture

```
 nuScenes-mini raw sensor records (LIDAR_TOP | 3x RADAR_FRONT* | CAM_FRONT)
                              |
       native-rate event stream (reacts at each sensor's own real rate)
                              |
        ,--------------------+--------------------,
   LiDAR adapter         Radar adapter        Camera adapter
 (ground removal +      (DBSCAN cluster,     (YOLOv8n boxes +
  DBSCAN cluster)         Doppler OFF)         ground-plane depth)
        `--------------------+--------------------'
                              |
              forward-path candidate gate (coarse filter)
                              |
        SHARED short-memory tracker (ONE tracker, all sensors --
           early fusion, Hungarian assignment, Mahalanobis gating)
                              |
          track-level path relevance  +  TTC estimator
                              |
           debounce (N consecutive real updates) + action tier
                              |
     bounded causal extrapolation to ground-truth keyframe instants
                              |
          event-level scoring (confusion matrix, Wilson intervals,
             observability variants, hard eval-split guard)
```

The "fused" run is not a fourth, separate pipeline — it is the exact same
code as the three single-sensor runs, just with all three adapters feeding
the same shared tracker at once. This matters: it means the four-way
comparison is a fair one. (Full diagram with build/approval status:
`reports/instant-ttc-fusion_architecture_status.docx`.)

## 3. Ten decisions, and the mistake each one prevents

1. **Native-rate event stream, not a fixed 2Hz tick.** Waiting for a
   synchronized tick throws away information — a sensor with new data
   right now should be allowed to act on it now, not wait for the slowest
   sensor's clock.
2. **One shared tracker, early fusion — never a per-sensor tracker merged
   afterward.** Late fusion has to solve "which track from sensor A is
   the same real object as which track from sensor B," a hard, error-
   prone re-identification problem this design avoids by construction.
3. **Gate on the innovation covariance `S = P_pred + R`, not a fixed
   Euclidean radius.** A fixed-distance gate is wrong for every sensor
   simultaneously — too tight for a noisy sensor, too loose for a precise
   one. Worked example: `d²(S=P+R)=0.626` (accept) vs. `d²(P alone)=29.195`
   (would wrongly reject) for the identical real match.
4. **Nearest-surface-to-ego reference point (DEC-1), never centroid.** A
   partially-seen car's centroid sits 1-2m behind its actual near bumper
   — exactly the margin an AEB threshold lives or dies on. Worked example:
   a 4m car with near/far edges at 24m/28m gives TTC=2.6s via centroid vs.
   the correct 2.4s via nearest-surface, at 10 m/s closing.
5. **Bounded, short-memory tracks (2-4 updates), not lifetime MOT
   identity.** This is the direct consequence of the §1 rescope — a
   collision system doesn't need to remember an object past the last few
   real updates, and not pretending otherwise avoids a whole class of
   long-horizon tracking bugs this project never had to solve.
6. **A structural isolation test for the camera path
   (`test_camera_isolation.py`).** The predecessor project's own `camera`
   baseline secretly used median LiDAR-point depth inside each 2D box —
   its "camera-only" result was actually LiDAR+camera fusion wearing a
   camera label. This project statically scans the camera module's own
   source for any reference to the other sensors, making that exact
   mistake impossible to reintroduce silently.
7. **Isolating real ego-motion from sensor jitter when measuring a
   closing-speed deadband.** A stationary object's *full* closing-speed
   formula includes ego's own real velocity (often several m/s, not
   noise) — measuring "jitter" from the full formula gave a contaminated
   ~5 m/s median; isolating just the object's own estimated velocity
   (which should be exactly 0 for a truly stationary object) gave the
   real, much smaller noise floor.
8. **Never promote a sensor's noise value (R) to MEASURED without an
   explicit human decision, even when the number clears the trust bar.**
   Both LiDAR (81.2% inlier rate) and radar (92.0%) cleared this
   project's own 70% threshold during development, and both were
   deliberately kept ASSUMED anyway — the measurement method has known
   limitations (nearest-neighbour matching, not real identity), and this
   would have been the project's first-ever MEASURED value, with system-
   wide effects on tracker gating.
9. **Pre-declare the tuning selection criterion (DEC-7) before running
   any sweep.** Choosing "what counts as a good result" *after* seeing
   the numbers is exactly how an evaluation quietly biases itself toward
   the answer someone wanted. The criterion (mean correct-action rate
   across all four streams) was written down and locked before the
   SIGMA_A sweep ever ran.
10. **A held-out eval split, scored exactly once, enforced by a hard
    code-level guard — not a documented policy.** Scoring the eval split
    requires an explicit `--final` flag *and* a cryptographic hash match
    against a frozen configuration file; every attempt, successful or
    refused, is permanently logged. This turns "don't tune on the test
    set" from a rule people have to remember into something the code
    itself refuses to violate.

## 4. The four-run result, and how to read it honestly

| Run | correct_action_rate | false_brake_rate | missed_brake_rate |
|---|---|---|---|
| lidar | **97.0%** | **0.8%** | 28.6% |
| radar | 94.0% | 3.2% | 42.9% |
| camera | 95.5% | 0.8% | 71.4% |
| **fused** | 87.2% | 11.1% | **28.6%** |

**Fusion did not beat the best single sensor** on this dataset with this
design — LiDAR alone has the best correctness and the fewest false
alarms. But fusion is not simply worse across the board: it ties LiDAR
for the *best* missed-brake rate, meaningfully better than radar or
camera alone — genuinely catching more real danger events by combining
sensors with different blind spots. The cost is a false-brake rate 3.5-
14x any single sensor's own rate, and about half of that specific cost
traces to one scene (a crowded parking lot) that is a textbook instance
of this project's own already-known, already-accepted crowd-conflation
limitation, not a new defect.

**How to read a table with tiny counts honestly**: every `missed_brake_rate`
above is built on only 4-7 real danger events (the eval split has 7
total: 2 GRADUAL, 5 AEB) — labelled "indicative only" throughout, and
correctly so. A missed-rate of "71.4%" sounds damning until you see it's
5 out of 7 — one or two different events going the other way would swing
that number by 15-30 percentage points. The `correct_action_rate` figures
(built on all 133 scored keyframes) are much more statistically solid and
should carry more weight in any real judgment.

**What an honest "fusion did not win" looks like**: it looks like this —
a real, reproducible, investigated result, reported with its full
counts and confidence intervals, with the mechanism behind the surprising
number (crowd conflation in one specific scene) traced and named rather
than hidden, and with the genuine partial benefit (missed-brake rate)
stated alongside the genuine cost (false-brake rate) instead of
collapsing both into a single misleading verdict.

## 5. What was measured vs. assumed

**0 parameters are MEASURED** in the strict sense this project uses that
word (independent empirical measurement against ground truth, with a
stated inlier rate). **50 are ASSUMED** (many with strong evidence behind
them — real sweeps, real percentile measurements from real data — but
still a human-reviewed choice, not a blind statistic). **1 remains
PLACEHOLDER** by design (`SENSOR_R_CAMERA` is a range-dependent function,
not a single number). This is not a weakness to apologize for — it is
this project's own honesty convention working as intended: a threshold
that came from a real sweep of real data, reviewed by a human before
being trusted, is exactly what "ASSUMED with evidence" is supposed to
mean, and reserving "MEASURED" for a stricter bar (like the sensor-R
measurements that cleared 70%+ inlier rates but were still held back)
keeps that label meaningful rather than diluted.

## 6. Limitations (see `reports/final_report.md` §4 for full detail)

Crowd conflation (open, safety-relevant); a straight (never curved)
forward-path corridor; flat-ground camera depth (error grows with
range²); radar Doppler deliberately unused; ghost radar returns not
filtered upstream; only 7 real danger events in the eval split; ego
velocity has a small structural KF lag (causal by design, never a
hidden shortcut).

## 7. What you'd do next

- **Solve crowd conflation properly** — this is the single highest-
  leverage next step, given it's the dominant traced cause of fusion's
  own false-brake cost. A lightweight appearance cue or a smarter gating
  rule for closely-spaced same-class detections would likely help fusion
  the most, since fusion is precisely where more detections create more
  opportunities for this failure mode.
- **A real Doppler ablation** (with explicit approval) — radar's
  instantaneous velocity is currently unused entirely; a careful,
  validated ablation could materially improve TTC responsiveness,
  especially for the fused stream.
- **Curved-path / lane-aware corridor** — would remove a real, named
  blind spot at turns and intersections.
- **A larger, denser dataset** — nuScenes-mini's 7-11 real danger events
  is the single biggest statistical limitation in every result this
  project reports; a larger eval set would turn "indicative only" into
  genuinely solid numbers.
- **Revisit the two sensor-R MEASURED promotions** once this project's
  own tracker has run across a larger dataset with more robust
  association — both LiDAR and radar were one honest decision away from
  MEASURED status already.
