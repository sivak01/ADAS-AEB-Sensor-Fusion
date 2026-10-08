# What NOT to do — instant-ttc-fusion

Every item here is a mistake this project's predecessors (V1, V2) actually
made, or a specific risk already identified but not yet resolved. Check a
design decision against this list BEFORE building it, not after.

---

## 1. Don't let MOT scope back in

These all look like reasonable "robustness" additions in isolation. They
aren't, for this project. Each one only pays for itself if identity needs
to survive a long time — this project's tracks don't.

- **Don't add a multi-second eviction timer.** A track here only needs to
  survive long enough to produce one trustworthy velocity — eviction
  should be roughly "a few missed inter-arrival gaps," not seconds.
- **Don't add scene-boundary reset as its own subsystem.** With short-lived
  tracks by design, this mostly can't happen anyway — building a dedicated
  mechanism for it is solving a problem that shouldn't exist here.
- **Don't track which sensors contributed to a track over its whole
  lifetime.** Track which sensors contributed to the CURRENT estimate.
  Lifetime-spanning bookkeeping is a MOT concept.
- **Don't build separate per-sensor trackers that get merged afterward.**
  That's late fusion, and it's the design V1 had to painfully audit its
  way out of (only ~35% of "fused" output was ever a real merge). Use one
  shared track set that every sensor writes into directly from the start.
- **Don't evaluate with full-track MAE/RMSE, or any MOT metric** (IDF1,
  MOTA, ID-switch counts, MT/PT/ML). These score identity persistence over
  time. Use event-level correctness instead.

## 2. Don't trust a sensor's own reported values without checking whose they are

- **Don't reuse one channel's timestamp for another channel.** LiDAR,
  radar, and camera each fire at their own real timestamp within one
  nuScenes "sample" — confirmed by direct measurement, not assumption.
- **Don't reuse one channel's `ego_pose` for another channel.** Same
  reasoning — each `sample_data` record has its own `ego_pose_token`,
  and the vehicle moves between when different sensors actually fire.
- **Don't feed radar's raw per-point returns straight into tracking.** A
  single real object commonly produces multiple blips per scan. Cluster
  per-channel-per-scan first, or the tracker will treat several points on
  one object's surface as repeated noisy measurements of one point.
- **Don't process only 2Hz `samples/`.** Use `sweeps/` for true native
  rate — the entire point of an event-driven design is reacting to the
  freshest detection, not waiting for a synchronized tick.

## 3. Don't estimate velocity by finite-differencing two raw points

- **Never** compute an object's velocity, or the ego vehicle's own
  velocity, as `(pos2 - pos1) / dt`. Run positions through the shared
  constant-velocity Kalman filter instead. This exact shortcut was found
  and fixed independently in four different places in V1/V2 — it's worth
  treating as a standing rule, not something to re-derive project by
  project.
- **Don't use a raw sensor-reported velocity field for TTC** (e.g. radar's
  Doppler `relative_speed`) in place of the KF's own velocity state,
  unless that's a deliberate, scoped ablation with its own KF-state
  extension — not a shortcut to skip the filter.

## 4. Don't gate on position alone

- **Don't gate using only the track's predicted covariance.** The correct
  quantity is the innovation covariance `S = P_pred + R`, including the
  INCOMING detection's own measurement noise. Gating on `P_pred` alone
  produces a gate that collapses too tight right after a low-noise
  sensor's update — a real, found-and-fixed bug, not a theoretical risk.
- **Don't use a fixed Euclidean distance threshold for gating.** Use
  Mahalanobis distance with a chi-square confidence threshold — a fixed
  gate either fragments fast objects (too tight) or admits wrong matches
  (too loose), and there's no single fixed number that avoids both.
- **Don't assume a short memory window automatically protects against
  crowd conflation.** Two adjacent same-class objects (pedestrians at a
  crosswalk) can be conflated on a track's very FIRST match — shortness
  of memory limits how long a bad decision persists, it does not prevent
  the bad decision from happening. This is a named, open limitation, not
  something the rescope silently solved.
- **Don't assume one shared `sigma_a` is safe across sensors with wildly
  different R without re-checking it in this project's own context.** The
  same constant that under-gates a low-R sensor can over-gate a high-R
  sensor — measured and confirmed in V2 across a 75x R range. Re-measure
  here; don't inherit the old number on faith.

## 5. Don't compute TTC carelessly

- **Don't compute closing speed from an object's raw absolute velocity.**
  Track state is global-frame; closing speed is the RELATIVE motion
  toward the ego vehicle: `((vx-ego_vx)*dx + (vy-ego_vy)*dy) / distance`.
  Skipping the ego-velocity term silently reports high closing speed for
  two vehicles simply traveling together at the same speed.
- **Don't let TTC go unbounded on near-zero closing speed.** A closing
  speed that's technically positive but tiny produces an astronomically
  large but "finite" TTC that isn't a meaningful signal. Deadband on
  `closing_speed <= MIN_CLOSING_SPEED`, not just `<= 0`.
- **Don't conflate `MIN_TRUSTED_SPEED` and `MIN_CLOSING_SPEED`.** One gates
  whether an object's raw speed is real motion vs. sensor jitter; the
  other gates whether its motion is actually closing the gap to ego. An
  object can have plenty of raw speed while still having near-zero
  closing speed (tangential motion) — they are not the same check.
- **Don't act on a single frame's TTC.** Require the danger condition to
  hold for N consecutive real updates (debounce) before triggering
  brake/steer/etc. — the cheap, standard defense against one noisy match.
- **Don't use inconsistent distance definitions between the tracker's
  output and ground truth** (e.g. centroid-to-ego on one side, box-surface
  on the other). Pick one definition, use it identically on both sides,
  or the comparison is systematically biased before any real error enters.

## 6. Don't evaluate against ground truth carelessly

- **Don't compare a track's nearest update to a GT annotation.** GT exists
  only at 2Hz keyframes; the tracker updates at native rate. Extrapolate
  the track's state (position via its own velocity) to the GT's EXACT
  timestamp before comparing.
- **Don't extrapolate without bounds.** Cap forward extrapolation at
  roughly the eviction gap (a track that's effectively already gone
  shouldn't be stretched arbitrarily far to match a late GT instant), and
  never extrapolate backward before a track's first real snapshot. Both
  are real bugs found in V2, not hypothetical edge cases — and with only
  2-4 updates of memory here, a track being mid-eviction or not-yet-
  started at a given GT instant is proportionally MORE likely than it was
  in V2's longer-lived tracks.
- **Don't score a sensor's blind spot as a tracking failure.** Use
  `num_lidar_pts`/`num_radar_pts`/visibility to identify GT objects a
  given sensor plausibly could never have seen, and report single-sensor
  results with and without those instances — otherwise a sensor's honest
  coverage gap gets misread as it performing worse than it did.
- **Don't let the camera baseline secretly borrow LiDAR depth.** If it
  does, it's not a real single-sensor comparison — same mistake V1 made
  with its original `camera` baseline before `camera_mono` was built to
  correct for it.
- **Don't apply class-based filtering asymmetrically across the four
  comparison runs.** Only camera naturally provides a class label — if
  class gates the forward-path filter in the fused run but LiDAR/radar-
  only runs are class-blind, the four-way comparison isn't apples to
  apples. Decide this explicitly, don't let it happen by default.

## 7. Don't tune or interpret dishonestly

- **Don't tune debounce thresholds or danger-zone cutoffs on the same
  scenes you'll report evaluation numbers on.** Use a separate tuning
  split. Numbers tuned and reported on the same data don't mean what they
  look like they mean.
- **Don't tune anything to make fusion look better than it measures.** If
  fused doesn't beat the best single sensor, report that plainly — it's a
  real, reportable finding, and V1/V2's most useful results came from
  exactly this kind of honest reporting, not from a clean win.
- **Don't claim an R value is "measured" when it's actually a literature-
  typical assumption.** State which sensors' noise values are empirically
  measured and which are assumed, and why, every time the config is
  documented.
- **Don't over-read radar-only's results if Doppler velocity hasn't been
  wired into the KF yet.** Deferring Doppler to a later ablation is a
  reasonable scope call, but it means radar is competing with its main
  advantage disabled — say so wherever the four-run table is reported, or
  "radar performed worst" will be read as a claim about radar's real
  capability rather than about this specific, incomplete configuration.
- **Don't treat a small number of danger events in nuScenes-mini as
  statistically solid.** Report raw counts alongside rates, and decide up
  front how you'll handle a thin denominator rather than discovering it's
  a problem after the numbers come back.

## 8. Don't skip the cheap verification steps

- **Don't accept a surprising or "wrong-looking" result without
  investigating it as a likely bug first.** Nearly every real finding in
  V1/V2's investigation started as a number that looked off and turned
  out to have a specific, fixable cause — not as a conclusion to accept
  at face value.
- **Don't run multiple build steps in one shot without reviewing each
  one.** The deliverable-by-deliverable, stop-and-report discipline is
  what caught the sigma_a/gating/GT-extrapolation bugs before they
  compounded into something harder to isolate later.
- **Don't skip the basic synthetic tests before running on real data:** a
  stationary object with a moving ego should give closing speed equal to
  ego speed; two co-traveling vehicles should give TTC = infinity;
  near-tangential motion should hit the deadband; a single bad match
  should not trigger an action, because of the debounce. These are cheap,
  and each one directly targets a mistake already made once.
