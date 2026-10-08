# STEP 06 — Shared short-memory tracker

**Build order #8.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 00, 02, 03, 07A, 05A, D0, 01 approved.
**Scope of this step: synthetic data only.** No real detections yet.

## 1. Purpose (plain language)
The tracker keeps just enough memory to link two or more detections of the *same* object so a velocity can be estimated. It is early-fusion: LiDAR, radar, and camera detections all write into **one shared set of tracks** (G10). Tracks are short-lived by design (G11).

## 2. Deliverables
- `src/ttcf/tracking/tracker.py`
- `tests/test_tracker.py` (synthetic scenarios)
- `scripts/synthetic_scenarios.py` (reusable generators for later steps)
- figures in `outputs/figs/step06/`, report + learning note

## 3. Specification

**Interface (event-driven):** `ShortMemoryTracker.process_scan(dets: list[Detection], t_us) -> list[TrackSnapshot]` — one call per sensor scan (one `RawEvent`'s detections). Detections from one scan never update the same track twice.

**Per scan:**
1. Predict every live track to `t_us` non-mutatingly (`state_at`).
2. Build the cost matrix `d² = mahalanobis_sq(z, R_det)` with `S = H·P_pred·Hᵀ + R_det` (**R of the incoming detection included — G7**).
3. Gate at `CHI2_GATE`. Assign with `scipy.optimize.linear_sum_assignment` on gated pairs so each track gets ≤ 1 detection per scan.
4. Matched → `predict` (mutating) then `update(z, R_det)`. Unmatched detection → spawn a **tentative** track (velocity 0, std `V0_STD_MPS`). Unmatched track → untouched.
5. Evict any track with `t − t_last_update > EVICTION_GAP_S` (G11). No other eviction machinery, no scene-reset code.
6. Each returned `TrackSnapshot` carries `n_updates`, `first_t_us`, and `sensors_in_estimate` = sensors of the **last `MAX_MEMORY_UPDATES` updates only** (default 4, ASSUMED) — *not* a lifetime set.

**Rules**
- Uses only the shared `ConstantVelocityKF`; the tracker never inspects sensor identity except to fill `sensors_in_estimate`.
- Ignores `Detection.aux` entirely (radar Doppler is OFF — G17). Add a test proving results are identical with `aux` stripped.
- Class is recorded but **never used in gating** (DEC-2).
- Records a decision log per detection: `matched / spawned`, its `d²`, and the winning track. This log is what makes debugging possible; keep it structured (list of dataclasses).
- Also expose counters: matches, spawns, evictions, and the fraction of snapshots whose `sensors_in_estimate` has ≥ 2 sensors (to detect V1's dilution problem later).
- Track IDs exist only for internal bookkeeping; no persistence guarantees beyond the track's short life.

## 4. Do NOT
- No per-sensor trackers merged afterwards (what-not-to-do §1).
- No `MAX_MISSED_SECONDS`-style long timers, no scene-boundary reset, no lifetime `sensors_seen`.
- No fixed Euclidean gate; no gating on `P_pred` alone (§4).
- No appearance re-ID.
- No use of raw sensor Doppler or finite-differenced velocity (§3).
- Do not chase a `sigma_a` that avoids *all* over-merging across a scene (lessons D) — this is re-measured in step 11.

## 5. Synthetic tests (must all pass)
Build generators for a constant-velocity object observed by alternating sensors with different `R` (e.g., σ 0.1, 0.5, 1.5 m).
1. Single object, three alternating sensors → exactly one track; velocity converges to truth.
2. **S = P + R regression:** after a low-R update, a high-R detection of the same object is accepted. Include a parametrised variant that gates on `P_pred` only and demonstrably rejects it — proving the test can fail.
3. Two objects 10 m apart → two tracks.
4. **Crowd conflation (named open limitation):** two same-class objects ~0.8 m apart, moving similarly. Assert and *document* what happens (likely one merged track). Mark clearly as a known limitation in the test name and report; do not "fix" it (what-not-to-do §4, lessons C.3).
5. Track not updated for > `EVICTION_GAP_S` is evicted; a later detection creates a *new* track (new id).
6. One track updated at most once per scan even with two close detections in the same scan.
7. Ego-motion: a **stationary** object detected (in global frame) while the ego moves → track velocity ≈ 0.
8. `aux` stripped ⇒ identical outputs.
9. Determinism: same input → same output.
10. Measurement of velocity jitter on a stationary synthetic object → informs `MIN_TRUSTED_SPEED_MPS`, `MIN_CLOSING_SPEED_MPS` and `MAX_VEL_STD_MPS` (report numbers; these config values stay PLACEHOLDER until measured on real data).

## 6. Decisions to ask
DEC-5 (meaning of "2–4 updates of memory": present the interpretation in §3 and get confirmation).

## 7. Learning-note topics
What gating is and why Mahalanobis beats a fixed radius (use the P=0.09/R=0.64 example); what early fusion means and why V1's late fusion diluted; why short memory limits *how long* a bad match lives but does not prevent a bad first match.

## 8. STOP
Report with test output and a figure (true vs estimated trajectory for test 1) and wait for "approved".
