# STEP 08 — Debounce and action tiers

**Build order #11.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 06, 07 (A+B), 05 (A+B) approved.
**Scope: synthetic streams only.**

## 1. Purpose (plain language)
A single frame's TTC can be wrong (one bad match, one noisy return). Real AEB systems therefore require the danger condition to hold for N consecutive real updates before acting (G16). This is the cheap, standard defence against one-off errors, used *instead of* long-horizon identity robustness.

## 2. Deliverables
- `src/ttcf/ttc/action.py`
- `tests/test_action.py`
- figures in `outputs/figs/step08/`, report + learning note

## 3. Specification

Input: the stream of **critical-object** `TTCResult`s (from step 07 Part B), one per *real update* of the critical track. Output: a timeline of `ActionDecision(t_us, tier, driving_track_id, consecutive_count)`.

**Definitions**
- Tiers: `NONE`, `GRADUAL` (TTC ≤ `TTC_GRADUAL_S`), `AEB` (TTC ≤ `TTC_AEB_S`). All thresholds from config; ASSUMED; tunable on the tuning split only.
- A **real update** is a detection that actually updated the critical track. Prediction-only ticks do **not** count towards N.
- Danger counter per tier: increments when a real update of the **same** critical track satisfies the tier condition; resets to 0 when the condition fails, when the critical track id changes, or when the track is evicted.
- The tier fires only when its counter reaches `DEBOUNCE_N`; the highest satisfied tier wins.
- **Release** is symmetric: a tier drops after `DEBOUNCE_N` consecutive real updates where its condition is false (avoids chatter). State this rule in the report; propose alternatives without building them.
- `NOT_ELIGIBLE` results never trigger an action and never count as "safe" evidence either (they neither increment nor reset the counter; they are recorded as `NO_ESTIMATE`).
- When the driving track is evicted (no update within `EVICTION_GAP_S`), the action state falls back to `NONE` and the log records `NO_ESTIMATE` so evaluation can attribute later misses to coverage rather than to logic.
- Expose `action_at(t_us)` (latest decision with timestamp ≤ `t_us`, honouring the eviction rule) — step 09 depends on this.
- Track **latency**: number of real updates and milliseconds between the first danger-condition update and the action firing.

## 4. Do NOT
- Do not act on a single frame (what-not-to-do §5).
- Do not debounce across *different* tracks (resetting on id change is deliberate and conservative).
- Do not count prediction-only ticks.
- Do not tune `DEBOUNCE_N` or thresholds on eval scenes.
- Do not add steering logic (no GT to score it against in v1); list as "Flagged, not built".

## 5. Tests (synthetic streams)
1. **One bad frame:** a single TTC dip below `TTC_AEB_S` amid safe values → no action (this is the required "bad match must not trigger" test).
2. `DEBOUNCE_N` consecutive dangerous updates → tier fires at exactly the N-th update, not before.
3. Interleaved critical-track id change resets the counter.
4. Release requires N consecutive safe updates.
5. `AEB` supersedes `GRADUAL` when both hold.
6. `NOT_ELIGIBLE` results neither trigger nor reset.
7. Eviction returns to `NONE` with a `NO_ESTIMATE` record.
8. Latency figures equal N × inter-arrival for a regular synthetic stream.
9. `action_at(t)` returns the correct decision for times between updates.

## 6. Decisions to ask
DEC-9 (confirm `DEBOUNCE_N`, thresholds), DEC-10 (point estimate vs conservative bound — applies here if the conservative option is chosen).

## 7. Learning-note topics
Debounce as a cheap substitute for identity robustness; the trade-off between false-brake protection and added reaction delay (worked example: N = 3 at 20 Hz adds ≈ 0.1 s, which at 15 m/s closing is 1.5 m of travel); why release is symmetric.

## 8. STOP
Report with a timeline figure (TTC trace + action decisions) and wait for "approved".
