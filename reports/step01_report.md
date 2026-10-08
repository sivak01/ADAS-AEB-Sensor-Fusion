# Step 01 report

## What was built (files, one line each)
- `src/ttcf/data/event_stream.py` — `scene_channel_events`, `merge_channel_events`, `iter_scene_events`, `keyframe_only`, `resolve_path`.
- `tests/test_event_stream.py` — 6 tests (step01 §5).
- `scripts/event_stream_evidence.py` — generates the §6 real-data evidence (raster plot, inter-arrival table, keyframe offsets).
- `outputs/figs/step01/raster_1s.png`, `outputs/tables/step01_interarrival.csv`.

## Decisions taken (DEC ids + Siva's answers)
None — DEC-3 (active channels) already settled at step00, consumed here.

## Precedent check (V1/v2), ported vs. rebuilt, with why
- **Ported**: `heapq.merge(*streams, key=...)` for combining already-sorted per-channel streams into one globally chronological stream — v2's `event_stream.py` already had this exactly right (lazy, O(N log k), never re-sorts a materialized list). Same technique, used identically here.
- **Not ported**: v2's `SensorEvent` type. It's already a fully-loaded detection (global-frame x/y, resolved sensor identity) because v2's adapters ran *before* the stream merge. This step wants the opposite — a lazy `RawEvent` (tokens/metadata only) merged *before* any file loading, since adapters sit near the very end of this project's build order (steps 14-16), well after the tracker/TTC/debounce machinery is proven on synthetic data. Built `RawEvent`-based streaming fresh.
- **No precedent found anywhere in V1/v2** for walking a `sample_data.next` chain through sweeps bounded to one scene — both stayed at 2Hz keyframes throughout (`for sample in nusc.sample`, no chain-walking at all). Built entirely fresh.

## Tests (command + pass/fail output)
Command: `.venv\Scripts\python.exe -m pytest tests\ -q`
```
56 passed, 14 warnings in 43.37s
```
All 6 of this step's tests pass, alongside the 50 from prior steps. No debugging narrative for this step — all 6 passed on the first implementation.

## Real-data evidence for Siva (step01 §6)
- **Raster plot** (`outputs/figs/step01/raster_1s.png`): event arrivals over the first 1 second of `scene-0061`, one row per channel. Clearly shows the interleaving this design exists for — LIDAR_TOP's ticks are visibly denser than the radars', which are denser than CAM_FRONT's, with no two channels ever landing on a shared tick.
- **Inter-arrival table** (`outputs/tables/step01_interarrival.csv`):

| channel | n_events | mean inter-arrival (ms) | std (ms) | implied Hz |
|---|---|---|---|---|
| LIDAR_TOP | 382 | 50.26 | 3.65 | 19.90 |
| RADAR_FRONT | 247 | 77.65 | 12.63 | 12.88 |
| RADAR_FRONT_LEFT | 262 | 73.38 | 6.77 | 13.63 |
| RADAR_FRONT_RIGHT | 258 | 74.46 | 11.22 | 13.43 |
| CAM_FRONT | 224 | 85.87 | 27.46 | 11.64 |

Matches step00's preflight measurements closely (LiDAR ≈20Hz, radars ≈13.3-13.6Hz). CAM_FRONT's implied rate here (11.64 Hz) sits between step00's overall-dataset median (10.00 Hz) and the commonly-cited ~12 Hz nominal figure — consistent with step00's own finding that camera timing has real per-scene jitter, not a single fixed rate; this is the same phenomenon from a different angle, not a new discrepancy.
- **Keyframe ms-offsets** (one sample, `scene-0061`, relative to LIDAR_TOP): RADAR_FRONT +16.2 ms, RADAR_FRONT_LEFT +4.7 ms, RADAR_FRONT_RIGHT −8.1 ms, CAM_FRONT −35.5 ms — confirms every channel fires at its own real instant within one nominal "sample," exactly the effect G2 exists to protect against conflating.

## Surprising or suspicious numbers (treat as bugs first)
None new this step. The CAM_FRONT rate variation noted above was already investigated and accepted at step00; no fresh investigation needed here beyond confirming it reproduces consistently at the per-scene, per-channel level.

## Assumptions introduced (each must be in config with a status)
None. This step introduces no new tunables; it only consumes `ACTIVE_CHANNELS` and `DATAROOT`, both already in `config.py`.

## Flagged, not built (out-of-scope temptations)
- No transmission-latency modeling — explicitly out of scope per step01 §4, noted here as future work rather than silently added or silently ignored.
- No channel synchronization — deliberately not built; the whole point of this stream is to preserve each channel's own native-rate timing, not collapse it onto a shared tick.

## Open questions for Siva
None.

Waiting for "approved" before proceeding to `step06_tracker.md` (build order #8).
