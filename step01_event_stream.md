# STEP 01 — Native-rate, multi-channel event stream

**Build order #7.** Follow `README_MASTER.md` §4 and §8. Prerequisites: steps 00, 02 (and D0) approved.

## 1. Purpose (plain language)
The pipeline should react to whichever sensor has new information *right now*, at that sensor's real rate, not wait for a synchronised 2 Hz tick (G3). This step turns the dataset into one time-ordered stream of lightweight events. It reads **no point clouds or images** — only records and tokens.

## 2. Deliverables
- `src/ttcf/data/event_stream.py`
- `tests/test_event_stream.py`
- `outputs/figs/step01/` (rate plots), report + learning note

## 3. Specification
- For a scene and a channel, find the first keyframe `sample_data` (via `scene.first_sample_token` → `sample.data[channel]`) and walk `sample_data.next` **through sweeps** until the last keyframe's `sample_data` of that scene. Start and end are the scene's first and last keyframe records, so nothing outside the scene is included (state this).
- Emit `RawEvent` (see step 00 types) for each record, with **that record's own** `timestamp`, `ego_pose_token`, `calibrated_sensor_token`, `filename`, `is_key_frame` (G2). Timestamps come from the JSON record, never from the filename.
- Merge all `ACTIVE_CHANNELS` (config, DEC-3) into a single generator ordered by `t_us`, using `heapq.merge`. Deterministic tie-break: `(t_us, channel_name)`.
- Verify per-channel monotonic timestamps; raise a clear error if violated.
- Resolve file paths with `pathlib` from `DATAROOT` + `filename` (forward slashes inside the JSON; Windows-safe).
- Provide `iter_scene_events(scene_token)` and a helper listing which events are keyframes (needed later to line up with GT).
- **Lazy:** events hold no sensor data. Adapters load files.

## 4. Do NOT
- Do not read only `samples/` (what-not-to-do §2).
- Do not reuse one channel's timestamp/pose for another (G2).
- Do not synchronise channels onto a shared frame.
- Do not model transmission latency (out of scope; mention as future work).

## 5. Tests
1. Merged stream is globally non-decreasing in `t_us`.
2. Per-channel event count ≈ duration × native rate (report per channel per scene vs the Hz found in step 00).
3. Every event's `ego_pose_token`/`timestamp` come from its own record (compare against a direct `nusc.get('sample_data', tok)`).
4. Keyframe events are exactly the scene's annotated samples for that channel.
5. Filename timestamp is never used (grep test for parsing of filenames).
6. Two-scene run: no event from scene A appears in scene B's stream.

## 6. Real-data evidence for Siva
- A plot of event arrivals over 1 second for each channel (a raster/timeline showing the interleaving of ~20 Hz LiDAR, ~13 Hz radars, ~12 Hz camera).
- A table of inter-arrival mean/std per channel.
- Within one keyframe sample, the millisecond offsets between LiDAR, radar, and camera timestamps.

## 7. Decisions to ask
DEC-3 (active channels) if still open.

## 8. Learning-note topics
Samples vs sweeps; why asynchronous event processing gives fresher TTC than a synchronous frame; what the interleaving plot shows.

## 9. STOP
Report and wait for "approved".
