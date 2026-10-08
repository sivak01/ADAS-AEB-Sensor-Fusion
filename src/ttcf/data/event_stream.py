"""
src/ttcf/data/event_stream.py — STEP 01: native-rate, multi-channel event
stream (G3).

The pipeline should react to whichever sensor has new information RIGHT
NOW, at that sensor's real rate, not wait for a synchronised 2Hz tick.
Reads no point clouds or images -- only records and tokens (RawEvent is
lazy; adapters load files later, near the end of the build order).

Ported from the predecessor project's v2/event_stream.py: the
heapq.merge(*streams, key=...) technique for combining multiple already-
sorted per-channel streams into one globally chronological stream --
lazy, O(N log k), never re-sorts a materialised list. NOT ported: v2's
SensorEvent is already a fully-loaded detection (global-frame x/y, sensor
identity resolved) because v2's adapters ran BEFORE the stream merge;
this step wants the opposite (RawEvent, metadata only, merged BEFORE any
file loading) -- built fresh. No precedent exists anywhere in V1/v2 for
walking a sample_data.next chain through sweeps bounded to one scene
either (both stayed at 2Hz keyframes throughout) -- also built fresh.
"""
from __future__ import annotations

import heapq
from pathlib import Path
from typing import Iterable, Iterator, Optional

from nuscenes.nuscenes import NuScenes

from ttcf import config
from ttcf.types import RawEvent


def _channel_and_modality(nusc: NuScenes, sd: dict) -> tuple[str, str]:
    sensor = nusc.get("sensor", nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])["sensor_token"])
    return sensor["channel"], sensor["modality"]


def _raw_event_from_sample_data(nusc: NuScenes, sd: dict, scene_token: str) -> RawEvent:
    channel, modality = _channel_and_modality(nusc, sd)
    return RawEvent(
        t_us=int(sd["timestamp"]),  # from the JSON record, never the filename
        channel=channel,
        modality=modality,
        sample_data_token=sd["token"],
        ego_pose_token=sd["ego_pose_token"],
        calibrated_sensor_token=sd["calibrated_sensor_token"],
        filename=sd["filename"],
        is_key_frame=bool(sd["is_key_frame"]),
        scene_token=scene_token,
    )


def scene_channel_events(nusc: NuScenes, scene_token: str, channel: str) -> Iterator[RawEvent]:
    """Every sample_data record (keyframes + sweeps) for ONE channel,
    within one scene, in time order -- from the scene's first keyframe's
    sample_data through its last keyframe's sample_data (both inclusive),
    walking .next chains. Start/end are the scene's own first/last
    keyframe records, so nothing outside the scene is included. Raises a
    clear error on a non-monotonic timestamp or an unexpectedly broken
    chain, rather than silently truncating or reordering."""
    scene = nusc.get("scene", scene_token)
    first_sample = nusc.get("sample", scene["first_sample_token"])
    last_sample = nusc.get("sample", scene["last_sample_token"])

    if channel not in first_sample["data"]:
        return

    end_token = last_sample["data"][channel]
    node = nusc.get("sample_data", first_sample["data"][channel])

    prev_t = None
    while True:
        t_us = int(node["timestamp"])
        if prev_t is not None and t_us < prev_t:
            raise ValueError(
                f"Non-monotonic timestamp on channel {channel} in scene {scene_token}: "
                f"{t_us} < {prev_t} at sample_data {node['token']}"
            )
        prev_t = t_us
        yield _raw_event_from_sample_data(nusc, node, scene_token)

        if node["token"] == end_token:
            return
        if not node["next"]:
            raise ValueError(
                f"sample_data chain for channel {channel} in scene {scene_token} ended "
                f"before reaching the scene's last keyframe (expected token {end_token})"
            )
        node = nusc.get("sample_data", node["next"])


def merge_channel_events(*event_iterables: Iterable[RawEvent]) -> Iterator[RawEvent]:
    """Merge any number of per-channel RawEvent iterables (each already
    time-ordered) into one globally chronological stream. Deterministic
    tie-break: (t_us, channel)."""
    return heapq.merge(*event_iterables, key=lambda e: (e.t_us, e.channel))


def iter_scene_events(
    nusc: NuScenes, scene_token: str, channels: Optional[list[str]] = None
) -> Iterator[RawEvent]:
    """The main public generator: every ACTIVE_CHANNELS event (or a given
    channel list) in one scene, native rate, globally time-ordered."""
    channels = channels if channels is not None else config.ACTIVE_CHANNELS.value
    per_channel = [scene_channel_events(nusc, scene_token, ch) for ch in channels]
    return merge_channel_events(*per_channel)


def keyframe_only(events: Iterable[RawEvent]) -> Iterator[RawEvent]:
    """Filter to just the keyframe (is_key_frame=True) events -- needed
    later to line events up with ground truth, which only exists at
    keyframe instants."""
    return (e for e in events if e.is_key_frame)


def resolve_path(event: RawEvent) -> Path:
    """DATAROOT / event.filename. nuScenes JSON always uses forward
    slashes; pathlib resolves this correctly on any OS, including
    Windows."""
    return config.DATAROOT / event.filename
