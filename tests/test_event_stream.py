"""
tests/test_event_stream.py — step01_event_stream.md §5.
"""
from __future__ import annotations

import inspect
import re
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.data import event_stream
from ttcf.data.event_stream import iter_scene_events, keyframe_only, scene_channel_events


@pytest.fixture(scope="module")
def nusc():
    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        pytest.skip(f"DATAROOT {config.DATAROOT} does not have the expected nuScenes layout")
    return NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)


# ── Test 1: merged stream is globally non-decreasing in t_us ─────────────

def test_merged_stream_globally_non_decreasing(nusc):
    scene = nusc.scene[0]
    events = list(iter_scene_events(nusc, scene["token"]))
    assert len(events) > 0
    t_values = [e.t_us for e in events]
    assert all(t_values[i] <= t_values[i + 1] for i in range(len(t_values) - 1))


# ── Test 2: per-channel event count ~= duration x native rate ────────────

def test_per_channel_count_matches_native_rate(nusc):
    scene = nusc.scene[0]
    channels = config.ACTIVE_CHANNELS.value

    # Scene duration from its first/last keyframe timestamps.
    first_sample = nusc.get("sample", scene["first_sample_token"])
    last_sample = nusc.get("sample", scene["last_sample_token"])
    duration_s = (last_sample["timestamp"] - first_sample["timestamp"]) / 1e6
    assert duration_s > 0

    print(f"\nscene {scene['name']}, duration {duration_s:.2f}s")
    for ch in channels:
        events = list(scene_channel_events(nusc, scene["token"], ch))
        n = len(events)
        implied_hz = n / duration_s
        print(f"  {ch:20s} n={n:5d}  implied_hz={implied_hz:6.2f}")
        # Loose sanity bound -- native rates measured in step00 ranged
        # ~10-20 Hz; this just confirms event count is in a plausible
        # ballpark, not an exact match (duration excludes the tail gap
        # after the last keyframe's own sweep interval).
        assert 5.0 < implied_hz < 30.0


# ── Test 3: every event's fields come from its OWN record ────────────────

def test_event_fields_come_from_own_record(nusc):
    scene = nusc.scene[0]
    channel = "RADAR_FRONT"
    events = list(scene_channel_events(nusc, scene["token"], channel))
    assert len(events) > 0

    rng = np.random.default_rng(0)
    sample_idx = rng.choice(len(events), size=min(15, len(events)), replace=False)
    for i in sample_idx:
        event = events[i]
        sd = nusc.get("sample_data", event.sample_data_token)
        assert event.t_us == int(sd["timestamp"])
        assert event.ego_pose_token == sd["ego_pose_token"]
        assert event.calibrated_sensor_token == sd["calibrated_sensor_token"]
        assert event.filename == sd["filename"]
        assert event.is_key_frame == bool(sd["is_key_frame"])


# ── Test 4: keyframe events are exactly the scene's annotated samples ────

def test_keyframe_events_match_scene_samples(nusc):
    scene = nusc.scene[0]
    channel = "LIDAR_TOP"
    events = list(scene_channel_events(nusc, scene["token"], channel))
    kf_events = list(keyframe_only(events))

    expected_tokens = set()
    sample_token = scene["first_sample_token"]
    while sample_token:
        sample = nusc.get("sample", sample_token)
        expected_tokens.add(sample["data"][channel])
        sample_token = sample["next"]

    got_tokens = {e.sample_data_token for e in kf_events}
    assert got_tokens == expected_tokens
    assert len(kf_events) == scene["nbr_samples"]


# ── Test 5: filename timestamp is never used (grep-style source check) ───

def test_filename_never_parsed_for_timestamp():
    source = inspect.getsource(event_stream)
    # Every reference to "filename" must be a plain passthrough (dict
    # lookup or attribute), never combined with string-parsing/splitting/
    # regex machinery that would suggest extracting a timestamp from it.
    forbidden_near_filename = re.findall(r".{0,25}filename.{0,25}", source)
    suspicious_ops = ["split(", "re.", "int(", "float(", "search(", "match("]
    for snippet in forbidden_near_filename:
        for op in suspicious_ops:
            assert op not in snippet, f"suspicious filename parsing found: {snippet!r}"
    # And confirm timestamps are explicitly sourced from the JSON record.
    assert 'int(sd["timestamp"])' in source


# ── Test 6: two-scene run -- no event from scene A appears in scene B ────

def test_no_cross_scene_leakage(nusc):
    scene_a, scene_b = nusc.scene[0], nusc.scene[1]
    events_a = list(iter_scene_events(nusc, scene_a["token"]))
    events_b = list(iter_scene_events(nusc, scene_b["token"]))

    tokens_a = {e.sample_data_token for e in events_a}
    tokens_b = {e.sample_data_token for e in events_b}
    assert tokens_a.isdisjoint(tokens_b)
    assert all(e.scene_token == scene_a["token"] for e in events_a)
    assert all(e.scene_token == scene_b["token"] for e in events_b)
