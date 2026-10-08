"""
tests/test_config.py — step00 §7.

Covers: every Param has a status and non-empty reason; no MEASURED status
exists yet; freeze() hash changes when a value changes and is stable
otherwise; describe() renders every parameter; t_us fields are ints.
"""
import json

import numpy as np
import pytest

from ttcf import config, types


def test_every_param_has_status_and_nonempty_reason():
    for name, param in config._REGISTRY.items():
        assert param.status in ("MEASURED", "ASSUMED", "PLACEHOLDER"), name
        assert param.reason and param.reason.strip(), f"{name} has an empty reason"


def test_no_measured_status_exists_yet():
    measured = [name for name, p in config._REGISTRY.items() if p.status == "MEASURED"]
    assert measured == [], f"These params are already MEASURED, expected none yet: {measured}"


def test_describe_renders_every_parameter():
    text = config.describe()
    for name in config._REGISTRY:
        assert f"`{name}`" in text


def test_freeze_hash_stable_when_nothing_changes(tmp_path):
    path_a = tmp_path / "frozen_a.json"
    path_b = tmp_path / "frozen_b.json"
    digest_a = config.freeze(path_a)
    digest_b = config.freeze(path_b)
    assert digest_a == digest_b


def test_freeze_hash_changes_when_a_value_changes(tmp_path):
    path_before = tmp_path / "before.json"
    digest_before = config.freeze(path_before)

    original = config._REGISTRY["SIGMA_A"]
    try:
        config._REGISTRY["SIGMA_A"] = config.Param(
            value=original.value + 1.0,
            status=original.status,
            reason=original.reason,
            tunable_on=original.tunable_on,
        )
        path_after = tmp_path / "after.json"
        digest_after = config.freeze(path_after)
        assert digest_after != digest_before
    finally:
        config._REGISTRY["SIGMA_A"] = original


def test_freeze_writes_valid_json_with_hash(tmp_path):
    path = tmp_path / "frozen.json"
    digest = config.freeze(path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["sha256"] == digest
    assert "SIGMA_A" in payload["params"]


def test_dataroot_and_active_channels_are_sane():
    assert isinstance(config.DATAROOT, object)  # a Path; exists-check belongs to preflight, not here
    assert config.ACTIVE_CHANNELS.value == [
        "LIDAR_TOP",
        "RADAR_FRONT",
        "RADAR_FRONT_LEFT",
        "RADAR_FRONT_RIGHT",
        "CAM_FRONT",
    ]


# ── t_us fields are ints (dataclass type check) ──────────────────────────

def test_raw_event_t_us_must_be_int():
    types.RawEvent(
        t_us=1533151603512404,
        channel="LIDAR_TOP",
        modality="lidar",
        sample_data_token="tok",
        ego_pose_token="tok",
        calibrated_sensor_token="tok",
        filename="x.pcd.bin",
        is_key_frame=True,
        scene_token="tok",
    )
    with pytest.raises(TypeError):
        types.RawEvent(
            t_us=1533151603512404.0,
            channel="LIDAR_TOP",
            modality="lidar",
            sample_data_token="tok",
            ego_pose_token="tok",
            calibrated_sensor_token="tok",
            filename="x.pcd.bin",
            is_key_frame=True,
            scene_token="tok",
        )


def test_detection_t_us_must_be_int():
    types.Detection(
        t_us=1533151603512404,
        channel="RADAR_FRONT",
        modality="radar",
        xy_global=np.zeros(2),
        R_global=np.eye(2),
        ref_point_kind="nearest_surface_to_ego",
    )
    with pytest.raises(TypeError):
        types.Detection(
            t_us="1533151603512404",
            channel="RADAR_FRONT",
            modality="radar",
            xy_global=np.zeros(2),
            R_global=np.eye(2),
            ref_point_kind="nearest_surface_to_ego",
        )


def test_track_snapshot_t_us_must_be_int():
    types.TrackSnapshot(
        track_id="t1",
        t_us=1533151603512404,
        x=np.zeros(4),
        P=np.eye(4),
        n_updates=2,
        sensors_in_estimate=frozenset({"LIDAR_TOP"}),
        first_t_us=1533151603000000,
    )
    with pytest.raises(TypeError):
        types.TrackSnapshot(
            track_id="t1",
            t_us=1533151603512404,
            x=np.zeros(4),
            P=np.eye(4),
            n_updates=2,
            sensors_in_estimate=frozenset({"LIDAR_TOP"}),
            first_t_us=1533151603000000.0,
        )


def test_ttc_result_t_us_must_be_int():
    types.TTCResult(
        t_us=1533151603512404,
        track_id="t1",
        distance_m=10.0,
        closing_speed_mps=2.0,
        ttc_s=5.0,
        reason=types.TTCReason.OK,
        closing_std_mps=0.1,
    )
    with pytest.raises(TypeError):
        types.TTCResult(
            t_us=None,
            track_id="t1",
            distance_m=10.0,
            closing_speed_mps=2.0,
            ttc_s=5.0,
            reason=types.TTCReason.OK,
            closing_std_mps=0.1,
        )


def test_action_decision_t_us_must_be_int():
    types.ActionDecision(t_us=1533151603512404, tier=types.ActionTier.NONE, driving_track_id=None, consecutive_count=0)
    with pytest.raises(TypeError):
        types.ActionDecision(t_us=1.5, tier=types.ActionTier.NONE, driving_track_id=None, consecutive_count=0)
