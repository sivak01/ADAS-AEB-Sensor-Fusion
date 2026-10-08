"""
tests/test_run_equivalence.py — step11_runs_tuning_report.md §3, §9 test 1:
the four stream runs (lidar/radar/camera/fused) must differ ONLY by which
adapter/channels are enabled -- everything else (SIGMA_A, DEBOUNCE_N, TTC
thresholds, corridor margins, ...) is the same code and the same config
values.
"""
from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from ttcf import config


def test_shared_hash_unaffected_by_per_sensor_params():
    """Mutating a per-sensor param (LIDAR_*/RADAR_*/CAMERA_*/SENSOR_R_*/
    GROUND_Z_EGO) must NOT change shared_config_hash() -- those are exactly
    the params allowed to differ between the four runs. Mutating a truly
    shared param (SIGMA_A) MUST change it -- otherwise the hash would be
    vacuously constant and prove nothing."""
    baseline = config.shared_config_hash()

    original = config._REGISTRY["LIDAR_CLUSTER_EPS_M"]
    config._REGISTRY["LIDAR_CLUSTER_EPS_M"] = replace(original, value=original.value + 100.0)
    try:
        assert config.shared_config_hash() == baseline
    finally:
        config._REGISTRY["LIDAR_CLUSTER_EPS_M"] = original

    original_ground_z = config._REGISTRY["GROUND_Z_EGO"]
    config._REGISTRY["GROUND_Z_EGO"] = replace(original_ground_z, value=original_ground_z.value + 1.0)
    try:
        assert config.shared_config_hash() == baseline
    finally:
        config._REGISTRY["GROUND_Z_EGO"] = original_ground_z

    original_sigma_a = config._REGISTRY["SIGMA_A"]
    config._REGISTRY["SIGMA_A"] = replace(original_sigma_a, value=original_sigma_a.value + 1.0)
    try:
        assert config.shared_config_hash() != baseline
    finally:
        config._REGISTRY["SIGMA_A"] = original_sigma_a

    # Restored exactly -- hash should be back to baseline.
    assert config.shared_config_hash() == baseline


def test_fused_stream_is_exactly_the_union_of_single_sensor_channels():
    from run_pipeline import _channel_adapter_dispatch

    lidar_channels = set(_channel_adapter_dispatch("lidar"))
    radar_channels = set(_channel_adapter_dispatch("radar"))
    camera_channels = set(_channel_adapter_dispatch("camera"))
    fused_channels = set(_channel_adapter_dispatch("fused"))

    assert fused_channels == lidar_channels | radar_channels | camera_channels
    # No stream partially overlaps another (DEC-3's own channel list has no
    # cross-sensor duplicates) -- a real union, not an accidental one.
    assert not (lidar_channels & radar_channels)
    assert not (lidar_channels & camera_channels)
    assert not (radar_channels & camera_channels)


def test_fused_dispatch_routes_each_channel_to_its_own_adapter():
    from run_pipeline import _channel_adapter_dispatch

    dispatch = _channel_adapter_dispatch("fused")
    assert dispatch["LIDAR_TOP"].__module__.endswith("adapters.lidar")
    assert dispatch["RADAR_FRONT"].__module__.endswith("adapters.radar")
    assert dispatch["RADAR_FRONT_LEFT"].__module__.endswith("adapters.radar")
    assert dispatch["RADAR_FRONT_RIGHT"].__module__.endswith("adapters.radar")
    assert dispatch["CAM_FRONT"].__module__.endswith("adapters.camera")


def test_current_config_hash_matches_freeze_output(tmp_path):
    """current_config_hash() (no file written) must equal what freeze()
    (which does write a file) computes for the identical live config --
    step11 §5.2's eval-time check depends on these being the same
    computation, not two independent ones that could silently drift apart."""
    live_hash = config.current_config_hash()
    frozen_hash = config.freeze(tmp_path / "scratch_frozen.json")
    assert live_hash == frozen_hash


def test_four_run_table_generator_produces_four_rows_and_footnotes(tmp_path):
    """step11 §9 test 4: the table generator's SUCCESS path -- all four
    runs present, both observability variants each -- produces exactly
    four rows and every mandatory footnote. (step10's own tests only
    covered the two FAILURE paths -- a missing footnote source, a missing
    run -- never this positive case.)"""
    from ttcf.evaluation.metrics import KeyframeGT, KeyframeSystem, score_run
    from ttcf.evaluation.report import build_footnotes, generate_four_run_table

    gt_keyframes = [KeyframeGT("kf1", False, ["NONE"], [{"obs_lidar": True, "obs_any": True}])]
    system = {"kf1": KeyframeSystem("kf1", "NONE")}
    runs = {}
    for name in ("lidar", "radar", "camera", "fused"):
        m_all = score_run(gt_keyframes, system, run_name=name, observability_variant="all", obs_field=None)
        m_obs = score_run(gt_keyframes, system, run_name=name, observability_variant="obs_lidar", obs_field="obs_lidar")
        runs[name] = {"all": m_all, "obs_lidar": m_obs}

    footnotes = build_footnotes(
        radar_doppler_disabled=True, sensor_r_status={"lidar": "ASSUMED", "radar": "ASSUMED", "camera": "ASSUMED"},
        indicative_rows=[], class_policy="symmetric, never gates", split_name="tune", tuning_split_only=True,
        distance_definition="nearest_surface_to_ego", n_gt_events=315, n_danger_events=11,
    )
    payload = generate_four_run_table(runs, footnotes, out_dir=tmp_path)
    assert len(payload["rows"]) == 8  # 4 runs x 2 variants each
    assert set(payload["footnotes"].keys()) == {
        "1_radar_doppler", "2_r_measured_vs_assumed", "3_indicative_only", "4_class_policy",
        "5_split", "6_distance_definition", "7_gt_events",
    }
    assert (tmp_path / "four_run_table.md").exists()
    assert (tmp_path / "four_run_table.csv").exists()
    assert (tmp_path / "four_run_table.json").exists()


def test_rerun_same_config_reproduces_identical_counts():
    """step11 §9 test 5 (determinism): re-running a tune-split run with the
    same config gives identical tracker counts and scores -- one real
    scene (lidar-only, the fastest real adapter) keeps this fast while
    still exercising the real dataset, real adapter, and real tracker."""
    import json

    from nuscenes.nuscenes import NuScenes
    from run_pipeline import run_real_stream, score_real_stream

    if not config._has_nuscenes_layout(config.DATAROOT):
        import pytest

        pytest.skip("real dataset not available in this environment")

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_token = split["tune"][2]["token"]  # scene-0655, the smallest tune scene -- keeps this test fast

    runs_a = run_real_stream(nusc, [scene_token], "lidar")
    runs_b = run_real_stream(nusc, [scene_token], "lidar")

    tracker_a, tracker_b = runs_a[scene_token]["tracker"], runs_b[scene_token]["tracker"]
    assert tracker_a.n_matches == tracker_b.n_matches
    assert tracker_a.n_spawns == tracker_b.n_spawns
    assert tracker_a.n_evictions == tracker_b.n_evictions

    metrics_a = score_real_stream(nusc, runs_a, "lidar")
    metrics_b = score_real_stream(nusc, runs_b, "lidar")
    assert metrics_a["all"].confusion == metrics_b["all"].confusion
