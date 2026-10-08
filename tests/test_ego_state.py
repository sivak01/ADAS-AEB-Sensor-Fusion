"""
tests/test_ego_state.py — step03_kf_ego_state.md §6, items 9-12 (real data).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.filtering.ego_state import EgoStateEstimator

OUT_DIR = Path("outputs/figs/step03")


@pytest.fixture(scope="module")
def nusc():
    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        pytest.skip(f"DATAROOT {config.DATAROOT} does not have the expected nuScenes layout")
    return NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)


def _lidar_timestamps_for_scene(nusc, scene_token):
    """Every LIDAR_TOP sample_data timestamp in this scene, native rate,
    sorted -- used as the sampling grid for the speed-vs-time plots."""
    ts = []
    for sd in nusc.sample_data:
        channel = nusc.get(
            "sensor", nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])["sensor_token"]
        )["channel"]
        if channel != "LIDAR_TOP":
            continue
        if nusc.get("sample", sd["sample_token"])["scene_token"] != scene_token:
            continue
        ts.append(int(sd["timestamp"]))
    return sorted(ts)


# ── Test 9: per-scene ego speed vs time (real data) ───────────────────────

def test_ego_speed_per_scene_is_smooth_and_plausible(nusc):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scenes = nusc.scene
    fig, axes = plt.subplots(2, 5, figsize=(22, 7), sharey=True)
    axes = axes.flatten()

    min_trusted = config.MIN_TRUSTED_SPEED_MPS.value
    any_stationary_checked = False

    for ax, scene in zip(axes, scenes):
        est = EgoStateEstimator(nusc, scene["token"])
        ts = _lidar_timestamps_for_scene(nusc, scene["token"])
        speeds = [est.state_at(t).speed_mps for t in ts]
        t0 = ts[0]
        t_s = [(t - t0) / 1e6 for t in ts]

        ax.plot(t_s, speeds, linewidth=1)
        ax.set_title(scene["name"], fontsize=9)
        ax.set_xlabel("s")

        # Urban plausibility: nuScenes-mini is city driving, not highway.
        assert max(speeds) < 25.0, f"{scene['name']}: implausible top speed {max(speeds):.1f} m/s"

        if min(speeds) < min_trusted:
            any_stationary_checked = True
            near_zero = [s for s in speeds if s < min_trusted]
            print(
                f"\n{scene['name']}: near-stationary segment found, "
                f"{len(near_zero)}/{len(speeds)} samples below "
                f"MIN_TRUSTED_SPEED_MPS={min_trusted} (max jitter there: "
                f"{max(near_zero):.3f} m/s)"
            )

    axes[0].set_ylabel("speed (m/s)")
    fig.suptitle("Ego speed vs time, all 10 scenes (native-rate LIDAR_TOP sampling grid)")
    fig.tight_layout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_DIR / "ego_speed_all_scenes.png", dpi=130)
    plt.close(fig)

    assert any_stationary_checked, "Expected at least one scene with a near-stationary segment"


# ── Test 10: causality -- output at t is identical if future poses removed ─

def test_causality_future_poses_do_not_affect_past_state(nusc):
    scene = nusc.scene[0]
    est_full = EgoStateEstimator(nusc, scene["token"])
    mid_idx = len(est_full._poses) // 2
    t_mid = est_full._poses[mid_idx][0]

    result_full = est_full.state_at(t_mid)

    # Simulate "future poses removed": a second estimator whose pose list
    # is truncated to timestamp <= t_mid, bypassing __init__'s dataset
    # query (already have the poses) -- valid since state_at() only ever
    # reads self._poses.
    est_truncated = EgoStateEstimator.__new__(EgoStateEstimator)
    est_truncated._nusc = nusc
    est_truncated._scene_token = scene["token"]
    est_truncated._poses = [p for p in est_full._poses if p[0] <= t_mid]
    est_truncated._pose_times = [t for t, _ in est_truncated._poses]
    est_truncated._cache_kf = None
    est_truncated._cache_progressed_idx = 0
    est_truncated._cache_latest_rotation = None

    result_truncated = est_truncated.state_at(t_mid)

    assert result_full.t_us == result_truncated.t_us
    np.testing.assert_allclose(
        [result_full.x, result_full.y, result_full.vx, result_full.vy],
        [result_truncated.x, result_truncated.y, result_truncated.vx, result_truncated.vy],
    )


# ── Test 11: lag vs. a smoothed reference during accel/braking (informational) ─

def test_kf_lag_vs_smoothed_reference_informational(nusc):
    """Informational only, per step03 §6 item 11 -- no pass/fail on the
    lag value itself, just a sanity bound and a printed report. The
    'smoothed reference' here is a raw finite-difference speed used ONLY
    as an external comparison signal for this diagnostic -- never fed
    into the actual pipeline (what-not-to-do.md #3 is about velocity
    used BY the pipeline, not an offline check on a report)."""
    # Pick the scene with the largest speed range as the most likely to
    # contain a real accel/braking segment.
    best_scene, best_range = None, -1.0
    for scene in nusc.scene:
        est = EgoStateEstimator(nusc, scene["token"])
        ts = _lidar_timestamps_for_scene(nusc, scene["token"])
        speeds = [est.state_at(t).speed_mps for t in ts]
        rng = max(speeds) - min(speeds)
        if rng > best_range:
            best_range, best_scene = rng, scene

    est = EgoStateEstimator(nusc, best_scene["token"])
    ts = _lidar_timestamps_for_scene(nusc, best_scene["token"])
    kf_speed = np.array([est.state_at(t).speed_mps for t in ts])
    t_s = np.array([(t - ts[0]) / 1e6 for t in ts])

    positions = np.array([[est.state_at(t).x, est.state_at(t).y] for t in ts])
    raw_diff_speed = np.zeros(len(ts))
    dt = np.diff(t_s)
    raw_diff_speed[1:] = np.linalg.norm(np.diff(positions, axis=0), axis=1) / dt
    # Light smoothing (3-point moving average) -- purely a reference
    # signal for this diagnostic, not a filter design choice.
    kernel = np.ones(3) / 3
    smoothed_ref = np.convolve(raw_diff_speed, kernel, mode="same")

    # Cross-correlation peak -> lag estimate (positive = KF lags reference).
    corr = np.correlate(kf_speed - kf_speed.mean(), smoothed_ref - smoothed_ref.mean(), mode="full")
    lag_idx = corr.argmax() - (len(smoothed_ref) - 1)
    median_dt = float(np.median(dt))
    lag_s = lag_idx * median_dt

    print(
        f"\nLag report ({best_scene['name']}, speed range {best_range:.1f} m/s): "
        f"KF vs. finite-difference-reference lag ~= {lag_s:.3f} s "
        f"(positive = KF lags the reference)"
    )
    assert abs(lag_s) < 1.0, "Lag is implausibly large -- check timestamp handling"


# ── Test 12: CAN-bus expansion check (real data / state-of-fact) ─────────

def test_can_bus_expansion_presence(nusc):
    can_bus_dir = config.DATAROOT.parent / "can_bus"
    candidates = [config.DATAROOT / "can_bus", can_bus_dir]
    found = [p for p in candidates if p.exists()]
    if found:
        print(f"\nCAN-bus expansion FOUND at {found[0]} -- an independent speed check is possible.")
    else:
        print(
            "\nCAN-bus expansion NOT found (checked "
            + ", ".join(str(p) for p in candidates)
            + "). No independent vehicle-speed ground truth exists for this dataset; "
            "the ego-speed checks above rely on ego_pose-derived KF velocity only."
        )
    assert True  # this test documents a fact; it does not gate on the outcome
