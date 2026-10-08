"""
tests/test_transforms.py — step02_transform.md §5.

Tests 1-4 are synthetic (hand-computed / analytic). Tests 5-6 run against the
real dataset and are the ones that actually demonstrate the bug this utility
exists to prevent (G1, G2) — see step02 §5 items 5 and 6.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from pyquaternion import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.geometry.transforms import (
    FrameContext,
    ego_pose_timestamp,
    ego_to_global,
    ego_to_global_vector,
    ego_yaw,
    global_to_ego,
    global_to_ego_vector,
    global_to_sensor,
    rotate_cov_ego_to_global,
    sensor_to_ego,
    sensor_to_ego_vector,
    sensor_to_global,
    sensor_to_global_vector,
)
from ttcf.types import RawEvent

IDENTITY_POSE = {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]}


def _ctx(cs_translation, cs_rotation, ep_translation, ep_rotation) -> FrameContext:
    return FrameContext(
        calibrated_sensor={"translation": cs_translation, "rotation": cs_rotation},
        ego_pose={"translation": ep_translation, "rotation": ep_rotation, "timestamp": 0},
    )


# ── Test 1: identity pose -> points unchanged ────────────────────────────

def test_identity_pose_leaves_points_unchanged():
    ctx = _ctx([0, 0, 0], [1, 0, 0, 0], [0, 0, 0], [1, 0, 0, 0])
    points = np.array([[1.0, 2.0, 3.0], [-4.0, 5.0, -6.0]])
    np.testing.assert_allclose(sensor_to_global(points, ctx), points, atol=1e-12)
    np.testing.assert_allclose(sensor_to_ego(points, ctx), points, atol=1e-12)


# ── Test 2: round trip sensor -> global -> sensor recovers points ────────

def test_round_trip_sensor_global_sensor():
    rng = np.random.default_rng(0)
    q_cs = Quaternion(axis=[0, 0, 1], angle=0.37).elements.tolist()
    q_ep = Quaternion(axis=[1, 2, 3], angle=1.1).elements.tolist()
    ctx = _ctx([1.5, -2.0, 0.3], q_cs, [100.0, 200.0, 0.0], q_ep)
    points = rng.uniform(-10, 10, size=(15, 3))

    glob = sensor_to_global(points, ctx)
    back = global_to_sensor(glob, ctx)
    np.testing.assert_allclose(back, points, atol=1e-9)


# ── Test 3: known 90 degree yaw + translation, computed by hand ──────────

def test_known_90deg_yaw_translation_by_hand():
    # Sensor == ego (identity calibration). Ego at (10, 5, 0), yawed +90 deg
    # about z. A point at ego-frame (1, 0, 0):
    # Rz(90) = [[cos90, -sin90], [sin90, cos90]] = [[0, -1], [1, 0]]
    # Rz(90) @ (1, 0) = (0, 1)  ->  global = (10 + 0, 5 + 1, 0) = (10, 6, 0)
    q_ep = Quaternion(axis=[0, 0, 1], angle=np.pi / 2).elements.tolist()
    ctx = _ctx([0, 0, 0], [1, 0, 0, 0], [10.0, 5.0, 0.0], q_ep)

    point = np.array([[1.0, 0.0, 0.0]])
    result = ego_to_global(point, ctx)
    np.testing.assert_allclose(result, [[10.0, 6.0, 0.0]], atol=1e-9)

    # And the inverse must recover it.
    back = global_to_ego(result, ctx)
    np.testing.assert_allclose(back, point, atol=1e-9)


# ── Test 4: covariance rotation swaps sigma_long^2 / sigma_lat^2 at 90deg ─

def test_covariance_rotation_90deg_swaps_axes():
    sigma_long, sigma_lat = 2.0, 1.0
    R_ego = np.diag([sigma_long**2, sigma_lat**2])  # diag(4, 1)
    R_global = rotate_cov_ego_to_global(R_ego, yaw=np.pi / 2)
    # By hand: Rz(90) @ diag(4,1) @ Rz(90)^T = diag(1, 4) -- axes swapped.
    np.testing.assert_allclose(R_global, np.diag([sigma_lat**2, sigma_long**2]), atol=1e-9)


# ── Test 5: vector transform rotates but never translates (G17 ablation support) ─

def test_vector_transform_rotates_not_translates():
    # Same 90-degree ego yaw as test 3, but now ego ALSO sits far from the
    # origin (10, 5, 0) -- a POINT would pick up that offset; a VECTOR
    # (e.g. a Doppler velocity) must not, since a direction has no
    # position of its own to translate.
    q_ep = Quaternion(axis=[0, 0, 1], angle=np.pi / 2).elements.tolist()
    ctx = _ctx([0, 0, 0], [1, 0, 0, 0], [10.0, 5.0, 0.0], q_ep)

    v_ego = np.array([[3.0, 0.0]])  # a "velocity" of 3 m/s in ego-forward
    v_global = ego_to_global_vector(v_ego, ctx)
    # Rz(90) @ (3, 0) = (0, 3) -- rotated, with NO (10, 5) translation added.
    np.testing.assert_allclose(v_global, [[0.0, 3.0]], atol=1e-9)


def test_vector_round_trip_sensor_to_global_and_back():
    rng = np.random.default_rng(1)
    q_cs = Quaternion(axis=[0, 0, 1], angle=0.6).elements.tolist()
    q_ep = Quaternion(axis=[1, 2, 3], angle=0.9).elements.tolist()
    # Deliberately large translations -- round trip must still land exactly
    # back on the original vector, proving translation truly cancels out
    # (it would NOT cancel for a point transform's inverse unless the
    # intermediate representation also discarded it correctly).
    ctx = _ctx([50.0, -30.0, 2.0], q_cs, [500.0, -800.0, 0.0], q_ep)
    vectors = rng.uniform(-5, 5, size=(10, 2))

    glob = sensor_to_global_vector(vectors, ctx)
    ego = sensor_to_ego_vector(vectors, ctx)
    np.testing.assert_allclose(glob, ego_to_global_vector(ego, ctx), atol=1e-9)


def test_global_to_ego_vector_is_the_true_inverse_of_ego_to_global_vector():
    # step12 (BEV): a track's global-frame KF velocity rotated into ego
    # frame for display. Round trip must land exactly back on the
    # original vector, with the large ego translation again contributing
    # nothing (only rotation, same w=0 trick as every other vector helper).
    #
    # Yaw-only rotation deliberately (axis=[0,0,1]), not an arbitrary 3D
    # tumble: a 2D vector's z=0 is only preserved through BOTH legs of the
    # round trip when the rotation keeps the xy-plane invariant. Real
    # nuScenes ego poses are always near-level (yaw, with only tiny real
    # pitch/roll), so this matches the one case this helper is actually
    # used for -- a true tumbling rotation would lose the z-component
    # truncated at the ego-frame intermediate step, which is a real,
    # already-accepted limitation of a 2D-only vector API, not something
    # this test needs to (or should) exercise.
    q_ep = Quaternion(axis=[0, 0, 1], angle=1.1).elements.tolist()
    ctx = _ctx([0, 0, 0], [1, 0, 0, 0], [500.0, -800.0, 3.0], q_ep)
    rng = np.random.default_rng(2)
    vectors = rng.uniform(-5, 5, size=(10, 2))

    ego = global_to_ego_vector(vectors, ctx)
    back_to_global = ego_to_global_vector(ego, ctx)
    np.testing.assert_allclose(back_to_global, vectors, atol=1e-9)


# ── Real-data fixtures ────────────────────────────────────────────────────

@pytest.fixture(scope="module")
def nusc():
    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        pytest.skip(f"DATAROOT {config.DATAROOT} does not have the expected nuScenes layout")
    return NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)


def _lidar_sample_data(nusc, sample):
    return nusc.get("sample_data", sample["data"]["LIDAR_TOP"])


def _raw_event_from_sample_data(nusc, sd) -> RawEvent:
    """Only used for LIDAR_TOP records in this test file."""
    sample = nusc.get("sample", sd["sample_token"])
    return RawEvent(
        t_us=sd["timestamp"],
        channel="LIDAR_TOP",
        modality="lidar",
        sample_data_token=sd["token"],
        ego_pose_token=sd["ego_pose_token"],
        calibrated_sensor_token=sd["calibrated_sensor_token"],
        filename=sd["filename"],
        is_key_frame=sd["is_key_frame"],
        scene_token=sample["scene_token"],
    )


def _has_attribute(nusc, ann, attribute_name: str) -> bool:
    for tok in ann["attribute_tokens"]:
        if nusc.get("attribute", tok)["name"] == attribute_name:
            return True
    return False


# A GT box that ACTUALLY didn't move must show it in its own translation,
# not just in the "vehicle.parked" attribute -- attributes can be wrong.
# The instance first tried here (b0e4c63c...) carries "vehicle.parked" on
# every one of its 21 annotations while its own GT translation advances
# ~2.7 m every keyframe (~5.4 m/s) -- a moving vehicle mislabeled parked in
# this data, most likely from weakly-fit early boxes with 0 LiDAR points.
# Trusting the attribute alone would have picked exactly this bad example
# (what-not-to-do.md's "don't trust a reported value without checking" logic
# applies to a dataset's own attribute label too, not just sensor output).
_MAX_TRUE_GT_DRIFT_M = 0.3


def _find_best_parked_pair(nusc, min_lidar_pts: int = 15):
    """Find the (ann_a, ann_b) pair of consecutive-keyframe annotations of
    the SAME vehicle.parked instance with the largest ego displacement
    between the two keyframes (a bigger, clearer demonstration of the bug
    this utility prevents than a near-stationary segment), enough LiDAR
    points at both ends for a meaningful centroid, AND whose own GT
    translation barely moves (independently confirms it is really
    stationary, not just attributed as such)."""
    best = None
    best_disp = -1.0
    for instance in nusc.instance:
        ann = nusc.get("sample_annotation", instance["first_annotation_token"])
        while ann["next"]:
            nxt = nusc.get("sample_annotation", ann["next"])
            gt_drift = float(np.linalg.norm(
                np.array(nxt["translation"]) - np.array(ann["translation"])
            ))
            if (
                _has_attribute(nusc, ann, "vehicle.parked")
                and _has_attribute(nusc, nxt, "vehicle.parked")
                and ann["num_lidar_pts"] >= min_lidar_pts
                and nxt["num_lidar_pts"] >= min_lidar_pts
                and gt_drift <= _MAX_TRUE_GT_DRIFT_M
            ):
                sample_a = nusc.get("sample", ann["sample_token"])
                sample_b = nusc.get("sample", nxt["sample_token"])
                ep_a = nusc.get("ego_pose", _lidar_sample_data(nusc, sample_a)["ego_pose_token"])
                ep_b = nusc.get("ego_pose", _lidar_sample_data(nusc, sample_b)["ego_pose_token"])
                disp = float(np.linalg.norm(
                    np.array(ep_b["translation"]) - np.array(ep_a["translation"])
                ))
                if disp > best_disp:
                    best_disp = disp
                    best = (ann, nxt)
            ann = nxt
    return best, best_disp


# ── Test 5: ego-motion evidence (real data) ───────────────────────────────

def test_ego_motion_evidence_parked_vehicle(nusc, tmp_path):
    from nuscenes.utils.data_classes import LidarPointCloud
    from nuscenes.utils.geometry_utils import points_in_box

    pair, ego_disp = _find_best_parked_pair(nusc)
    if pair is None:
        pytest.skip("No vehicle.parked instance with >=2 consecutive well-observed keyframes found")
    ann_a, ann_b = pair
    assert ego_disp > 0.1, "Chosen pair has negligible ego displacement -- picked a weak case"

    raw_centroids = []
    global_centroids = []
    for ann in (ann_a, ann_b):
        sample = nusc.get("sample", ann["sample_token"])
        sd = _lidar_sample_data(nusc, sample)
        _, boxes, _ = nusc.get_sample_data(sd["token"], selected_anntokens=[ann["token"]])
        box = boxes[0]  # already in LIDAR_TOP's own sensor frame

        pc = LidarPointCloud.from_file(str(config.DATAROOT / sd["filename"]))
        mask = points_in_box(box, pc.points[:3, :])
        pts_sensor = pc.points[:3, mask].T  # (M, 3), LiDAR's own local frame
        assert pts_sensor.shape[0] >= 5, "Too few points inside the GT box for a stable centroid"

        raw_centroids.append(pts_sensor.mean(axis=0))

        ctx = FrameContext.from_event(nusc, _raw_event_from_sample_data(nusc, sd))
        pts_global = sensor_to_global(pts_sensor, ctx)
        global_centroids.append(pts_global.mean(axis=0))

    raw_diff = np.linalg.norm(raw_centroids[1] - raw_centroids[0])
    global_diff = np.linalg.norm(global_centroids[1][:2] - global_centroids[0][:2])

    # The parked object's GLOBAL centroid must agree closely across keyframes ...
    assert global_diff < 0.5, f"Global centroid disagreement too large: {global_diff:.3f} m"
    # ... while the RAW sensor-frame centroid differs by roughly the ego displacement,
    # because the object didn't move but the sensor (mounted on ego) did.
    assert raw_diff > 0.5 * ego_disp, (
        f"Raw sensor-frame centroid barely moved ({raw_diff:.3f} m) despite ego moving "
        f"{ego_disp:.3f} m -- this test is supposed to demonstrate that exact discrepancy"
    )

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5))
    raw_arr = np.array(raw_centroids)
    glob_arr = np.array(global_centroids)
    axes[0].plot(raw_arr[:, 0], raw_arr[:, 1], "o-", color="crimson")
    for i, p in enumerate(raw_arr):
        axes[0].annotate(f"t{i}", (p[0], p[1]))
    axes[0].set_title(f"Raw sensor-frame centroid\n(moves {raw_diff:.2f} m -- ego motion, not the object)")
    axes[0].set_xlabel("x (m, LiDAR-local)")
    axes[0].set_ylabel("y (m, LiDAR-local)")
    axes[0].axis("equal")

    axes[1].plot(glob_arr[:, 0], glob_arr[:, 1], "o-", color="seagreen")
    for i, p in enumerate(glob_arr):
        axes[1].annotate(f"t{i}", (p[0], p[1]))
    axes[1].set_title(f"Global-frame centroid\n(agrees within {global_diff:.2f} m -- correctly stationary)")
    axes[1].set_xlabel("x (m, global)")
    axes[1].set_ylabel("y (m, global)")
    axes[1].axis("equal")

    fig.suptitle(
        f"Ego-motion evidence: parked instance, ego displaced {ego_disp:.2f} m between keyframes"
    )
    fig.tight_layout()

    out_dir = Path("outputs/figs/step02")
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "ego_motion_evidence.png", dpi=150)
    plt.close(fig)


# ── Test 6: per-channel independence (real data) ──────────────────────────

def test_per_channel_pose_and_timestamp_independence(nusc):
    channels = config.ACTIVE_CHANNELS.value
    n_checked = 0
    for sample in nusc.sample[:10]:
        records = {ch: nusc.get("sample_data", sample["data"][ch]) for ch in channels}
        lidar = records["LIDAR_TOP"]

        for ch, sd in records.items():
            if ch == "LIDAR_TOP":
                continue
            dt_ms = abs(sd["timestamp"] - lidar["timestamp"]) / 1000.0
            same_pose = sd["ego_pose_token"] == lidar["ego_pose_token"]

            ep_ch = nusc.get("ego_pose", sd["ego_pose_token"])
            ep_lidar = nusc.get("ego_pose", lidar["ego_pose_token"])
            disp_m = float(np.linalg.norm(
                np.array(ep_ch["translation"]) - np.array(ep_lidar["translation"])
            ))
            print(
                f"{sample['token'][:8]} {ch:18s} vs LIDAR_TOP: "
                f"dt={dt_ms:7.2f} ms, same_ego_pose_token={same_pose}, "
                f"ego displacement between the two poses={disp_m * 1000:.2f} mm"
            )
            n_checked += 1

        # G2: a radar event's FrameContext must resolve to THAT record's own
        # ego_pose, never LIDAR_TOP's -- prove it structurally, not just by token string.
        radar_ch = next(ch for ch in channels if "RADAR" in ch)
        radar_sd = records[radar_ch]
        radar_event = RawEvent(
            t_us=radar_sd["timestamp"],
            channel=radar_ch,
            modality="radar",
            sample_data_token=radar_sd["token"],
            ego_pose_token=radar_sd["ego_pose_token"],
            calibrated_sensor_token=radar_sd["calibrated_sensor_token"],
            filename=radar_sd["filename"],
            is_key_frame=radar_sd["is_key_frame"],
            scene_token=sample["scene_token"],
        )
        ctx = FrameContext.from_event(nusc, radar_event)
        expected_ego_pose = nusc.get("ego_pose", radar_sd["ego_pose_token"])
        assert ctx.ego_pose == expected_ego_pose
        assert ego_pose_timestamp(ctx) == expected_ego_pose["timestamp"]

    assert n_checked > 0
