"""
tests/test_lidar_adapter.py — step04a_lidar_adapter.md §6, items 1-6
(synthetic). Item 7 (real-data BEV overlay) lives in
scripts/lidar_adapter_evidence.py, run separately against the real dataset.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.adapters.lidar import lidar_detections, process_lidar_sweep
from ttcf.geometry.transforms import FrameContext, global_to_ego
from ttcf.types import RawEvent

IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]


class FakeNuScenes:
    def __init__(self, tables: dict):
        self._tables = tables

    def get(self, table, token):
        return self._tables[table][token]


def _write_lidar_bin(path: Path, points_xyz: np.ndarray) -> None:
    n = len(points_xyz)
    scan = np.zeros((n, 5), dtype=np.float32)
    scan[:, :3] = points_xyz
    scan.astype(np.float32).tofile(str(path))


def _ground_grid(x_range=(0.5, 20.0), y_range=(-6.0, 6.0), step=0.5, z=0.0):
    xs = np.arange(*x_range, step)
    ys = np.arange(*y_range, step)
    xx, yy = np.meshgrid(xs, ys)
    zz = np.full(xx.shape, z)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)


def _box_near_face(center_x, center_y, width, height=1.5, n_y=10, n_z=8):
    """A flat grid of points on a box's near (ego-facing) face at x=center_x."""
    ys = np.linspace(center_y - width / 2, center_y + width / 2, n_y)
    zs = np.linspace(0.2, height, n_z)
    yy, zz = np.meshgrid(ys, zs)
    xx = np.full(yy.shape, center_x)
    return np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)


def _build(tmp_path, name, points_xyz, ego_translation=(0.0, 0.0, 0.0), ego_rotation=IDENTITY_Q, t_us=0):
    """Writes a synthetic sweep to disk and builds a matching (nusc, event)
    pair with identity sensor calibration -- ego frame == sensor frame,
    so synthetic points can be authored directly in the frame that
    matters for each test."""
    bin_path = tmp_path / f"{name}.bin"
    _write_lidar_bin(bin_path, points_xyz)

    cs_token, ep_token = f"cs_{name}", f"ep_{name}"
    tables = {
        "calibrated_sensor": {cs_token: {"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q}},
        "ego_pose": {ep_token: {"translation": list(ego_translation), "rotation": list(ego_rotation)}},
    }
    nusc = FakeNuScenes(tables)
    event = RawEvent(
        t_us=t_us, channel="LIDAR_TOP", modality="lidar", sample_data_token=f"sd_{name}",
        ego_pose_token=ep_token, calibrated_sensor_token=cs_token, filename=str(bin_path),
        is_key_frame=True, scene_token="scene_0",
    )
    return nusc, event


# ── Test 1: one box -> exactly one detection at the expected reference point ─

def test_single_box_gives_one_detection_at_expected_reference_point(tmp_path):
    ground = _ground_grid()
    box = _box_near_face(center_x=10.0, center_y=0.0, width=1.8)  # 4.5x1.8m box, near face at x=10
    points = np.vstack([ground, box])
    nusc, event = _build(tmp_path, "single_box", points)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    assert len(dets) == 1
    assert dets[0].xy_global[0] == pytest.approx(10.0, abs=0.2)
    assert dets[0].xy_global[1] == pytest.approx(0.0, abs=0.2)
    assert dets[0].n_points == len(box)


# ── Test 2: two boxes 3m apart -> two detections; touching boxes documented ─

def test_two_separated_boxes_give_two_detections(tmp_path):
    ground = _ground_grid()
    box_a = _box_near_face(center_x=10.0, center_y=-1.5, width=1.0)
    box_b = _box_near_face(center_x=10.0, center_y=1.5, width=1.0)
    points = np.vstack([ground, box_a, box_b])
    nusc, event = _build(tmp_path, "two_boxes_separated", points)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    assert len(dets) == 2


def test_two_touching_boxes_documented_behaviour(tmp_path):
    """Two boxes close enough together that DBSCAN's eps (0.7m) connects
    them transitively -- documents the observed behaviour (merges into
    one cluster), consistent with the same, already-named crowd-
    conflation-adjacent limitation from step06: position-only clustering
    cannot always separate two close real objects. Not fixed here."""
    ground = _ground_grid()
    box_a = _box_near_face(center_x=10.0, center_y=-0.3, width=1.0)
    box_b = _box_near_face(center_x=10.0, center_y=0.3, width=1.0)
    points = np.vstack([ground, box_a, box_b])
    nusc, event = _build(tmp_path, "two_boxes_touching", points)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    print(f"\nTouching-boxes scenario: {len(dets)} detection(s) for 2 boxes 0.6m apart "
          f"(DBSCAN eps={config.LIDAR_CLUSTER_EPS_M.value}m). Documented, not fixed.")
    assert len(dets) in (1, 2)


# ── Test 3: ground-only cloud -> zero detections ──────────────────────────

def test_ground_only_cloud_gives_zero_detections(tmp_path):
    ground = _ground_grid()
    nusc, event = _build(tmp_path, "ground_only", ground)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    assert dets == []


# ── Test 4: detections are in the GLOBAL frame (G1) ───────────────────────

def test_detection_position_is_frame_invariant_global(tmp_path):
    from pyquaternion import Quaternion

    # Within the adapter's ROI half-width (~7.7m: CORRIDOR_HALF_WIDTH_M +
    # CANDIDATE_EXTRA_MARGIN_M + LIDAR_ROI_SLACK_M) -- an earlier version
    # of this test placed the object 20m to the side, well outside the
    # ROI by design, so every box point got cropped out before reaching
    # clustering (0 detections). Fixed the scenario, not the adapter.
    target_global = np.array([50.0, 3.0, 0.5])

    # Ego pose 1: at the global origin, facing +x.
    ctx1 = FrameContext(
        calibrated_sensor={"translation": [0, 0, 0], "rotation": IDENTITY_Q},
        ego_pose={"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q},
    )
    local1 = global_to_ego(np.array([target_global]), ctx1)[0]
    ground1 = _ground_grid(x_range=(0.5, 60.0), y_range=(-10.0, 30.0), step=1.0)
    box1 = _box_near_face(local1[0], local1[1], width=1.8)
    nusc1, event1 = _build(tmp_path, "pose1", np.vstack([ground1, box1]), ego_translation=(0.0, 0.0, 0.0))

    # Ego pose 2: a different position/heading, but modest enough that the
    # SAME physical target still falls within ego2's own ROI too (a
    # larger 40-degree/10m version of this test put the target 27m to
    # the side of ego2's own forward axis -- outside its ROI by design,
    # so it got cropped out entirely, 0 detections. Fixed the scenario's
    # pose choice, not the adapter -- both egos must actually be able to
    # see the target for a same-object comparison to make sense).
    yaw2 = 0.05
    ep2_rotation = Quaternion(axis=[0, 0, 1], angle=yaw2).elements.tolist()
    ctx2 = FrameContext(
        calibrated_sensor={"translation": [0, 0, 0], "rotation": IDENTITY_Q},
        ego_pose={"translation": [1.0, 0.5, 0.0], "rotation": ep2_rotation},
    )
    local2 = global_to_ego(np.array([target_global]), ctx2)[0]
    ground2 = _ground_grid(x_range=(0.5, 70.0), y_range=(-15.0, 15.0), step=1.0)
    box2 = _box_near_face(local2[0], local2[1], width=1.8)
    nusc2, event2 = _build(
        tmp_path, "pose2", np.vstack([ground2, box2]), ego_translation=(1.0, 0.5, 0.0), ego_rotation=ep2_rotation
    )

    dets1 = lidar_detections(nusc1, event1, rng=np.random.default_rng(1))
    dets2 = lidar_detections(nusc2, event2, rng=np.random.default_rng(1))
    assert len(dets1) == 1 and len(dets2) == 1
    np.testing.assert_allclose(dets1[0].xy_global, dets2[0].xy_global, atol=0.3)
    np.testing.assert_allclose(dets1[0].xy_global, target_global[:2], atol=0.3)


# ── Test 5: timestamp/pose come from the event's own record ──────────────

def test_timestamp_comes_from_event_record(tmp_path):
    ground = _ground_grid()
    box = _box_near_face(10.0, 0.0, 1.8)
    nusc, event = _build(tmp_path, "timestamp_check", np.vstack([ground, box]), t_us=1533151603512404)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    assert len(dets) == 1
    assert dets[0].t_us == 1533151603512404
    assert dets[0].channel == "LIDAR_TOP"


# ── Test 6: R_global symmetric PSD, yaw rotation applied correctly ───────

def test_r_global_symmetric_psd_and_yaw_applied(tmp_path):
    from pyquaternion import Quaternion

    ground = _ground_grid()
    box = _box_near_face(10.0, 0.0, 1.8)

    # 90 degree ego yaw -- should swap sigma_long^2/sigma_lat^2 (step02's own worked check).
    ep_rotation = Quaternion(axis=[0, 0, 1], angle=np.pi / 2).elements.tolist()
    nusc, event = _build(tmp_path, "yaw_check", np.vstack([ground, box]), ego_rotation=ep_rotation)

    dets = lidar_detections(nusc, event, rng=np.random.default_rng(0))
    assert len(dets) == 1
    R = dets[0].R_global
    np.testing.assert_allclose(R, R.T, atol=1e-9)  # symmetric
    eigvals = np.linalg.eigvalsh(R)
    assert np.all(eigvals > 0)  # positive definite

    sigma_long = config.SENSOR_R_LIDAR.value["sigma_long_m"]
    sigma_lat = config.SENSOR_R_LIDAR.value["sigma_lat_m"]
    expected = np.diag([sigma_lat**2, sigma_long**2])  # swapped at 90 degrees
    np.testing.assert_allclose(R, expected, atol=1e-6)
