"""
tests/test_radar_adapter.py — step04b_radar_adapter.md §5, items 1-5, 7
(synthetic) + item 4 (doppler-ignored, reuses the tracker directly). Item 6
(real-data plot) lives in scripts/radar_adapter_evidence.py, run separately
against the real dataset.
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.adapters.radar import radar_detections
from ttcf.geometry.transforms import FrameContext, global_to_ego
from ttcf.tracking.tracker import ShortMemoryTracker
from ttcf.types import Detection, RawEvent

IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]

# RadarPointCloud's own field layout (from_file docstring) -- 18 fields,
# per-field struct format codes exactly matching the devkit's own reader
# (unpacking_lut: F/4->'f', I/1->'b', I/2->'h').
_FIELDS = (
    "x", "y", "z", "dyn_prop", "id", "rcs", "vx", "vy", "vx_comp", "vy_comp",
    "is_quality_valid", "ambig_state", "x_rms", "y_rms", "invalid_state", "pdh0", "vx_rms", "vy_rms",
)
_FMT = ("f", "f", "f", "b", "h", "f", "f", "f", "f", "f", "b", "b", "b", "b", "b", "b", "b", "b")

_PCD_HEADER = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z dyn_prop id rcs vx vy vx_comp vy_comp is_quality_valid ambig_state x_rms y_rms invalid_state pdh0 vx_rms vy_rms\n"
    "SIZE 4 4 4 1 2 4 4 4 4 4 1 1 1 1 1 1 1 1\n"
    "TYPE F F F I I F F F F F I I I I I I I I\n"
    "COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1\n"
    "WIDTH {n}\n"
    "HEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS {n}\n"
    "DATA binary\n"
)


class FakeNuScenes:
    def __init__(self, tables: dict):
        self._tables = tables

    def get(self, table, token):
        return self._tables[table][token]


def _write_radar_pcd(path: Path, xyz: np.ndarray, dyn_prop=None, vx_comp=None, vy_comp=None, rcs=None,
                      invalid_state=None) -> None:
    n = len(xyz)
    cols = {
        "x": xyz[:, 0], "y": xyz[:, 1], "z": xyz[:, 2],
        "dyn_prop": np.zeros(n) if dyn_prop is None else np.asarray(dyn_prop),
        "id": np.arange(n),
        "rcs": np.zeros(n) if rcs is None else np.asarray(rcs),
        "vx": np.zeros(n), "vy": np.zeros(n),
        "vx_comp": np.zeros(n) if vx_comp is None else np.asarray(vx_comp),
        "vy_comp": np.zeros(n) if vy_comp is None else np.asarray(vy_comp),
        "is_quality_valid": np.zeros(n), "ambig_state": np.full(n, 3),
        "x_rms": np.zeros(n), "y_rms": np.zeros(n),
        "invalid_state": np.zeros(n) if invalid_state is None else np.asarray(invalid_state),
        "pdh0": np.zeros(n),
        "vx_rms": np.zeros(n), "vy_rms": np.zeros(n),
    }
    with open(path, "wb") as f:
        f.write(_PCD_HEADER.format(n=n).encode("utf-8"))
        for i in range(n):
            for field, fmt in zip(_FIELDS, _FMT):
                val = cols[field][i]
                f.write(struct.pack(fmt, int(val) if fmt != "f" else float(val)))
        # from_file()'s own byte-offset check is a strict `<` against the
        # buffer length -- the last field of the last point needs at least
        # one trailing byte of slack, or that assertion fails.
        f.write(b"\x00")


def _build(tmp_path, name, points_xyz, channel="RADAR_FRONT", ego_translation=(0.0, 0.0, 0.0),
           ego_rotation=IDENTITY_Q, t_us=0, **pcd_kwargs):
    pcd_path = tmp_path / f"{name}.pcd"
    _write_radar_pcd(pcd_path, points_xyz, **pcd_kwargs)

    cs_token, ep_token = f"cs_{name}", f"ep_{name}"
    tables = {
        "calibrated_sensor": {cs_token: {"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q}},
        "ego_pose": {ep_token: {"translation": list(ego_translation), "rotation": list(ego_rotation)}},
    }
    nusc = FakeNuScenes(tables)
    event = RawEvent(
        t_us=t_us, channel=channel, modality="radar", sample_data_token=f"sd_{name}",
        ego_pose_token=ep_token, calibrated_sensor_token=cs_token, filename=str(pcd_path),
        is_key_frame=True, scene_token="scene_0",
    )
    return nusc, event


def _blips(center_x, center_y, n=3, spread=0.3, rng=None):
    """A handful of radar returns scattered near one point -- radar
    typically gives 1-3 points per real object, not a dense face grid."""
    rng = rng if rng is not None else np.random.default_rng(0)
    offsets = rng.uniform(-spread, spread, size=(n, 2))
    xy = np.array([center_x, center_y]) + offsets
    z = np.zeros(n)
    return np.column_stack([xy, z])


# ── Test 1: several blips on one footprint -> exactly one detection ──────

def test_single_object_blips_give_one_detection(tmp_path):
    points = _blips(15.0, 0.0, n=4, spread=0.5)
    nusc, event = _build(tmp_path, "single_object", points)

    dets = radar_detections(nusc, event)
    assert len(dets) == 1
    assert dets[0].xy_global[0] == pytest.approx(15.0, abs=0.6)
    assert dets[0].xy_global[1] == pytest.approx(0.0, abs=0.6)
    assert dets[0].n_points == 4


# ── Test 2: two objects 5m apart -> two detections ────────────────────────

def test_two_objects_five_m_apart_give_two_detections(tmp_path):
    points = np.vstack([_blips(15.0, -2.5, n=2, spread=0.2), _blips(15.0, 2.5, n=2, spread=0.2)])
    nusc, event = _build(tmp_path, "two_objects", points)

    dets = radar_detections(nusc, event)
    assert len(dets) == 2


# ── Test 3: detections carry THIS channel's own t_us/pose ────────────────

def test_detections_carry_own_channel_timestamp(tmp_path):
    points = _blips(15.0, 0.0, n=3)
    nusc, event = _build(tmp_path, "front_left", points, channel="RADAR_FRONT_LEFT", t_us=1533151603512404)

    dets = radar_detections(nusc, event)
    assert len(dets) == 1
    assert dets[0].t_us == 1533151603512404
    assert dets[0].channel == "RADAR_FRONT_LEFT"


# ── Test 4: Doppler-ignored -- tracker output identical with aux stripped ─

def test_tracker_output_identical_with_doppler_aux_stripped(tmp_path):
    points = np.vstack([_blips(15.0, -2.5, n=2, spread=0.2), _blips(15.0, 2.5, n=2, spread=0.2)])
    nusc, event = _build(tmp_path, "doppler_check", points, vx_comp=[5.0, -3.0, 8.0, -1.0], rcs=[12.0, 9.0, 20.0, 3.0])

    dets = radar_detections(nusc, event)
    assert len(dets) == 2
    assert any(d.aux.get("doppler_vx_comp") for d in dets)  # aux actually populated, not a vacuous test

    stripped = [
        Detection(
            t_us=d.t_us, channel=d.channel, modality=d.modality, xy_global=d.xy_global, R_global=d.R_global,
            ref_point_kind=d.ref_point_kind, cls=d.cls, n_points=d.n_points, aux={},
        )
        for d in dets
    ]

    tracker_a = ShortMemoryTracker()
    tracker_b = ShortMemoryTracker()
    snaps_a = tracker_a.process_scan(dets, event.t_us)
    snaps_b = tracker_b.process_scan(stripped, event.t_us)

    assert len(snaps_a) == len(snaps_b) == 2
    for sa, sb in zip(
        sorted(snaps_a, key=lambda s: s.x[1]), sorted(snaps_b, key=lambda s: s.x[1])
    ):
        np.testing.assert_allclose(sa.x, sb.x)
        np.testing.assert_allclose(sa.P, sb.P)


# ── Test 5: detections are in the GLOBAL frame (G1) ───────────────────────

def test_detection_position_is_frame_invariant_global(tmp_path):
    from pyquaternion import Quaternion

    target_global = np.array([50.0, 3.0, 0.0])

    ctx1 = FrameContext(
        calibrated_sensor={"translation": [0, 0, 0], "rotation": IDENTITY_Q},
        ego_pose={"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q},
    )
    local1 = global_to_ego(np.array([target_global]), ctx1)[0]
    blips1 = _blips(local1[0], local1[1], n=3, spread=0.2, rng=np.random.default_rng(1))
    nusc1, event1 = _build(tmp_path, "pose1", blips1, ego_translation=(0.0, 0.0, 0.0))

    yaw2 = 0.05
    ep2_rotation = Quaternion(axis=[0, 0, 1], angle=yaw2).elements.tolist()
    ctx2 = FrameContext(
        calibrated_sensor={"translation": [0, 0, 0], "rotation": IDENTITY_Q},
        ego_pose={"translation": [1.0, 0.5, 0.0], "rotation": ep2_rotation},
    )
    local2 = global_to_ego(np.array([target_global]), ctx2)[0]
    blips2 = _blips(local2[0], local2[1], n=3, spread=0.2, rng=np.random.default_rng(1))
    nusc2, event2 = _build(
        tmp_path, "pose2", blips2, ego_translation=(1.0, 0.5, 0.0), ego_rotation=ep2_rotation
    )

    dets1 = radar_detections(nusc1, event1)
    dets2 = radar_detections(nusc2, event2)
    assert len(dets1) == 1 and len(dets2) == 1
    np.testing.assert_allclose(dets1[0].xy_global, dets2[0].xy_global, atol=0.3)
    np.testing.assert_allclose(dets1[0].xy_global, target_global[:2], atol=0.3)


# ── Test 7 (step04b's own numbering): R_global symmetric PSD, yaw applied ─

def test_r_global_symmetric_psd_and_yaw_applied(tmp_path):
    from pyquaternion import Quaternion

    points = _blips(15.0, 0.0, n=3)
    ep_rotation = Quaternion(axis=[0, 0, 1], angle=np.pi / 2).elements.tolist()
    nusc, event = _build(tmp_path, "yaw_check", points, ego_rotation=ep_rotation)

    dets = radar_detections(nusc, event)
    assert len(dets) == 1
    R = dets[0].R_global
    np.testing.assert_allclose(R, R.T, atol=1e-9)
    eigvals = np.linalg.eigvalsh(R)
    assert np.all(eigvals > 0)

    sigma_long = config.SENSOR_R_RADAR.value["sigma_long_m"]
    sigma_lat = config.SENSOR_R_RADAR.value["sigma_lat_m"]
    expected = np.diag([sigma_lat**2, sigma_long**2])  # swapped at 90 degrees
    np.testing.assert_allclose(R, expected, atol=1e-6)


# ── Devkit-default filter behaviour: invalid_state excludes near-field artefacts

def test_devkit_default_invalid_state_filter_is_respected(tmp_path):
    """Not in step04b's own numbered list, but directly verifies the
    module docstring's claim (devkit defaults keep only invalid_state==0)
    -- a point flagged invalid_state=2 (near-field artefact, per the
    devkit's own dynProp/invalid_state table) must never reach clustering."""
    good = _blips(15.0, 0.0, n=3, spread=0.2)
    bad = np.array([[0.5, 0.1, 0.0]])  # would-be near-field artefact, far from the real cluster
    points = np.vstack([good, bad])
    nusc, event = _build(tmp_path, "invalid_state_check", points, invalid_state=[0, 0, 0, 2])

    dets = radar_detections(nusc, event)
    assert len(dets) == 1
    assert dets[0].n_points == 3  # the invalid_state=2 point never reached clustering
