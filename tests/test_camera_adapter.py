"""
tests/test_camera_adapter.py — step04c_camera_adapter.md §5, items 2-7
(synthetic). Item 1 (isolation) lives in tests/test_camera_isolation.py.
Item 8 (real-data overlay) lives in scripts/camera_adapter_evidence.py.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.adapters.camera import _cache_path, _camera_R, camera_detections
from ttcf.geometry.transforms import FrameContext, ego_to_sensor
from ttcf.types import RawEvent

IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]
# A real front-camera's own frame (x-right, y-down, z-forward) is a
# DIFFERENT axis convention than ego's (x-forward, y-left, z-up) -- an
# identity rotation is not physically valid here (it would point the
# camera's forward axis along ego's own x, with no axis swap at all, so a
# "below the horizon" pixel would ray-trace UPWARD in the ego frame
# instead of down to the ground). Reusing nuScenes' own real CAM_FRONT
# calibration quaternion (w,x,y,z) keeps every synthetic test physically
# realistic rather than inventing an ad hoc rotation.
CAM_ROTATION_TYPICAL = [0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755]
K_DEFAULT = np.array([[1000.0, 0.0, 800.0], [0.0, 1000.0, 450.0], [0.0, 0.0, 1.0]])
IMAGE_H, IMAGE_W = 900, 1600
CAM_HEIGHT_M = 1.5


class FakeNuScenes:
    def __init__(self, tables: dict):
        self._tables = tables

    def get(self, table, token):
        return self._tables[table][token]


def _build(
    name, box_source, ego_translation=(0.0, 0.0, 0.0), ego_rotation=IDENTITY_Q,
    cam_translation=(0.0, 0.0, CAM_HEIGHT_M), cam_rotation=CAM_ROTATION_TYPICAL, K=K_DEFAULT, t_us=0,
):
    cs_token, ep_token, sd_token = f"cs_{name}", f"ep_{name}", f"sd_{name}"
    tables = {
        "calibrated_sensor": {cs_token: {"translation": list(cam_translation), "rotation": list(cam_rotation),
                                          "camera_intrinsic": K.tolist()}},
        "ego_pose": {ep_token: {"translation": list(ego_translation), "rotation": list(ego_rotation)}},
        "sample_data": {sd_token: {"height": IMAGE_H, "width": IMAGE_W}},
    }
    nusc = FakeNuScenes(tables)
    event = RawEvent(
        t_us=t_us, channel="CAM_FRONT", modality="camera", sample_data_token=sd_token,
        ego_pose_token=ep_token, calibrated_sensor_token=cs_token, filename=f"{name}.jpg",
        is_key_frame=True, scene_token="scene_0",
    )
    return nusc, event


def _project_ego_point_to_pixel(point_ego, K, ctx) -> tuple:
    """Forward pinhole projection (ego point -> pixel), used only by tests
    to construct a synthetic box whose bottom-centre pixel corresponds to
    a KNOWN ego-frame ground point -- the adapter itself only ever goes
    pixel -> ego (backward), never this direction."""
    p_cam = ego_to_sensor(np.array([point_ego]), ctx)[0]
    uvw = K @ p_cam
    return float(uvw[0] / uvw[2]), float(uvw[1] / uvw[2])


# ── Test 2: ground-plane geometry round trip ──────────────────────────────

def test_ground_plane_round_trip_recovers_known_point():
    ctx = FrameContext(
        calibrated_sensor={"translation": [0.0, 0.0, CAM_HEIGHT_M], "rotation": CAM_ROTATION_TYPICAL,
                            "camera_intrinsic": K_DEFAULT.tolist()},
        ego_pose={"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q},
    )
    known_ego_point = np.array([20.0, 1.5, 0.0])  # on the ground, DEC-6's own z=0 default
    u, v = _project_ego_point_to_pixel(known_ego_point, K_DEFAULT, ctx)

    def box_source(nusc, event):
        return [{"bbox": [u - 5, v - 40, u + 5, v], "cls_id": 2, "conf": 0.9}]

    nusc, event = _build("roundtrip", box_source)
    dets = camera_detections(nusc, event, box_source=box_source)
    assert len(dets) == 1
    np.testing.assert_allclose(dets[0].xy_global, known_ego_point[:2], atol=1e-3)


# ── Test 3: sigma_long grows quadratically with range ─────────────────────

def test_sigma_long_grows_quadratically_with_range():
    focal_px, cam_h = 1000.0, 1.5
    R_near = _camera_R(range_m=10.0, focal_px=focal_px, cam_height_m=cam_h)
    R_far = _camera_R(range_m=20.0, focal_px=focal_px, cam_height_m=cam_h)
    sigma_long_near = np.sqrt(R_near[0, 0])
    sigma_long_far = np.sqrt(R_far[0, 0])
    # range doubled -> sigma_long should quadruple (variance x16, std x4...
    # wait: sigma_long itself ~ range^2, so doubling range -> sigma_long x4)
    assert sigma_long_far / sigma_long_near == pytest.approx(4.0, rel=1e-6)


# ── Test 4: detections are in the GLOBAL frame (G1) ───────────────────────

def test_detection_position_is_frame_invariant_global():
    from pyquaternion import Quaternion

    target_global = np.array([30.0, 2.0, 0.0])  # on the ground

    ctx1 = FrameContext(
        calibrated_sensor={"translation": [0.0, 0.0, CAM_HEIGHT_M], "rotation": CAM_ROTATION_TYPICAL,
                            "camera_intrinsic": K_DEFAULT.tolist()},
        ego_pose={"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q},
    )
    from ttcf.geometry.transforms import global_to_ego
    local1 = global_to_ego(np.array([target_global]), ctx1)[0]
    u1, v1 = _project_ego_point_to_pixel(local1, K_DEFAULT, ctx1)

    yaw2 = 0.05
    ep2_rotation = Quaternion(axis=[0, 0, 1], angle=yaw2).elements.tolist()
    ctx2 = FrameContext(
        calibrated_sensor={"translation": [0.0, 0.0, CAM_HEIGHT_M], "rotation": CAM_ROTATION_TYPICAL,
                            "camera_intrinsic": K_DEFAULT.tolist()},
        ego_pose={"translation": [1.0, 0.5, 0.0], "rotation": ep2_rotation},
    )
    local2 = global_to_ego(np.array([target_global]), ctx2)[0]
    u2, v2 = _project_ego_point_to_pixel(local2, K_DEFAULT, ctx2)

    def box_source_1(nusc, event):
        return [{"bbox": [u1 - 5, v1 - 40, u1 + 5, v1], "cls_id": 2, "conf": 0.9}]

    def box_source_2(nusc, event):
        return [{"bbox": [u2 - 5, v2 - 40, u2 + 5, v2], "cls_id": 2, "conf": 0.9}]

    nusc1, event1 = _build("pose1", box_source_1, ego_translation=(0.0, 0.0, 0.0))
    nusc2, event2 = _build("pose2", box_source_2, ego_translation=(1.0, 0.5, 0.0), ego_rotation=ep2_rotation)

    dets1 = camera_detections(nusc1, event1, box_source=box_source_1)
    dets2 = camera_detections(nusc2, event2, box_source=box_source_2)
    assert len(dets1) == 1 and len(dets2) == 1
    np.testing.assert_allclose(dets1[0].xy_global, dets2[0].xy_global, atol=1e-2)
    np.testing.assert_allclose(dets1[0].xy_global, target_global[:2], atol=1e-2)


# ── Test 5: cache -- identical on 2nd run; invalidates on model change ────

def test_cache_hit_is_deterministic_and_keyed_by_model(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)

    def box_source(nusc, event):
        return [{"bbox": [780.0, 400.0, 820.0, 700.0], "cls_id": 2, "conf": 0.9}]

    nusc, event = _build("cache_check", box_source)

    model_a = config.CAMERA_DETECTOR_MODEL.value
    path_a = _cache_path(model_a, event.sample_data_token)
    path_a.parent.mkdir(parents=True, exist_ok=True)
    path_a.write_text(json.dumps(box_source(None, None)), encoding="utf-8")

    dets_run1 = camera_detections(nusc, event)  # reads from disk cache, no box_source
    dets_run2 = camera_detections(nusc, event)
    assert len(dets_run1) == len(dets_run2) == 1
    np.testing.assert_allclose(dets_run1[0].xy_global, dets_run2[0].xy_global)

    # A different model key has no cache entry -- must NOT silently reuse
    # model_a's cached result (cache invalidates on model version change).
    path_b = _cache_path("yolov8s", event.sample_data_token)
    assert not path_b.exists()
    with pytest.raises(FileNotFoundError):
        camera_detections(nusc, event, cfg=_OverrideModelConfig(config, "yolov8s"))


class _OverrideModelConfig:
    """Thin wrapper presenting a different CAMERA_DETECTOR_MODEL.value
    without mutating the real, shared config module."""

    def __init__(self, real_cfg, model_name):
        self._real = real_cfg
        self._model_name = model_name

    def __getattr__(self, name):
        if name == "CAMERA_DETECTOR_MODEL":
            class _P:
                value = self._model_name
            return _P()
        return getattr(self._real, name)


# ── Test 6: truncated-bottom boxes dropped and counted ────────────────────

def test_truncated_bottom_boxes_are_dropped_and_counted():
    from ttcf.adapters.camera import process_camera_frame

    def box_source(nusc, event):
        return [
            {"bbox": [700.0, 400.0, 900.0, 800.0], "cls_id": 2, "conf": 0.9},  # normal
            {"bbox": [300.0, 500.0, 400.0, IMAGE_H - 1], "cls_id": 0, "conf": 0.8},  # truncated at bottom
        ]

    nusc, event = _build("truncated", box_source)
    result = process_camera_frame(nusc, event, box_source=box_source)
    assert result.n_raw_boxes == 2
    assert result.n_truncated_dropped == 1
    assert len(result.detections) == 1


# ── Test 7: R_global symmetric PSD ─────────────────────────────────────────

def test_r_global_symmetric_and_positive_definite():
    def box_source(nusc, event):
        return [{"bbox": [780.0, 400.0, 820.0, 700.0], "cls_id": 2, "conf": 0.9}]

    nusc, event = _build("r_check", box_source)
    dets = camera_detections(nusc, event, box_source=box_source)
    assert len(dets) == 1
    R = dets[0].R_global
    np.testing.assert_allclose(R, R.T, atol=1e-9)
    eigvals = np.linalg.eigvalsh(R)
    assert np.all(eigvals > 0)
