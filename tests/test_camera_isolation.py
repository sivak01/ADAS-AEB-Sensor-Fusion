"""
tests/test_camera_isolation.py — step04c_camera_adapter.md §5 test 1: the
camera path must never import or touch LiDAR/radar/GT machinery (G14,
what-not-to-do.md §6) -- this is exactly V1's own `camera` mistake
(secretly using median LiDAR-point depth inside each 2D box, so its
"camera-only" result was actually LiDAR+camera fusion), made
structurally, not just procedurally, impossible to repeat here.
"""
from __future__ import annotations

import ast
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

CAMERA_MODULE_PATH = Path(__file__).resolve().parent.parent / "src" / "ttcf" / "adapters" / "camera.py"

FORBIDDEN_SYMBOLS = [
    "LidarPointCloud", "RadarPointCloud", "LIDAR_TOP", "RADAR_", "sample_annotation", "box_velocity",
]


def test_camera_module_source_has_no_forbidden_symbols():
    """Static scan of camera.py's own source TEXT (not just its imports --
    a forbidden symbol appearing anywhere, e.g. a string literal channel
    name, is just as real a leak) for exactly the symbols step04c §5
    test 1 names."""
    source = CAMERA_MODULE_PATH.read_text(encoding="utf-8")
    found = [sym for sym in FORBIDDEN_SYMBOLS if sym in source]
    assert not found, f"Forbidden symbol(s) found in camera.py: {found}"


def test_camera_module_imports_have_no_forbidden_modules():
    """AST-level check of camera.py's own import statements -- catches an
    indirect import (e.g. `from ttcf.adapters.lidar import X`) even if the
    bare symbol name never appears as a string literal."""
    tree = ast.parse(CAMERA_MODULE_PATH.read_text(encoding="utf-8"))
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported.append(node.module)
            imported.extend(alias.name for alias in node.names)
    forbidden_modules = ["ttcf.adapters.lidar", "ttcf.adapters.radar", "ttcf.gt"]
    found = [m for m in imported for f in forbidden_modules if f in m]
    assert not found, f"Forbidden module import(s) found in camera.py: {found}"


def test_adapter_produces_detections_with_only_camera_records_available():
    """Runs the real adapter with a loader that RAISES on any non-camera
    table access -- if the adapter secretly needed a LiDAR/radar file or
    a GT annotation, this fails loudly instead of silently working. It
    must still produce a detection (step04c §5 test 1's own requirement)."""
    from ttcf.adapters.camera import camera_detections
    from ttcf.types import RawEvent

    IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]
    # A real camera's own frame (x-right, y-down, z-forward) is a different
    # axis convention than ego's (x-forward, y-left, z-up) -- reusing
    # nuScenes' own real CAM_FRONT calibration quaternion here rather than
    # an identity rotation, which would not be physically valid.
    CAM_ROTATION_TYPICAL = [0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755]
    K = [[1000.0, 0.0, 800.0], [0.0, 1000.0, 450.0], [0.0, 0.0, 1.0]]
    cs_token, ep_token, sd_token = "cs_iso", "ep_iso", "sd_iso"
    tables = {
        "calibrated_sensor": {cs_token: {"translation": [0.0, 0.0, 1.5], "rotation": CAM_ROTATION_TYPICAL, "camera_intrinsic": K}},
        "ego_pose": {ep_token: {"translation": [0.0, 0.0, 0.0], "rotation": IDENTITY_Q}},
        "sample_data": {sd_token: {"height": 900, "width": 1600}},
    }

    class OnlyCameraNuScenes:
        def get(self, table, token):
            if table not in ("calibrated_sensor", "ego_pose", "sample_data"):
                raise AssertionError(f"Adapter tried to access a non-camera table: {table!r}")
            return tables[table][token]

    def fake_box_source(nusc, event):
        return [{"bbox": [700.0, 400.0, 900.0, 800.0], "cls_id": 2, "conf": 0.9}]  # a "car"

    event = RawEvent(
        t_us=0, channel="CAM_FRONT", modality="camera", sample_data_token=sd_token,
        ego_pose_token=ep_token, calibrated_sensor_token=cs_token, filename="fake.jpg",
        is_key_frame=True, scene_token="scene_0",
    )
    dets = camera_detections(OnlyCameraNuScenes(), event, box_source=fake_box_source)
    assert len(dets) == 1
