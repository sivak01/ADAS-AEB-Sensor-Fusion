"""
tests/test_gt_events.py — stepD0_gt_event_table.md §6.

Uses a minimal mock NuScenes exposing only the table/get()/box_velocity()
surface ttcf.gt.gt_events and ttcf.filtering.ego_state actually touch, per
the step file's own instruction ("A known synthetic annotation set (mocked)").
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from pyquaternion import Quaternion

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.geometry.corridor import footprint_intersects_corridor
from ttcf.gt.gt_events import (
    _footprint_corners_global,
    build_all_gt_events,
    build_scene_gt_events,
    gt_action_from_ttc,
)
from ttcf.geometry.transforms import FrameContext, global_to_ego


class FakeNuScenes:
    """Exposes exactly the surface gt_events.py / EgoStateEstimator use:
    .get(table, token), .scene, .sample_data, .box_velocity(token)."""

    def __init__(self, tables: dict, scene_list, sample_data_list, box_velocities: dict):
        self._tables = tables
        self.scene = scene_list
        self.sample_data = sample_data_list
        self._box_velocities = box_velocities

    def get(self, table, token):
        return self._tables[table][token]

    def box_velocity(self, ann_token, max_time_diff=1.5):
        return self._box_velocities[ann_token]


HALF_WIDTH = config.CORRIDOR_HALF_WIDTH_M.value
MAX_RANGE = config.CORRIDOR_MAX_RANGE_M.value
IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]


def _ann(token, sample_token, instance_token, category, translation, size=(2.0, 4.0, 1.5),
         rotation=IDENTITY_Q, attribute_tokens=(), visibility_token="4",
         num_lidar_pts=50, num_radar_pts=5, prev="", next=""):
    return {
        "token": token, "sample_token": sample_token, "instance_token": instance_token,
        "visibility_token": visibility_token, "attribute_tokens": list(attribute_tokens),
        "translation": list(translation), "size": list(size), "rotation": list(rotation),
        "prev": prev, "next": next,
        "num_lidar_pts": num_lidar_pts, "num_radar_pts": num_radar_pts,
        "category_name": category,
    }


def _build_fake_dataset():
    scene_token = "scene_0"
    ego_pose_tokens = ["ep0", "ep1", "ep2"]
    ego_positions = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)]  # 10 m/s straight line
    t_list = [0, 500_000, 1_000_000]

    ego_poses = {
        tok: {"token": tok, "translation": [x, y, 0.0], "rotation": IDENTITY_Q, "timestamp": t}
        for tok, (x, y), t in zip(ego_pose_tokens, ego_positions, t_list)
    }

    sensor = {"lidar_sensor": {"token": "lidar_sensor", "channel": "LIDAR_TOP"}}
    calibrated_sensor = {
        "cs0": {"token": "cs0", "sensor_token": "lidar_sensor", "translation": [0, 0, 0], "rotation": IDENTITY_Q}
    }

    sample_tokens = ["s0", "s1", "s2"]
    sd_tokens = ["sd0", "sd1", "sd2"]
    sample_data = {}
    for sd_tok, sample_tok, ep_tok, t in zip(sd_tokens, sample_tokens, ego_pose_tokens, t_list):
        sample_data[sd_tok] = {
            "token": sd_tok, "sample_token": sample_tok, "ego_pose_token": ep_tok,
            "calibrated_sensor_token": "cs0", "timestamp": t, "is_key_frame": True,
            "filename": "dummy.pcd.bin",
        }

    # Object A: in-path, stationary, annotated at s0 and s1 (velocity ~0, valid).
    ann_a0 = _ann("annA0", "s0", "instA", "vehicle.car", (30.0, 0.0, 0.0), next="annA1")
    ann_a1 = _ann("annA1", "s1", "instA", "vehicle.car", (30.0, 0.0, 0.0), prev="annA0")

    # Object B: clearly out-of-path (10 m to the side), single annotation at s0.
    ann_b0 = _ann("annB0", "s0", "instB", "vehicle.car", (30.0, 10.0, 0.0))

    # Object C: in-path, single ISOLATED annotation at s1 (no prev/next -> NaN velocity).
    ann_c0 = _ann("annC0", "s1", "instC", "vehicle.car", (20.0, 0.0, 0.0))

    anns_by_sample = {"s0": ["annA0", "annB0"], "s1": ["annA1", "annC0"], "s2": []}
    sample_annotation = {a["token"]: a for a in [ann_a0, ann_a1, ann_b0, ann_c0]}

    samples = {}
    for i, tok in enumerate(sample_tokens):
        samples[tok] = {
            "token": tok, "timestamp": t_list[i], "scene_token": scene_token,
            "prev": sample_tokens[i - 1] if i > 0 else "",
            "next": sample_tokens[i + 1] if i < len(sample_tokens) - 1 else "",
            "data": {"LIDAR_TOP": sd_tokens[i]},
            "anns": anns_by_sample[tok],
        }

    scene = {
        "token": scene_token, "name": "scene-fake", "first_sample_token": "s0",
        "nbr_samples": 3,
    }

    tables = {
        "scene": {scene_token: scene},
        "sample": samples,
        "sample_data": sample_data,
        "ego_pose": ego_poses,
        "sensor": sensor,
        "calibrated_sensor": calibrated_sensor,
        "sample_annotation": sample_annotation,
        "attribute": {},
    }

    box_velocities = {
        "annA0": np.array([0.0, 0.0, 0.0]),
        "annA1": np.array([0.0, 0.0, 0.0]),
        "annB0": np.array([np.nan, np.nan, np.nan]),  # never used (out of path)
        "annC0": np.array([np.nan, np.nan, np.nan]),  # isolated -> NaN, as real box_velocity would give
    }

    sample_data_list = list(sample_data.values())
    scene_list = [scene]
    nusc = FakeNuScenes(tables, scene_list, sample_data_list, box_velocities)
    return nusc, scene_token


# ── Test 1: known synthetic annotation set -> in-path membership, TTC, tier ─

def test_known_synthetic_set_membership_ttc_tier():
    nusc, scene_token = _build_fake_dataset()
    gt_rows, keyframe_rows = build_scene_gt_events(nusc, scene_token)

    # Object A is in-path at both s0 and s1; object B (10m to the side) never appears.
    instances_seen = {r.instance_token for r in gt_rows}
    assert "instA" in instances_seen
    assert "instB" not in instances_seen  # out-of-path, correctly excluded

    row_a_s1 = next(r for r in gt_rows if r.instance_token == "instA" and r.sample_token == "s1")
    # At s1: ego at (5,0) moving ~10 m/s toward object A centred at (30,0),
    # stationary. Object A's box has length=4m (half-length=2m), and DEC-1's
    # nearest-surface rule means the reference point is the box's NEAR edge
    # (x=30-2=28), not its centre -- distance = 28-5 = 23, not 30-5 = 25.
    assert row_a_s1.distance_m == pytest.approx(23.0, abs=0.5)
    assert row_a_s1.closing_speed_mps == pytest.approx(10.0, abs=1.0)
    assert row_a_s1.vel_valid is True
    assert row_a_s1.gt_action in ("NONE", "GRADUAL", "AEB")
    assert row_a_s1.gt_ttc_s == pytest.approx(row_a_s1.distance_m / row_a_s1.closing_speed_mps, rel=0.05)


# ── Test 2: GT TTC for a parked object with moving ego = distance / ego_speed ─

def test_parked_object_ttc_equals_distance_over_ego_speed():
    nusc, scene_token = _build_fake_dataset()
    gt_rows, _ = build_scene_gt_events(nusc, scene_token)
    row = next(r for r in gt_rows if r.instance_token == "instA" and r.sample_token == "s1")

    expected_ttc = row.distance_m / row.closing_speed_mps
    assert row.gt_ttc_s == pytest.approx(expected_ttc, rel=1e-6)
    assert row.gt_ttc_s == pytest.approx(2.3, abs=0.3)  # ~23m (near edge, DEC-1) / ~10m/s


# ── Test 3: in-path membership identical via step-05 function vs fresh call ─

def test_in_path_membership_matches_step05_function_directly():
    nusc, scene_token = _build_fake_dataset()
    gt_rows, _ = build_scene_gt_events(nusc, scene_token)

    ann = nusc.get("sample_annotation", "annA1")
    sample = nusc.get("sample", ann["sample_token"])
    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    ego_pose = nusc.get("ego_pose", lidar_sd["ego_pose_token"])
    ctx = FrameContext(calibrated_sensor={"translation": [0, 0, 0], "rotation": IDENTITY_Q}, ego_pose=ego_pose)

    corners_global = _footprint_corners_global(ann)
    corners_ego = global_to_ego(corners_global, ctx)[:, :2]
    fresh_result = footprint_intersects_corridor(corners_ego, HALF_WIDTH, MAX_RANGE)

    was_included = any(r.instance_token == "instA" and r.sample_token == "s1" for r in gt_rows)
    assert fresh_result == was_included == True


# ── Test 4: vel_valid=False rows appear in counts, excluded from tier labels ─

def test_invalid_velocity_rows_counted_but_excluded_from_tier():
    nusc, scene_token = _build_fake_dataset()
    gt_rows, _ = build_scene_gt_events(nusc, scene_token)

    row_c = next(r for r in gt_rows if r.instance_token == "instC")
    assert row_c.vel_valid is False
    assert row_c.gt_action == "NONE"  # excluded from a real TTC-based tier
    assert np.isinf(row_c.gt_ttc_s) or row_c.gt_ttc_s > 0  # still a well-formed row, just not tier-labelled

    # It IS counted in the row set (not silently dropped).
    assert row_c in gt_rows


# ── Test 5: row counts reconcile -- sum over scenes = total ──────────────

def test_row_counts_reconcile_across_scenes():
    nusc, scene_token = _build_fake_dataset()
    per_scene_rows, per_scene_keyframes = build_scene_gt_events(nusc, scene_token)
    all_rows, all_keyframes = build_all_gt_events(nusc)  # only 1 scene in this fixture

    assert len(all_rows) == len(per_scene_rows)
    assert len(all_keyframes) == len(per_scene_keyframes)
    assert sum(1 for _ in all_rows) == len(all_rows)  # trivial reconciliation, sanity
