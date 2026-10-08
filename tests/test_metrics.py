"""
tests/test_metrics.py — step10_metrics.md §5.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf.evaluation.metrics import (
    EvalSplitGuardError,
    KeyframeGT,
    KeyframeSystem,
    guard_eval_scoring,
    keyframe_gt_tier,
    score_run,
    wilson_score_interval,
)
from ttcf.evaluation.report import ReportFootnoteError, build_footnotes, generate_four_run_table


# ── Test 1: hand-built toy logs -> exact expected counts and rates ───────

def test_hand_built_toy_logs_exact_counts_and_rates():
    gt_keyframes = [
        KeyframeGT("kf1", False, ["NONE"], [{}]),
        KeyframeGT("kf2", False, ["NONE"], [{}]),
        KeyframeGT("kf3", False, ["GRADUAL"], [{}]),
        KeyframeGT("kf4", False, ["AEB"], [{}]),
        KeyframeGT("kf5", False, ["AEB"], [{}]),
        KeyframeGT("kf6", False, ["GRADUAL"], [{}]),
    ]
    system = {
        "kf1": KeyframeSystem("kf1", "NONE"),
        "kf2": KeyframeSystem("kf2", "GRADUAL"),  # false brake
        "kf3": KeyframeSystem("kf3", "GRADUAL"),  # correct
        "kf4": KeyframeSystem("kf4", "NONE", no_estimate_reason="NOT_ELIGIBLE"),  # missed
        "kf5": KeyframeSystem("kf5", "GRADUAL"),  # under-brake (AEB warranted, only GRADUAL given)
        "kf6": KeyframeSystem("kf6", "AEB"),  # over-brake (GRADUAL warranted, AEB given)
    }

    m = score_run(gt_keyframes, system, run_name="toy")

    assert m.n_scored == 6
    assert m.correct_action_rate.numerator == 2  # kf1, kf3
    assert m.correct_action_rate.denominator == 6
    assert m.correct_action_rate.value == pytest.approx(2 / 6)
    assert m.false_brake_rate.numerator == 1
    assert m.false_brake_rate.denominator == 2  # kf1, kf2 have GT=NONE
    assert m.missed_brake_rate.numerator == 1
    assert m.missed_brake_rate.denominator == 4  # kf3-kf6 have GT!=NONE
    assert m.under_brake_count == 1
    assert m.over_brake_count == 1
    assert m.no_estimate_reason_breakdown == {"NOT_ELIGIBLE": 1}
    assert m.confusion[("AEB", "NONE")] == 1
    assert m.confusion[("AEB", "GRADUAL")] == 1
    assert m.confusion[("GRADUAL", "AEB")] == 1
    assert m.confusion[("NONE", "NONE")] == 1
    assert m.confusion[("NONE", "GRADUAL")] == 1
    assert m.confusion[("GRADUAL", "GRADUAL")] == 1


# ── Test 2: zero-denominator cells -> n/a ─────────────────────────────────

def test_zero_denominator_gives_na():
    # Every keyframe has GT=NONE -- no danger keyframes at all, so
    # missed-brake rate has denominator 0.
    gt_keyframes = [KeyframeGT("kf1", False, ["NONE"], [{}])]
    system = {"kf1": KeyframeSystem("kf1", "NONE")}
    m = score_run(gt_keyframes, system, run_name="toy")

    assert m.missed_brake_rate.value is None
    assert m.missed_brake_rate.denominator == 0
    assert m.missed_brake_rate.wilson_low is None


# ── Test 3: Wilson interval matches a reference implementation ───────────

def test_wilson_interval_matches_reference_values():
    # Published reference values for the 95% Wilson score interval.
    low, high = wilson_score_interval(50, 100)
    assert low == pytest.approx(0.404, abs=0.001)
    assert high == pytest.approx(0.596, abs=0.001)

    low0, high0 = wilson_score_interval(0, 10)
    assert low0 == pytest.approx(0.0, abs=1e-9)
    assert high0 == pytest.approx(0.2775, abs=0.001)

    assert wilson_score_interval(5, 0) == (None, None)


# ── Test 4: observable-restricted variant removes exactly the unobservable objects ─

def test_observable_restricted_variant_excludes_unobservable_objects():
    tiers = ["AEB", "NONE"]
    obs = [{"obs_lidar": False, "obs_any": True}, {"obs_lidar": True, "obs_any": True}]

    all_variant = keyframe_gt_tier(tiers, obs, obs_field=None)
    lidar_variant = keyframe_gt_tier(tiers, obs, obs_field="obs_lidar")

    assert all_variant == "AEB"  # the AEB object counts when unrestricted
    assert lidar_variant == "NONE"  # excluded when lidar couldn't have seen it


def test_score_run_with_observability_variant():
    gt_keyframes = [
        KeyframeGT(
            "kf1", False, ["AEB", "NONE"],
            [{"obs_lidar": False}, {"obs_lidar": True}],
        )
    ]
    system = {"kf1": KeyframeSystem("kf1", "NONE")}

    all_metrics = score_run(gt_keyframes, system, run_name="toy", obs_field=None)
    lidar_metrics = score_run(gt_keyframes, system, run_name="toy", observability_variant="obs_lidar", obs_field="obs_lidar")

    # Unrestricted: GT=AEB, system=NONE -> a missed brake.
    assert all_metrics.missed_brake_rate.numerator == 1
    # Lidar-restricted: the AEB object isn't observable by lidar, so
    # GT=NONE here -- system=NONE is now CORRECT, not missed.
    assert lidar_metrics.missed_brake_rate.denominator == 0
    assert lidar_metrics.correct_action_rate.numerator == 1


# ── Test 5: report generator raises if a footnote source is missing ──────

def test_report_generator_raises_on_missing_footnote_source():
    with pytest.raises(ReportFootnoteError):
        build_footnotes(
            radar_doppler_disabled=True,
            sensor_r_status={"lidar": "ASSUMED"},
            indicative_rows=[],
            class_policy=None,  # missing
            split_name="eval",
            tuning_split_only=True,
            distance_definition="nearest_surface_to_ego",
            n_gt_events=315,
            n_danger_events=11,
        )


def test_report_generator_raises_if_a_run_is_missing():
    gt_keyframes = [KeyframeGT("kf1", False, ["NONE"], [{}])]
    system = {"kf1": KeyframeSystem("kf1", "NONE")}
    m = score_run(gt_keyframes, system, run_name="lidar")
    with pytest.raises(ReportFootnoteError):
        generate_four_run_table({"lidar": {"all": m}}, footnotes={"x": "y"})  # missing radar/camera/fused


# ── Test 6: split guard ───────────────────────────────────────────────────

def test_split_guard_refuses_without_final(tmp_path):
    log_path = tmp_path / "eval_runs.log"
    frozen_path = tmp_path / "frozen_config.json"
    with pytest.raises(EvalSplitGuardError):
        guard_eval_scoring("eval", final=False, config_hash="abc", frozen_config_path=frozen_path,
                            log_path=log_path, run_name="r1")
    assert log_path.exists()
    assert "run=r1" in log_path.read_text(encoding="utf-8")
    assert "REFUSED" in log_path.read_text(encoding="utf-8")


def test_split_guard_refuses_on_hash_mismatch(tmp_path):
    log_path = tmp_path / "eval_runs.log"
    frozen_path = tmp_path / "frozen_config.json"
    frozen_path.write_text(json.dumps({"sha256": "the_real_hash"}), encoding="utf-8")

    with pytest.raises(EvalSplitGuardError):
        guard_eval_scoring("eval", final=True, config_hash="a_different_hash", frozen_config_path=frozen_path,
                            log_path=log_path, run_name="r2")

    with pytest.raises(EvalSplitGuardError):
        guard_eval_scoring("eval", final=False, config_hash="the_real_hash", frozen_config_path=frozen_path,
                            log_path=log_path, run_name="r3")

    log_lines = log_path.read_text(encoding="utf-8").strip().splitlines()
    assert len(log_lines) == 2  # every eval-split call is logged, pass or fail


def test_split_guard_passes_with_matching_hash(tmp_path):
    log_path = tmp_path / "eval_runs.log"
    frozen_path = tmp_path / "frozen_config.json"
    frozen_path.write_text(json.dumps({"sha256": "the_real_hash"}), encoding="utf-8")

    guard_eval_scoring("eval", final=True, config_hash="the_real_hash", frozen_config_path=frozen_path,
                        log_path=log_path, run_name="r4")  # must not raise
    assert "OK" in log_path.read_text(encoding="utf-8")


def test_split_guard_unrestricted_on_tune(tmp_path):
    log_path = tmp_path / "eval_runs.log"
    frozen_path = tmp_path / "does_not_exist.json"
    guard_eval_scoring("tune", final=False, config_hash="anything", frozen_config_path=frozen_path,
                        log_path=log_path, run_name="r5")  # must not raise
    assert not log_path.exists()  # tune-split calls are not eval-audit-logged


# ── Test 7: phantom-brake counting on an empty-corridor toy keyframe ─────

def test_phantom_brake_counting():
    gt_keyframes = [
        KeyframeGT("kf_empty_1", True),  # empty corridor, system stays quiet
        KeyframeGT("kf_empty_2", True),  # empty corridor, system phantom-brakes
        KeyframeGT("kf_real", False, ["NONE"], [{}]),
    ]
    system = {
        "kf_empty_1": KeyframeSystem("kf_empty_1", "NONE"),
        "kf_empty_2": KeyframeSystem("kf_empty_2", "GRADUAL"),
        "kf_real": KeyframeSystem("kf_real", "NONE"),
    }
    m = score_run(gt_keyframes, system, run_name="toy")

    assert m.phantom_brake_denominator == 2
    assert m.phantom_brake_count == 1
    assert m.n_scored == 1  # only the real (non-empty-corridor) keyframe is in E
    # false_brake_incl_phantom combines the primary false-brake denominator
    # (GT=NONE keyframes in E: kf_real) with the phantom denominator.
    assert m.false_brake_incl_phantom_rate.denominator == 1 + 2
    assert m.false_brake_incl_phantom_rate.numerator == 0 + 1
