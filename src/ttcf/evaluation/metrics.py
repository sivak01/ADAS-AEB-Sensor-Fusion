"""
src/ttcf/evaluation/metrics.py — STEP 10: event-level metrics (G11, G16).

Scores the system the way an AEB engineer would: at each moment something
real is in the path, did the system take the right action? No MOT
metrics, no full-track MAE/RMSE.

Precedent check: v2's evaluate.py (compute_sensor_metrics,
compute_length_bucket_metrics, compute_multi_sensor_composition) is
entirely full-track MAE/RMSE and lifetime match-rate based -- exactly
what what-not-to-do.md §1 and this step's own §4 forbid reusing. No
event-level scoring, confusion matrix, or Wilson-interval precedent
exists anywhere in V1/v2. This module is built entirely fresh. The one
thing worth carrying over in SPIRIT, not code: v2's own discipline of
always reporting a raw count (n_matched_pairs) alongside a rate (MAE) --
this project's own "always report n/a over a hidden 0, always show raw
counts" rule (§3.2) is the same honesty principle taken further.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

_SEVERITY = {"NONE": 0, "GRADUAL": 1, "AEB": 2}
_TIER_NAMES = {v: k for k, v in _SEVERITY.items()}
_Z_95 = 1.959963984540054  # scipy.stats.norm.ppf(0.975), hardcoded so this module has no scipy dependency


def wilson_score_interval(k: int, n: int, z: float = _Z_95) -> tuple:
    """95% Wilson score interval for k successes out of n trials.
    Returns (None, None) for n == 0 -- there is no interval for an
    undefined rate, and this module never reports 0 in that case (§3.2:
    "If a denominator is 0, report n/a -- never 0 or NaN silently")."""
    if n == 0:
        return None, None
    p_hat = k / n
    denom = 1 + z * z / n
    center = (p_hat + z * z / (2 * n)) / denom
    margin = (z * math.sqrt(p_hat * (1 - p_hat) / n + z * z / (4 * n * n))) / denom
    return max(0.0, center - margin), min(1.0, center + margin)


@dataclass(frozen=True)
class RateResult:
    value: Optional[float]  # None means "n/a"
    numerator: int
    denominator: int
    wilson_low: Optional[float]
    wilson_high: Optional[float]
    indicative_only: bool  # True if denominator < MIN_COUNT_FOR_RATE


def _rate(numerator: int, denominator: int, min_count_for_rate: int) -> RateResult:
    if denominator == 0:
        return RateResult(None, numerator, denominator, None, None, indicative_only=False)
    low, high = wilson_score_interval(numerator, denominator)
    return RateResult(
        value=numerator / denominator,
        numerator=numerator,
        denominator=denominator,
        wilson_low=low,
        wilson_high=high,
        indicative_only=denominator < min_count_for_rate,
    )


def keyframe_gt_tier(object_tiers, observability_flags=None, obs_field: Optional[str] = None) -> str:
    """Most severe tier among a keyframe's in-path GT objects.
    object_tiers: list[str] ("NONE"/"GRADUAL"/"AEB"), one per in-path GT object.
    observability_flags: list[dict], aligned with object_tiers, each e.g.
        {"obs_lidar": True, "obs_radar": False, "obs_camera": True, "obs_any": True}.
    obs_field: if given (e.g. "obs_lidar"), only objects with that flag
        True count toward the tier -- the observable-restricted variant
        (step10 §3.3). None (default) = all GT events, unrestricted."""
    best = 0
    for i, tier in enumerate(object_tiers):
        if obs_field is not None:
            flags = observability_flags[i] if observability_flags else {}
            if not flags.get(obs_field, False):
                continue
        best = max(best, _SEVERITY.get(tier, 0))
    return _TIER_NAMES[best]


@dataclass(frozen=True)
class KeyframeGT:
    """One keyframe's GT side, from stepD0: which in-path objects (with
    their tiers and observability flags), or empty-corridor."""

    sample_token: str
    is_empty_corridor: bool
    object_tiers: list = field(default_factory=list)
    object_observability: list = field(default_factory=list)  # aligned dicts, see keyframe_gt_tier


@dataclass(frozen=True)
class KeyframeSystem:
    """One keyframe's system side, from step09's system_view_at()."""

    sample_token: str
    tier: str  # "NONE"/"GRADUAL"/"AEB"
    no_estimate_reason: Optional[str] = None  # only meaningful when tier == "NONE"


@dataclass(frozen=True)
class RunMetrics:
    run_name: str
    observability_variant: str  # "all" or an obs_field name
    n_scored: int  # |E| for this variant
    confusion: dict  # (gt_tier, system_tier) -> count, all 9 cells present
    correct_action_rate: RateResult
    false_brake_rate: RateResult
    missed_brake_rate: RateResult
    under_brake_count: int
    over_brake_count: int
    phantom_brake_count: int
    phantom_brake_denominator: int
    false_brake_incl_phantom_rate: RateResult
    no_estimate_reason_breakdown: dict  # reason -> count, missed-brakes only


def score_run(
    gt_keyframes: list,
    system_by_sample: dict,
    run_name: str,
    observability_variant: str = "all",
    obs_field: Optional[str] = None,
    min_count_for_rate: int = 10,
) -> RunMetrics:
    """gt_keyframes: list[KeyframeGT], every keyframe (both in-path and
    empty-corridor) for the scene(s) being scored.
    system_by_sample: sample_token -> KeyframeSystem.
    obs_field: None for the "all GT events" variant, else e.g. "obs_lidar"
    for the observable-restricted variant (§3.3)."""
    confusion = {(g, s): 0 for g in _SEVERITY for s in _SEVERITY}
    correct = missed = under = false = over = 0
    n_gt_none = n_gt_danger = 0
    phantom = 0
    phantom_denom = 0
    no_estimate_reasons: dict = {}
    n_scored = 0

    for kf in gt_keyframes:
        sys_kf = system_by_sample.get(kf.sample_token)
        sys_tier = sys_kf.tier if sys_kf is not None else "NONE"

        if kf.is_empty_corridor:
            phantom_denom += 1
            if sys_tier != "NONE":
                phantom += 1
            continue

        # Primary set E: keyframes with >=1 in-path GT object (DEC-4).
        gt_tier = keyframe_gt_tier(kf.object_tiers, kf.object_observability, obs_field)
        n_scored += 1
        confusion[(gt_tier, sys_tier)] += 1

        g, s = _SEVERITY[gt_tier], _SEVERITY[sys_tier]
        if s == g:
            correct += 1
        if g == 0:
            n_gt_none += 1
            if s != 0:
                false += 1
        else:
            n_gt_danger += 1
            if s == 0:
                missed += 1
                reason = sys_kf.no_estimate_reason if sys_kf is not None else "NO_TRACK"
                no_estimate_reasons[reason] = no_estimate_reasons.get(reason, 0) + 1
            elif s < g:
                under += 1
            elif s > g:
                over += 1

    correct_rate = _rate(correct, n_scored, min_count_for_rate)
    false_rate = _rate(false, n_gt_none, min_count_for_rate)
    missed_rate = _rate(missed, n_gt_danger, min_count_for_rate)
    false_incl_phantom = _rate(false + phantom, n_gt_none + phantom_denom, min_count_for_rate)

    return RunMetrics(
        run_name=run_name,
        observability_variant=observability_variant,
        n_scored=n_scored,
        confusion=confusion,
        correct_action_rate=correct_rate,
        false_brake_rate=false_rate,
        missed_brake_rate=missed_rate,
        under_brake_count=under,
        over_brake_count=over,
        phantom_brake_count=phantom,
        phantom_brake_denominator=phantom_denom,
        false_brake_incl_phantom_rate=false_incl_phantom,
        no_estimate_reason_breakdown=no_estimate_reasons,
    )


# ── Split guard (step10 §3.5) ─────────────────────────────────────────────


class EvalSplitGuardError(PermissionError):
    pass


def guard_eval_scoring(
    split: str,
    final: bool,
    config_hash: Optional[str],
    frozen_config_path: Path,
    log_path: Path = Path("outputs/eval_runs.log"),
    run_name: str = "unnamed",
) -> None:
    """Refuses to score `eval` scenes unless final=True AND config_hash
    equals the hash stored in frozen_config_path. Scoring `tune` is
    unrestricted. Every eval-split invocation (pass or fail) is appended
    to log_path for an audit trail, logged BEFORE raising on failure."""
    if split != "eval":
        return

    import json
    from datetime import datetime, timezone

    frozen_hash = None
    if frozen_config_path.exists():
        try:
            frozen_hash = json.loads(frozen_config_path.read_text(encoding="utf-8")).get("sha256")
        except (json.JSONDecodeError, OSError):
            frozen_hash = None

    ok = final and frozen_hash is not None and config_hash == frozen_hash

    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "a", encoding="utf-8") as f:
        f.write(
            f"{datetime.now(timezone.utc).isoformat()}\trun={run_name}\tfinal={final}\t"
            f"config_hash={config_hash}\tfrozen_hash={frozen_hash}\tresult={'OK' if ok else 'REFUSED'}\n"
        )

    if not final:
        raise EvalSplitGuardError(
            f"Refusing to score the eval split for run '{run_name}': --final was not passed."
        )
    if frozen_hash is None:
        raise EvalSplitGuardError(
            f"Refusing to score the eval split for run '{run_name}': "
            f"no frozen config found at {frozen_config_path} (configs/frozen_config.json is written at step11)."
        )
    if config_hash != frozen_hash:
        raise EvalSplitGuardError(
            f"Refusing to score the eval split for run '{run_name}': "
            f"config hash {config_hash} does not match the frozen config hash {frozen_hash}."
        )
