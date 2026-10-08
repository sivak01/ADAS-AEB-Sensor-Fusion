"""
tests/test_action.py — step08_debounce_action.md §5 (synthetic streams only).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config
from ttcf.ttc.action import ActionTierDebouncer
from ttcf.types import ActionTier, TTCReason, TTCResult

N = config.DEBOUNCE_N.value  # 3
GRADUAL_S = config.TTC_GRADUAL_S.value  # 2.5
AEB_S = config.TTC_AEB_S.value  # 1.2
SAFE_TTC = 10.0


def _result(t_us, ttc_s, track_id="trk_A", reason=TTCReason.OK):
    return TTCResult(
        t_us=t_us, track_id=track_id, distance_m=10.0, closing_speed_mps=5.0,
        ttc_s=ttc_s, reason=reason, closing_std_mps=0.1,
    )


# ── Test 1: one bad frame must not trigger ────────────────────────────────

def test_single_bad_frame_does_not_trigger():
    deb = ActionTierDebouncer()
    t = 0
    for ttc in (SAFE_TTC, SAFE_TTC, AEB_S - 0.1, SAFE_TTC, SAFE_TTC):
        record = deb.process(t, _result(t, ttc))
        t += 100_000
    assert record.tier == ActionTier.NONE


# ── Test 2: DEBOUNCE_N consecutive dangerous updates fires at exactly N ──

def test_fires_at_exactly_nth_dangerous_update():
    deb = ActionTierDebouncer()
    t = 0
    records = []
    for _ in range(N):
        records.append(deb.process(t, _result(t, AEB_S - 0.1)))
        t += 100_000
    for i, rec in enumerate(records):
        if i < N - 1:
            assert rec.tier == ActionTier.NONE, f"fired too early at update {i}"
        else:
            assert rec.tier == ActionTier.AEB
            assert rec.consecutive_count == N


# ── Test 3: critical-track id change resets the counter ──────────────────

def test_track_id_change_resets_counter():
    deb = ActionTierDebouncer()
    t = 0
    deb.process(t, _result(t, AEB_S - 0.1, track_id="trk_A")); t += 100_000
    deb.process(t, _result(t, AEB_S - 0.1, track_id="trk_A")); t += 100_000
    # Third dangerous update, but a DIFFERENT track -- must not fire.
    # The id change resets the streak, then this same call's own
    # dangerous condition starts a fresh streak at 1 (not 0) -- it IS
    # trk_B's first real dangerous update.
    record = deb.process(t, _result(t, AEB_S - 0.1, track_id="trk_B"))
    assert record.tier == ActionTier.NONE
    assert record.consecutive_count == 1


# ── Test 4: release requires N consecutive safe updates ──────────────────

def test_release_requires_n_consecutive_safe_updates():
    deb = ActionTierDebouncer()
    t = 0
    for _ in range(N):
        rec = deb.process(t, _result(t, AEB_S - 0.1)); t += 100_000
    assert rec.tier == ActionTier.AEB

    for i in range(N - 1):
        rec = deb.process(t, _result(t, SAFE_TTC)); t += 100_000
        assert rec.tier == ActionTier.AEB, f"released too early at safe update {i}"

    rec = deb.process(t, _result(t, SAFE_TTC))
    assert rec.tier == ActionTier.NONE


# ── Test 5: AEB supersedes GRADUAL when both hold ─────────────────────────

def test_aeb_supersedes_gradual():
    deb = ActionTierDebouncer()
    t = 0
    for _ in range(N):
        rec = deb.process(t, _result(t, AEB_S - 0.1))  # satisfies BOTH gates
        t += 100_000
    assert rec.tier == ActionTier.AEB


# ── Test 6: NOT_ELIGIBLE neither triggers nor resets ──────────────────────

def test_not_eligible_freezes_state():
    deb = ActionTierDebouncer()
    t = 0
    deb.process(t, _result(t, AEB_S - 0.1)); t += 100_000  # streak=1
    deb.process(t, _result(t, AEB_S - 0.1)); t += 100_000  # streak=2

    frozen = deb.process(t, _result(t, float("inf"), reason=TTCReason.NOT_ELIGIBLE))
    t += 100_000
    assert frozen.no_estimate is True
    assert frozen.tier == ActionTier.NONE  # nothing active yet

    fires = deb.process(t, _result(t, AEB_S - 0.1))  # the 3rd REAL dangerous update
    assert fires.tier == ActionTier.AEB
    assert fires.consecutive_count == N


# ── Test 7: eviction returns to NONE with a NO_ESTIMATE record ───────────

def test_eviction_returns_to_none_with_no_estimate():
    deb = ActionTierDebouncer()
    t = 0
    for _ in range(N):
        deb.process(t, _result(t, AEB_S - 0.1)); t += 100_000

    record = deb.process(t, None)  # driving track evicted / nothing critical
    assert record.tier == ActionTier.NONE
    assert record.no_estimate is True
    assert record.driving_track_id is None

    # A fresh dangerous streak afterward starts from zero (deliberate reset).
    t += 100_000
    rec = deb.process(t, _result(t, AEB_S - 0.1))
    assert rec.tier == ActionTier.NONE
    assert rec.consecutive_count == 1


# ── Test 8: latency = (N-1) x inter-arrival for a regular stream ─────────

def test_latency_equals_n_minus_one_times_interarrival():
    deb = ActionTierDebouncer()
    dt_us = 100_000
    t = 0
    for _ in range(N):
        deb.process(t, _result(t, AEB_S - 0.1))
        t += dt_us
    # ttc = AEB_S - 0.1 satisfies BOTH the AEB and GRADUAL gates at once,
    # so both tiers independently reach DEBOUNCE_N on the same update and
    # both get a latency record -- not just AEB.
    assert len(deb.latencies) == 2
    assert {lat.tier for lat in deb.latencies} == {ActionTier.AEB, ActionTier.GRADUAL}
    expected_ms = (N - 1) * dt_us / 1000.0
    for lat in deb.latencies:
        assert lat.n_updates == N
        assert lat.latency_ms == pytest.approx(expected_ms)


# ── Test 9: action_at(t) between updates ──────────────────────────────────

def test_action_at_between_updates():
    deb = ActionTierDebouncer()
    deb.process(0, _result(0, SAFE_TTC))
    deb.process(200_000, _result(200_000, SAFE_TTC))

    assert deb.action_at(0).t_us == 0
    assert deb.action_at(100_000).t_us == 0  # between updates -> the earlier one
    assert deb.action_at(200_000).t_us == 200_000
    assert deb.action_at(300_000).t_us == 200_000  # after the last update
    assert deb.action_at(-1) is None  # before anything happened


# ── Evidence: timeline figure (TTC trace + action decisions), step08 §8 ──

def test_timeline_figure_ttc_trace_and_action_decisions():
    """Not a correctness test -- generates the required report figure:
    a closing-then-opening synthetic TTC profile, showing the debounced
    action tier lagging the raw TTC crossing by DEBOUNCE_N updates on
    both the trigger and the release side."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dt_us = 100_000
    # V-shaped TTC profile with a HOLD phase at the bottom: closes from 8s
    # down to 1.0s, holds at 0.8s for several updates, then opens back up
    # to 8s. (A first version without the hold only spent 2 consecutive
    # updates below TTC_AEB_S=1.2 -- one short of DEBOUNCE_N=3 -- so AEB
    # never fired at all; fixed by widening the dip, not by lowering N.)
    closing = np.linspace(8.0, 1.0, 10)
    hold = np.full(6, 0.8)
    opening = np.linspace(1.0, 8.0, 10)
    ttc_profile = np.concatenate([closing, hold, opening])

    n = len(ttc_profile)
    deb = ActionTierDebouncer()
    t_values, tiers = [], []
    for i, ttc in enumerate(ttc_profile):
        t_us = i * dt_us
        rec = deb.process(t_us, _result(t_us, float(ttc)))
        t_values.append(t_us / 1e6)
        tiers.append(rec.tier.value)

    tier_rank = {"NONE": 0, "GRADUAL": 1, "AEB": 2}
    tier_y = [tier_rank[t] for t in tiers]

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t_values, ttc_profile, color="tab:blue", marker=".")
    axes[0].axhline(GRADUAL_S, color="orange", linestyle="--", label=f"TTC_GRADUAL_S={GRADUAL_S}")
    axes[0].axhline(AEB_S, color="red", linestyle="--", label=f"TTC_AEB_S={AEB_S}")
    axes[0].set_ylabel("TTC (s)")
    axes[0].set_title("Step 08: TTC trace vs. debounced action decision")
    axes[0].legend(fontsize=8)

    axes[1].step(t_values, tier_y, where="post", color="black")
    axes[1].set_yticks([0, 1, 2])
    axes[1].set_yticklabels(["NONE", "GRADUAL", "AEB"])
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("action tier")
    fig.tight_layout()

    out_dir = Path("outputs/figs/step08")
    out_dir.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_dir / "timeline.png", dpi=130)
    plt.close(fig)

    # The debounced tier must lag the raw threshold crossing by DEBOUNCE_N
    # updates on the way in, and release DEBOUNCE_N updates after TTC
    # clears the threshold on the way out -- not fire/release instantly.
    first_aeb_raw = next(i for i, v in enumerate(ttc_profile) if v <= AEB_S)
    first_aeb_debounced = next(i for i, v in enumerate(tiers) if v == "AEB")
    assert first_aeb_debounced == first_aeb_raw + (N - 1)
