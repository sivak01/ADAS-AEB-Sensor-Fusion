"""
src/ttcf/ttc/action.py — STEP 08: debounce and action tiers (G16).

A single frame's TTC can be wrong (one bad match, one noisy return). Real
AEB systems require the danger condition to hold for N consecutive REAL
updates before acting -- this is the cheap, standard defence against
one-off errors, used INSTEAD OF long-horizon identity robustness (the
design contract's own stated tradeoff).

No precedent exists anywhere in V1/v2 for debounce or action-tier logic
(checked directly -- grep for debounce/ActionTier/consecutive across both
turned up only unrelated uses of the word "consecutive" describing ego
pose ordering). Neither V1 nor v2 ever built an actual decision/action
layer; both only tracked and evaluated positions. Built entirely fresh.

Public return type note: types.ActionDecision (the step00 contract) has
no way to represent "no usable estimate right now" separately from a
confident NONE (safe) decision -- but step08 §3 explicitly requires that
distinction ("NOT_ELIGIBLE results... are recorded as NO_ESTIMATE", "the
log records NO_ESTIMATE so evaluation can attribute later misses to
coverage rather than to logic"). Rather than redefine the already-
approved step00 ActionTier/ActionDecision contract, this module returns
its own ActionRecord (a strict superset: everything ActionDecision has,
plus the no_estimate flag) -- composition instead of modifying a shared
type other steps may already depend on.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
from typing import Optional

from ttcf import config
from ttcf.types import ActionTier, TTCReason, TTCResult


@dataclass(frozen=True)
class ActionRecord:
    t_us: int
    tier: ActionTier
    driving_track_id: Optional[str]
    consecutive_count: int
    no_estimate: bool  # True for NOT_ELIGIBLE updates and eviction fallbacks


@dataclass
class _TierState:
    active: bool = False
    danger_streak: int = 0
    safe_streak: int = 0
    first_danger_t_us: Optional[int] = None

    def reset(self) -> None:
        self.active = False
        self.danger_streak = 0
        self.safe_streak = 0
        self.first_danger_t_us = None


@dataclass(frozen=True)
class LatencyRecord:
    tier: ActionTier
    track_id: str
    n_updates: int  # always DEBOUNCE_N by construction -- fires exactly at the Nth
    first_danger_t_us: int
    fired_t_us: int
    latency_ms: float


class ActionTierDebouncer:
    """Input: one call to process() per REAL update of the critical track
    (step07 Part B's critical_object() output). Pass result=None when
    there is no critical object this update (nothing path-relevant, or
    the previous driving track was evicted) -- prediction-only ticks
    must never be passed in at all (§4)."""

    def __init__(self, cfg=config):
        self._cfg = cfg
        self._driving_track_id: Optional[str] = None
        self._tiers: dict[ActionTier, _TierState] = {
            ActionTier.GRADUAL: _TierState(),
            ActionTier.AEB: _TierState(),
        }
        self._log: list[ActionRecord] = []
        self._log_t_us: list[int] = []
        self.latencies: list[LatencyRecord] = []

    @property
    def log(self) -> list[ActionRecord]:
        return list(self._log)

    def _reset_all(self) -> None:
        for tier_state in self._tiers.values():
            tier_state.reset()

    def _current_active_tier(self) -> ActionTier:
        # Highest satisfied tier wins -- AEB is strictly more severe than
        # GRADUAL (TTC_AEB_S < TTC_GRADUAL_S), so check it first.
        if self._tiers[ActionTier.AEB].active:
            return ActionTier.AEB
        if self._tiers[ActionTier.GRADUAL].active:
            return ActionTier.GRADUAL
        return ActionTier.NONE

    def _record(self, t_us: int, no_estimate: bool) -> ActionRecord:
        active_tier = self._current_active_tier()
        # consecutive_count reflects the current IN-PROGRESS danger streak
        # (whichever tier's condition currently holds), not just the
        # streak of a tier that has already fired -- so a caller can see
        # incremental progress toward firing (1, 2, ..., N), not just 0
        # right up until the instant it fires.
        count = max((state.danger_streak for state in self._tiers.values()), default=0)
        record = ActionRecord(
            t_us=t_us,
            tier=active_tier,
            driving_track_id=self._driving_track_id,
            consecutive_count=count,
            no_estimate=no_estimate,
        )
        self._log.append(record)
        self._log_t_us.append(t_us)
        return record

    def process(self, t_us: int, result: Optional[TTCResult]) -> ActionRecord:
        t_us = int(t_us)

        # No critical object this update: eviction, or nothing path-relevant.
        if result is None:
            self._reset_all()
            self._driving_track_id = None
            return self._record(t_us, no_estimate=True)

        # Critical track changed identity -- deliberate, conservative
        # reset (§4: never debounce across different tracks).
        if result.track_id != self._driving_track_id:
            self._reset_all()
            self._driving_track_id = result.track_id

        # NOT_ELIGIBLE: neither increments nor resets anything (§3).
        if result.reason == TTCReason.NOT_ELIGIBLE:
            return self._record(t_us, no_estimate=True)

        gradual_gate = self._cfg.TTC_GRADUAL_S.value
        aeb_gate = self._cfg.TTC_AEB_S.value
        conditions = {
            ActionTier.GRADUAL: result.ttc_s <= gradual_gate,
            ActionTier.AEB: result.ttc_s <= aeb_gate,
        }

        for tier, holds in conditions.items():
            state = self._tiers[tier]
            if holds:
                if state.danger_streak == 0:
                    state.first_danger_t_us = t_us
                state.danger_streak += 1
                state.safe_streak = 0
                if not state.active and state.danger_streak >= self._cfg.DEBOUNCE_N.value:
                    state.active = True
                    assert state.first_danger_t_us is not None
                    self.latencies.append(
                        LatencyRecord(
                            tier=tier,
                            track_id=result.track_id,
                            n_updates=state.danger_streak,
                            first_danger_t_us=state.first_danger_t_us,
                            fired_t_us=t_us,
                            latency_ms=(t_us - state.first_danger_t_us) / 1000.0,
                        )
                    )
            else:
                state.safe_streak += 1
                state.danger_streak = 0
                state.first_danger_t_us = None
                if state.active and state.safe_streak >= self._cfg.DEBOUNCE_N.value:
                    state.active = False

        return self._record(t_us, no_estimate=False)

    def action_at(self, t_us: int) -> Optional[ActionRecord]:
        """Latest decision with timestamp <= t_us, or None if nothing has
        been processed yet at or before t_us."""
        idx = bisect.bisect_right(self._log_t_us, int(t_us))
        if idx == 0:
            return None
        return self._log[idx - 1]
