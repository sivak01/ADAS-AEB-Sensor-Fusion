"""
scripts/nis_check.py — post-step11, G17 NIS filter sanity check, TUNE SPLIT ONLY.

Prompted by the predecessor project's own methodology (`F:\\Sensor fusion
Research`, reviewed 2026-10-08, docs/decisions.md): a filter is well-
calibrated when its own reported uncertainty (P, R) matches its real
error -- checked via NIS (Normalized Innovation Squared), the exact same
quantity this project's tracker already computes for every REAL matched
update as `AssociationLogEntry.d2` (step06's own S = P_pred + R
Mahalanobis distance, tracking/tracker.py). No new tracker instrumentation
needed -- this script just runs the real pipeline (reusing
run_pipeline.run_real_stream unchanged) and analyses the association log
it already produces.

Theory (2 DOF -- NIS is always position-only, x/y, regardless of whether
a given update's own H carried extra channels under RADAR_DOPPLER_ABLATION
-- gating is never conditioned on measurement channels, kalman.py's own
mahalanobis_sq/innovation_cov are unchanged by G17): a consistent filter's
real matched-update NIS values are chi2(df=2)-distributed --
mean=2.0, median~=1.386, and config.CHI2_GATE (~=9.21, the 99% threshold)
should be exceeded by ~1% of them. Systematically high NIS => filter is
OVERCONFIDENT (P/R too small, real error larger than reported). Low NIS
=> filter is UNDERCONFIDENT (too conservative). Both are sanity signals
only here -- this script does not change any config value; it is a
report, exactly like the Doppler ablation (docs/decisions.md, tune split
only, does not touch configs/frozen_config.json).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ttcf import config  # noqa: E402
from run_pipeline import run_real_stream  # noqa: E402

STREAMS = ["lidar", "radar", "camera", "fused"]
DOF = 2
CHI2_MEAN = chi2.mean(DOF)  # 2.0
CHI2_MEDIAN = chi2.median(DOF)  # ~1.386
TAIL_FRACTION_NOMINAL = 0.01  # CHI2_GATE is by construction the 99th percentile


def _nis_values_for_stream(nusc, scene_tokens: list[str], stream: str) -> list[float]:
    scene_runs = run_real_stream(nusc, scene_tokens, stream)
    values = []
    for run in scene_runs.values():
        for entry in run["tracker"].association_log:
            if entry.outcome == "matched":
                values.append(entry.d2)
    return values


def summarize(stream: str, values: list[float]) -> dict:
    arr = np.asarray(values, dtype=float)
    gate = config.CHI2_GATE.value
    tail_frac = float(np.mean(arr > gate)) if arr.size else float("nan")
    return {
        "stream": stream,
        "n_matched_updates": int(arr.size),
        "mean_nis": float(np.mean(arr)) if arr.size else float("nan"),
        "median_nis": float(np.median(arr)) if arr.size else float("nan"),
        "theoretical_mean": CHI2_MEAN,
        "theoretical_median": CHI2_MEDIAN,
        "tail_fraction_over_gate": tail_frac,
        "nominal_tail_fraction": TAIL_FRACTION_NOMINAL,
        "chi2_gate": gate,
    }


def main():
    from nuscenes.nuscenes import NuScenes

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_tokens = [s["token"] for s in split["tune"]]  # TUNE SPLIT ONLY -- never eval, per the module docstring

    print(f"NIS sanity check -- {len(scene_tokens)} tune-split scenes, RADAR_DOPPLER_ABLATION={config.RADAR_DOPPLER_ABLATION.value}\n")

    summaries = []
    for stream in STREAMS:
        values = _nis_values_for_stream(nusc, scene_tokens, stream)
        s = summarize(stream, values)
        summaries.append(s)
        verdict = (
            "OVERCONFIDENT (NIS too high -- P/R underestimate real error)" if s["mean_nis"] > CHI2_MEAN * 1.5
            else "UNDERCONFIDENT (NIS too low -- P/R overestimate real error)" if s["mean_nis"] < CHI2_MEAN * 0.5
            else "consistent with a well-calibrated filter"
        )
        print(f"--- {stream} ---")
        print(f"  n_matched_updates     = {s['n_matched_updates']}")
        print(f"  mean_nis              = {s['mean_nis']:.3f}  (theory: {CHI2_MEAN:.3f})")
        print(f"  median_nis            = {s['median_nis']:.3f}  (theory: {CHI2_MEDIAN:.3f})")
        print(f"  tail_fraction_over_gate = {s['tail_fraction_over_gate']:.4f}  (nominal: {TAIL_FRACTION_NOMINAL:.4f}, gate={s['chi2_gate']:.3f})")
        print(f"  verdict: {verdict}\n")

    out_path = Path("outputs/tables/nis_check.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(summaries, indent=2), encoding="utf-8")
    print(f"Written to {out_path}")


if __name__ == "__main__":
    main()
