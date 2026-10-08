"""
scripts/doppler_ablation.py — post-step11, G17 radar-Doppler-velocity
ablation, TUNE SPLIT ONLY (docs/decisions.md, 2026-10-08 entry).

Runs radar and fused streams on the tune split TWICE each -- once with
config.RADAR_DOPPLER_ABLATION=False (the official default, every report
through step11) and once with it flipped True (a combined position+
velocity KF update via H4, src/ttcf/filtering/kalman.py /
src/ttcf/tracking/tracker.py._kf_update_args) -- and reports the same
event-level metrics step11 used (correct/false-brake/missed-brake rate),
side by side. lidar/camera are not run here: RADAR_DOPPLER_ABLATION only
changes radar's own detections (adapters/radar.py), so a lidar-only or
camera-only run is bit-identical regardless of the flag and would waste
an expensive pipeline run proving nothing new.

Flips the flag via dataclasses.replace() on the frozen Param, writing
back onto the config MODULE's own attribute -- every call site
(adapters/radar.py, this script's own run_real_stream call) reads
`cfg.RADAR_DOPPLER_ABLATION.value` off that same module object at call
time, never a cached copy, so this takes effect immediately and only for
this process. Does NOT touch configs/frozen_config.json; does NOT run
--split eval (guard_eval_scoring is never invoked here at all).
"""
from __future__ import annotations

import json
import sys
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ttcf import config  # noqa: E402
from run_pipeline import run_real_stream, score_real_stream  # noqa: E402

STREAMS = ["radar", "fused"]


def _set_doppler_ablation(flag: bool) -> None:
    config.RADAR_DOPPLER_ABLATION = replace(config.RADAR_DOPPLER_ABLATION, value=flag)


def _run_one(nusc, scene_tokens, stream: str) -> dict:
    scene_runs = run_real_stream(nusc, scene_tokens, stream)
    return score_real_stream(nusc, scene_runs, stream)


def _rate(r) -> dict:
    """RunMetrics' rate fields are RateResult (value/numerator/denominator/
    wilson bounds/indicative_only), not a bare float -- unwrap for JSON."""
    return {
        "value": r.value, "numerator": r.numerator, "denominator": r.denominator,
        "wilson_low": r.wilson_low, "wilson_high": r.wilson_high,
        "indicative_only": r.indicative_only,
    }


def main():
    from nuscenes.nuscenes import NuScenes

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_tokens = [s["token"] for s in split["tune"]]  # TUNE SPLIT ONLY

    results = {}
    for ablation_flag in (False, True):
        _set_doppler_ablation(ablation_flag)
        assert config.RADAR_DOPPLER_ABLATION.value is ablation_flag  # confirm the flip actually took
        label = "doppler_ON" if ablation_flag else "doppler_OFF"
        results[label] = {}
        for stream in STREAMS:
            metrics = _run_one(nusc, scene_tokens, stream)
            results[label][stream] = {
                variant: {
                    "n_scored": m.n_scored,
                    "correct_action_rate": _rate(m.correct_action_rate),
                    "false_brake_rate": _rate(m.false_brake_rate),
                    "missed_brake_rate": _rate(m.missed_brake_rate),
                    "under_brake": m.under_brake_count,
                    "over_brake": m.over_brake_count,
                }
                for variant, m in metrics.items()
            }
    _set_doppler_ablation(False)  # restore the default before this process exits

    print(json.dumps(results, indent=2))

    print("\n=== Side-by-side (variant='all') ===")
    for stream in STREAMS:
        off = results["doppler_OFF"][stream]["all"]
        on = results["doppler_ON"][stream]["all"]
        print(f"\n--- {stream} (n_scored OFF={off['n_scored']} ON={on['n_scored']}) ---")
        for key in ("correct_action_rate", "false_brake_rate", "missed_brake_rate"):
            ov, nv = off[key]["value"], on[key]["value"]
            ov_s = f"{ov:.3f}" if ov is not None else "n/a"
            nv_s = f"{nv:.3f}" if nv is not None else "n/a"
            print(f"  {key:20s}: OFF={ov_s}  ON={nv_s}")

    out_path = Path("outputs/tables/doppler_ablation.json")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(f"\nWritten to {out_path}")


if __name__ == "__main__":
    main()
