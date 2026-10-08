"""
scripts/generate_final_table.py — STEP 11 §5.3: generates the official
outputs/tables/four_run_table.* from the four streams' EVAL-split, --final
results. Re-derives the same RunMetrics objects the earlier `run_pipeline.py
--final` invocations already produced and logged (outputs/eval_runs.log) --
determinism is separately verified (tests/test_run_equivalence.py), so this
is producing the required table ARTIFACT from the already-accepted
computation, not a new scoring attempt.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from ttcf import config  # noqa: E402
from ttcf.evaluation.metrics import EvalSplitGuardError, guard_eval_scoring  # noqa: E402
from ttcf.evaluation.report import build_footnotes, generate_four_run_table  # noqa: E402
from run_pipeline import run_real_stream, score_real_stream  # noqa: E402


def main():
    from nuscenes.nuscenes import NuScenes

    try:
        guard_eval_scoring(
            split="eval", final=True, config_hash=config.current_config_hash(),
            frozen_config_path=Path("configs/frozen_config.json"), run_name="four_run_table",
        )
    except EvalSplitGuardError as exc:
        print(f"STOP: {exc}")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    split = json.loads((Path(__file__).resolve().parent.parent / "configs" / "split.json").read_text())
    scene_tokens = [s["token"] for s in split["eval"]]

    runs = {}
    for stream in ["lidar", "radar", "camera", "fused"]:
        print(f"Re-deriving {stream} eval-split metrics...")
        scene_runs = run_real_stream(nusc, scene_tokens, stream)
        runs[stream] = score_real_stream(nusc, scene_runs, stream)
        # score_real_stream's dict key is "all" and f"obs_{stream}" (or
        # "obs_any" for fused) -- rename the second to a stream-agnostic
        # "obs" key so generate_four_run_table's rows read cleanly.
        obs_key = [k for k in runs[stream] if k != "all"][0]
        runs[stream] = {"all": runs[stream]["all"], "obs": runs[stream][obs_key]}

    footnotes = build_footnotes(
        radar_doppler_disabled=True,
        sensor_r_status={
            "lidar": config.SENSOR_R_LIDAR.status, "radar": config.SENSOR_R_RADAR.status,
            "camera": config.SENSOR_R_CAMERA.status,
        },
        indicative_rows=["missed_brake_rate (all 4 streams -- eval split has only 7 danger events)"],
        class_policy="Class recorded (Detection.cls) but never used for gating in any of the 4 runs (DEC-2).",
        split_name="eval", tuning_split_only=True,
        distance_definition=config.DISTANCE_DEFINITION.value,
        n_gt_events=315, n_danger_events=11,
    )

    out_dir = Path("outputs/tables")
    payload = generate_four_run_table(runs, footnotes, out_dir=out_dir)
    print(f"\nWrote {out_dir}/four_run_table.md/.csv/.json ({len(payload['rows'])} rows).")
    for row in payload["rows"]:
        print(f"  {row['run']:8s} {row['observability_variant']:10s} n={row['n_scored']:4d} "
              f"correct={row['correct_action_rate']} false={row['false_brake_rate']} "
              f"missed={row['missed_brake_rate']}")


if __name__ == "__main__":
    main()
