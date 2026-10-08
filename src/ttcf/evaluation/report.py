"""
src/ttcf/evaluation/report.py — STEP 10: the four-run report generator.

Produces outputs/tables/four_run_table.md/.csv/.json -- four rows
(lidar/radar/camera/fused), every metric with raw counts, both
observability variants. The 7 required footnotes (§3.6) are generated
from explicit inputs and the generator FAILS if any is missing, per the
step file's own instruction -- never silently omitted.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Optional


class ReportFootnoteError(ValueError):
    pass


def _fmt_rate(rate) -> str:
    if rate.value is None:
        return "n/a"
    marker = " (indicative only)" if rate.indicative_only else ""
    ci = f" [{rate.wilson_low:.2f}, {rate.wilson_high:.2f}]" if rate.wilson_low is not None else ""
    return f"{rate.value:.1%} ({rate.numerator}/{rate.denominator}){ci}{marker}"


def build_footnotes(
    radar_doppler_disabled: Optional[bool],
    sensor_r_status: Optional[dict],
    indicative_rows: Optional[list],
    class_policy: Optional[str],
    split_name: Optional[str],
    tuning_split_only: Optional[bool],
    distance_definition: Optional[str],
    n_gt_events: Optional[int],
    n_danger_events: Optional[int],
) -> dict:
    """Every argument is required -- raises ReportFootnoteError if any
    input is missing, per step10 §3.6: "the generator must fail if any
    is missing." indicative_rows and n_danger_events may legitimately be
    empty/0 (a real, reportable state), so they're checked for None,
    not falsiness; everything else is checked for a genuinely missing
    (None/empty) value."""
    missing = []
    if radar_doppler_disabled is None:
        missing.append("radar_doppler_disabled")
    if not sensor_r_status:
        missing.append("r_measured_vs_assumed (sensor_r_status)")
    if indicative_rows is None:
        missing.append("indicative_only_rows")
    if not class_policy:
        missing.append("class_policy")
    if not split_name:
        missing.append("split_used")
    if tuning_split_only is None:
        missing.append("tuning_split_only")
    if not distance_definition:
        missing.append("distance_definition")
    if n_gt_events is None:
        missing.append("gt_event_count")
    if n_danger_events is None:
        missing.append("danger_event_count")
    if missing:
        raise ReportFootnoteError(f"Cannot generate report: missing required footnote source(s): {missing}")

    danger_pct = f"{n_danger_events / n_gt_events:.1%}" if n_gt_events else "n/a"
    return {
        "1_radar_doppler": (
            "Radar-only ran with Doppler DISABLED (G17) -- this reflects this configuration, "
            "not radar's real capability (what-not-to-do.md §7)."
            if radar_doppler_disabled
            else "Radar Doppler was enabled for this run."
        ),
        "2_r_measured_vs_assumed": (
            "Per-sensor measurement noise (R) status: "
            + ", ".join(f"{sensor}={status}" for sensor, status in sorted(sensor_r_status.items()))
        ),
        "3_indicative_only": (
            f"{len(indicative_rows)} row(s) fall under MIN_COUNT_FOR_RATE and are labelled "
            f"'indicative only': {indicative_rows}"
            if indicative_rows
            else "No rows fall under MIN_COUNT_FOR_RATE."
        ),
        "4_class_policy": class_policy,
        "5_split": (
            f"Split used: {split_name}. "
            + (
                "Tuning was performed on the tuning split only."
                if tuning_split_only
                else "WARNING: tuning-split-only was NOT confirmed for this run."
            )
        ),
        "6_distance_definition": f"Distance definition (DEC-1): {distance_definition}.",
        "7_gt_events": (
            f"{n_gt_events} total GT rows, {n_danger_events} danger events ({danger_pct} of GT rows) -- "
            "nuScenes-mini has few danger events; treat small-denominator rates as indicative only."
        ),
    }


def generate_four_run_table(runs: dict, footnotes: dict, out_dir: Path = Path("outputs/tables")) -> dict:
    """runs: run_name ("lidar"/"radar"/"camera"/"fused") -> {variant_name
    ("all" or an obs_field): RunMetrics}. Fused numbers are never reported
    without the other three (step10 §4) -- raises if any is missing.
    Returns the dict written to JSON, so tests/callers can inspect it
    without re-parsing files."""
    required_runs = ["lidar", "radar", "camera", "fused"]
    missing_runs = [r for r in required_runs if r not in runs]
    if missing_runs:
        raise ReportFootnoteError(
            f"Cannot generate report: missing run(s) {missing_runs} -- "
            "fused numbers are never reported without the other three (step10 §4)."
        )

    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for run_name in required_runs:
        for variant_name, m in runs[run_name].items():
            rows.append(
                {
                    "run": run_name,
                    "observability_variant": variant_name,
                    "n_scored": m.n_scored,
                    "correct_action_rate": _fmt_rate(m.correct_action_rate),
                    "false_brake_rate": _fmt_rate(m.false_brake_rate),
                    "missed_brake_rate": _fmt_rate(m.missed_brake_rate),
                    "under_brake_count": m.under_brake_count,
                    "over_brake_count": m.over_brake_count,
                    "phantom_brake_count": f"{m.phantom_brake_count}/{m.phantom_brake_denominator}",
                    "false_brake_incl_phantom_rate": _fmt_rate(m.false_brake_incl_phantom_rate),
                }
            )

    payload = {"rows": rows, "footnotes": footnotes}
    (out_dir / "four_run_table.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")

    with open(out_dir / "four_run_table.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    md_lines = [
        "# Four-run comparison\n",
        "| " + " | ".join(rows[0].keys()) + " |",
        "|" + "---|" * len(rows[0]),
    ]
    for row in rows:
        md_lines.append("| " + " | ".join(str(v) for v in row.values()) + " |")
    md_lines.append("\n## Footnotes\n")
    for key, text in footnotes.items():
        md_lines.append(f"- **{key}**: {text}")
    (out_dir / "four_run_table.md").write_text("\n".join(md_lines), encoding="utf-8")

    return payload
