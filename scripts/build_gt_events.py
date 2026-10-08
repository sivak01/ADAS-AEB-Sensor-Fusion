"""
scripts/build_gt_events.py — STEP D0: build the ground-truth event table
and the reports Siva needs to see (stepD0_gt_event_table.md §4).

Does NOT write configs/split.json -- that only happens after Siva
approves the split proposal printed here (step §9's STOP gate: "Only
then write configs/split.json").
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.geometry.corridor import footprint_intersects_corridor  # noqa: E402
from ttcf.geometry.transforms import FrameContext, global_to_ego  # noqa: E402
from ttcf.gt.gt_events import _footprint_corners_global, build_all_gt_events  # noqa: E402

OUT_FIGS = Path("outputs/figs/stepD0")
OUT_TABLES = Path("outputs/tables")
OUT_CSV = Path("outputs/gt_events.csv")

_IDENTITY_CS = {"translation": [0.0, 0.0, 0.0], "rotation": [1.0, 0.0, 0.0, 0.0]}
_TIER_COLOR = {"NONE": "tab:gray", "GRADUAL": "tab:orange", "AEB": "tab:red"}


def section(title: str) -> None:
    print(f"\n{'=' * 70}\n{title}\n{'=' * 70}")


def to_dataframe(rows) -> pd.DataFrame:
    return pd.DataFrame([r.__dict__ for r in rows])


def write_data_dictionary_and_csv(gt_df: pd.DataFrame) -> None:
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    dictionary = {
        "scene_token/scene_name": "which scene this row's keyframe belongs to",
        "sample_token": "the keyframe (2Hz) sample this row is at",
        "t_us": "keyframe timestamp, integer microseconds",
        "instance_token": "the GT object's identity",
        "category": "nuScenes leaf category (DEC-2 scope: vehicle.*, human.pedestrian.*)",
        "attributes": "semicolon-joined attribute names (e.g. vehicle.parked)",
        "visibility_level": "1-4 (v0-40 .. v80-100)",
        "num_lidar_pts/num_radar_pts": "real sensor points inside the GT box at this instant",
        "distance_m": "DEC-1 nearest-surface-to-ego distance (m)",
        "closing_speed_mps": "relative closing speed (m/s), G8 convention",
        "gt_ttc_s": "ground-truth TTC (s), may be inf",
        "gt_action": "NONE/GRADUAL/AEB from gt_ttc_s vs TTC_GRADUAL_S/TTC_AEB_S",
        "vel_valid": "False if nusc.box_velocity() returned NaN (excluded from gt_action, still counted)",
        "obs_lidar/obs_radar/obs_camera/obs_any": "was this object plausibly observable by that sensor",
    }
    with open(OUT_CSV, "w", encoding="utf-8") as f:
        f.write("# Data dictionary (see reports/stepD0_report.md for full detail)\n")
        for k, v in dictionary.items():
            f.write(f"# {k}: {v}\n")
    gt_df.to_csv(OUT_CSV, mode="a", index=False)
    print(f"Wrote {OUT_CSV} ({len(gt_df)} rows)")


def per_scene_table(gt_df: pd.DataFrame, kf_df: pd.DataFrame) -> pd.DataFrame:
    section("1. Per-scene table")
    rows = []
    for scene_name in kf_df["scene_name"].unique():
        kf_group = kf_df[kf_df["scene_name"] == scene_name]
        scene_gt = gt_df[gt_df["scene_name"] == scene_name]
        tier_counts = scene_gt["gt_action"].value_counts().to_dict() if len(scene_gt) else {}
        rows.append(
            {
                "scene": scene_name,
                "n_keyframes": len(kf_group),
                "n_keyframes_in_path": int((kf_group["n_in_path"] > 0).sum()),
                "n_gt_rows": len(scene_gt),
                "NONE": tier_counts.get("NONE", 0),
                "GRADUAL": tier_counts.get("GRADUAL", 0),
                "AEB": tier_counts.get("AEB", 0),
                "vel_invalid": int((~scene_gt["vel_valid"]).sum()) if len(scene_gt) else 0,
            }
        )
    df = pd.DataFrame(rows)
    print(df.to_string(index=False))
    OUT_TABLES.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_TABLES / "stepD0_per_scene.csv", index=False)
    return df


def overall_totals(gt_df: pd.DataFrame, kf_df: pd.DataFrame) -> dict:
    section("2. Overall totals")
    total_keyframes = len(kf_df)
    total_in_path_kf = int((kf_df["n_in_path"] > 0).sum())
    total_gt_rows = len(gt_df)
    tier_counts = gt_df["gt_action"].value_counts().to_dict() if total_gt_rows else {}
    danger = tier_counts.get("GRADUAL", 0) + tier_counts.get("AEB", 0)

    print(f"Total keyframes (all scenes): {total_keyframes}")
    print(
        f"Keyframes with >=1 in-path GT object: {total_in_path_kf} "
        f"({100 * total_in_path_kf / total_keyframes:.1f}%); "
        f"empty-corridor keyframes: {total_keyframes - total_in_path_kf}"
    )
    print(f"Total GT rows (keyframe, in-path object pairs): {total_gt_rows}")
    print(f"Tier counts: {tier_counts}")
    if total_gt_rows:
        print(f"Danger events (tier != NONE): {danger} / {total_gt_rows} ({100 * danger / total_gt_rows:.2f}%)")
    print(
        "NOTE (stepD0 §4.2): expect this danger-event count to be small in nuScenes-mini "
        "— stated plainly, not treated as statistically solid on its own (what-not-to-do.md §7)."
    )
    return {
        "total_keyframes": total_keyframes,
        "total_in_path_keyframes": total_in_path_kf,
        "total_gt_rows": total_gt_rows,
        "tier_counts": tier_counts,
        "danger": danger,
    }


def histograms(gt_df: pd.DataFrame) -> None:
    section("3. Histograms: GT TTC (finite) and closing speed")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    finite_ttc = gt_df.loc[np.isfinite(gt_df["gt_ttc_s"]), "gt_ttc_s"]
    closing = gt_df["closing_speed_mps"]

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].hist(finite_ttc, bins=40, color="tab:blue")
    axes[0].set_title(f"GT TTC, finite values (n={len(finite_ttc)}/{len(gt_df)})")
    axes[0].set_xlabel("TTC (s)")
    axes[1].hist(closing, bins=40, color="tab:green")
    axes[1].set_title("Closing speed, all GT rows")
    axes[1].set_xlabel("closing speed (m/s)")
    fig.tight_layout()
    OUT_FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_FIGS / "histograms.png", dpi=130)
    plt.close(fig)

    if len(finite_ttc):
        print(f"finite TTC: min={finite_ttc.min():.2f} max={finite_ttc.max():.2f} median={finite_ttc.median():.2f}")
    print(f"closing speed: min={closing.min():.2f} max={closing.max():.2f} median={closing.median():.2f}")


def bev_figures(nusc, gt_df: pd.DataFrame, kf_df: pd.DataFrame, per_scene_df: pd.DataFrame) -> None:
    section("4. Sample BEV figures (corridor + GT boxes by tier)")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    half_width = config.CORRIDOR_HALF_WIDTH_M.value
    max_range = config.CORRIDOR_MAX_RANGE_M.value

    danger_by_scene = (per_scene_df["GRADUAL"] + per_scene_df["AEB"]).values
    order = np.argsort(-danger_by_scene)
    top_scene = per_scene_df.iloc[order[0]]["scene"]
    other_scenes = [per_scene_df.iloc[i]["scene"] for i in order[1:3]]
    chosen_scenes = [top_scene] + other_scenes

    tier_rank = {"NONE": 0, "GRADUAL": 1, "AEB": 2}

    def _pick_sample(scene_name: str) -> str:
        """Prefer the keyframe showing the most severe tier present (so a
        danger scene's figure actually shows the danger event, not just
        whichever keyframe happens to have the most objects); fall back
        to the busiest in-path keyframe if the scene has no danger tier."""
        scene_gt = gt_df[gt_df["scene_name"] == scene_name]
        if len(scene_gt):
            severities = scene_gt["gt_action"].map(tier_rank)
            best_idx = severities.idxmax()
            if severities.loc[best_idx] > 0:
                return scene_gt.loc[best_idx, "sample_token"]
        scene_kf = kf_df[kf_df["scene_name"] == scene_name]
        if scene_kf["n_in_path"].max() == 0:
            return scene_kf.iloc[0]["sample_token"]
        return scene_kf.loc[scene_kf["n_in_path"].idxmax(), "sample_token"]

    OUT_FIGS.mkdir(parents=True, exist_ok=True)
    for scene_name in chosen_scenes:
        chosen_sample = _pick_sample(scene_name)
        sample = nusc.get("sample", chosen_sample)
        lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
        ego_pose = nusc.get("ego_pose", lidar_sd["ego_pose_token"])
        ctx = FrameContext(calibrated_sensor=_IDENTITY_CS, ego_pose=ego_pose)

        rows_here = gt_df[gt_df["sample_token"] == chosen_sample]

        fig, ax = plt.subplots(figsize=(6, 8))
        plotted_x = [0.0]
        plotted_y = [0.0]

        for _, row in rows_here.iterrows():
            ann = next(
                a
                for a in [nusc.get("sample_annotation", tok) for tok in sample["anns"]]
                if a["instance_token"] == row["instance_token"]
            )
            corners_global = _footprint_corners_global(ann)
            corners_ego = global_to_ego(corners_global, ctx)[:, :2]
            poly = np.vstack([corners_ego, corners_ego[0]])
            ax.plot(poly[:, 0], poly[:, 1], color=_TIER_COLOR[row["gt_action"]])
            ax.fill(poly[:, 0], poly[:, 1], color=_TIER_COLOR[row["gt_action"]], alpha=0.4)
            plotted_x.extend(corners_ego[:, 0].tolist())
            plotted_y.extend(corners_ego[:, 1].tolist())

        # Dynamic view window (padded around ego + whatever's plotted) --
        # a fixed window clipped a farther-away object off-screen for one
        # scene in an earlier version of this figure, silently showing an
        # empty plot even though the underlying data was correct.
        x_max = max(30.0, max(plotted_x) + 5.0)
        y_span = max(10.0, max(abs(min(plotted_y)), abs(max(plotted_y))) + 3.0)

        corridor_x = [0, max_range, max_range, 0]
        corridor_y = [-half_width, -half_width, half_width, half_width]
        ax.fill(corridor_x, corridor_y, color="lightblue", alpha=0.4, label="corridor", zorder=0)
        ax.plot(0, 0, marker="^", color="black", markersize=10, label="ego", zorder=5)

        ax.set_xlim(-5, x_max)
        ax.set_ylim(-y_span, y_span)
        ax.set_xlabel("x, ego forward (m)")
        ax.set_ylabel("y, ego left (m)")
        ax.set_title(f"{scene_name} @ {chosen_sample[:8]} (most severe in-path tier)")
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(OUT_FIGS / f"bev_{scene_name}.png", dpi=130)
        plt.close(fig)
        print(f"  saved outputs/figs/stepD0/bev_{scene_name}.png")


def split_proposal(per_scene_df: pd.DataFrame) -> pd.DataFrame:
    section("5. Split proposal (DEC-8) — PROPOSAL ONLY, not written to configs/split.json")
    df = per_scene_df.copy()
    df["danger"] = df["GRADUAL"] + df["AEB"]

    # A scene can only go on ONE side -- if a single scene holds all (or
    # nearly all) of one tier's events, no split can give both sides real
    # representation of that tier. Checked directly rather than assumed:
    aeb_scenes = df.loc[df["AEB"] > 0, "scene"].tolist()
    if len(aeb_scenes) == 1:
        print(
            f"NOTE: all {int(df['AEB'].sum())} AEB events are in a single scene ({aeb_scenes[0]}). "
            "A scene-level split cannot give both tune and eval real AEB representation -- "
            "assigning it to EVAL (the side the final Definition-of-Done table is built from, "
            "so the reported AEB tier isn't empty), and stating this plainly rather than letting "
            "a naive balanced-count split hide it."
        )
        forced_eval = aeb_scenes
    else:
        forced_eval = []

    remaining = df[~df["scene"].isin(forced_eval)].sort_values("danger", ascending=False).reset_index(drop=True)
    tune, eval_ = [], [df[df["scene"] == s].iloc[0] for s in forced_eval]
    for i, row in remaining.iterrows():
        (tune if i % 2 == 0 else eval_).append(row)
    tune_df = pd.DataFrame(tune)
    eval_df = pd.DataFrame(eval_)

    print("Proposed TUNE scenes:")
    print(tune_df[["scene", "n_gt_rows", "GRADUAL", "AEB", "danger"]].to_string(index=False))
    print(f"  tune totals: gt_rows={tune_df['n_gt_rows'].sum()}, danger={tune_df['danger'].sum()}")
    print("\nProposed EVAL scenes:")
    print(eval_df[["scene", "n_gt_rows", "GRADUAL", "AEB", "danger"]].to_string(index=False))
    print(f"  eval totals: gt_rows={eval_df['n_gt_rows'].sum()}, danger={eval_df['danger'].sum()}")

    return pd.concat(
        [tune_df.assign(split="tune"), eval_df.assign(split="eval")], ignore_index=True
    )


def thin_denominator_report(split_df: pd.DataFrame) -> None:
    section("6. Thin-denominator rule (MIN_COUNT_FOR_RATE)")
    min_count = config.MIN_COUNT_FOR_RATE.value
    print(f"MIN_COUNT_FOR_RATE = {min_count}: any rate computed from fewer events is labelled 'indicative only'.")

    eval_rows = split_df[split_df["split"] == "eval"]
    eval_danger = int(eval_rows["danger"].sum())
    eval_gt_rows = int(eval_rows["n_gt_rows"].sum())
    eval_aeb = int(eval_rows["AEB"].sum())
    eval_gradual = int(eval_rows["GRADUAL"].sum())

    cells = {
        "eval-side total GT rows": eval_gt_rows,
        "eval-side danger events (GRADUAL+AEB)": eval_danger,
        "eval-side AEB events": eval_aeb,
        "eval-side GRADUAL events": eval_gradual,
    }
    thin_cells = []
    for name, count in cells.items():
        flag = "INDICATIVE ONLY (below MIN_COUNT_FOR_RATE)" if count < min_count else "OK"
        print(f"  {name}: {count}  -> {flag}")
        if count < min_count:
            thin_cells.append(name)

    print(f"\n{len(thin_cells)}/{len(cells)} proposed eval-side cells fall below MIN_COUNT_FOR_RATE={min_count}.")
    print(
        "Per stepD0 §4.6: propose Wilson score intervals (step10) and/or a looser 'brake-worthy' "
        "threshold as an ADDITIONAL, clearly separate row alongside these counts -- never as a replacement."
    )


def main():
    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)

    section("0. GT category scope (DEC-2) and instant/pose source (stepD0 §3.1)")
    print(f"GT_CATEGORIES ({len(config.GT_CATEGORIES.value)}): {config.GT_CATEGORIES.value}")

    # Report the ms difference between candidate "GT instant" sources, per §3.1.
    sample = nusc.sample[0]
    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    dt_ms = (lidar_sd["timestamp"] - sample["timestamp"]) / 1000.0
    print(
        f"Chosen: LIDAR_TOP keyframe sample_data's own ego_pose (position), "
        f"EgoStateEstimator (velocity). LIDAR_TOP sample_data timestamp vs. "
        f"sample.timestamp differs by {dt_ms:.3f} ms (example sample)."
    )

    section("Building GT event table over all 10 scenes...")
    gt_rows, keyframe_rows = build_all_gt_events(nusc)
    gt_df = to_dataframe(gt_rows)
    kf_df = to_dataframe(keyframe_rows)
    print(f"{len(gt_df)} GT rows, {len(kf_df)} keyframe rows.")

    write_data_dictionary_and_csv(gt_df)
    per_scene_df = per_scene_table(gt_df, kf_df)
    overall_totals(gt_df, kf_df)
    histograms(gt_df)
    bev_figures(nusc, gt_df, kf_df, per_scene_df)
    split_df = split_proposal(per_scene_df)
    thin_denominator_report(split_df)

    section("Done")
    print("configs/split.json NOT written -- waiting for Siva's approval of the split proposal above.")


if __name__ == "__main__":
    main()
