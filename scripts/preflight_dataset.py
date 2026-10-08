"""
scripts/preflight_dataset.py — step00_config_contracts.md §3.

Run this BEFORE writing any pipeline code. It only reads archive/ (never
writes into it — README_MASTER.md §2) and confirms the dataset on disk is
what this project assumes it is, especially that native-rate sweeps/
actually exist (G3) — the whole point of an event-driven design.

Usage:
    .venv\\Scripts\\python.exe scripts\\preflight_dataset.py
"""
from __future__ import annotations

import inspect
import platform
import random
import sys
from collections import defaultdict
from pathlib import Path
from statistics import median

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402


def section(title: str) -> None:
    print(f"\n{'=' * 70}\n{title}\n{'=' * 70}")


def step1_discover_dataroot() -> Path:
    section("1. DATAROOT discovery")
    print(f"BASE_DIR       : {config.BASE_DIR}")
    print(f"TTCF_DATAROOT  : {__import__('os').environ.get('TTCF_DATAROOT', '(not set)')}")
    print(f"Resolved DATAROOT : {config.DATAROOT}")
    print(f"Resolved VERSION  : {config.NUSCENES_VERSION}")
    if not config._has_nuscenes_layout(config.DATAROOT):
        print(
            f"\nSTOP: {config.DATAROOT} does not have the expected nuScenes layout "
            "(samples/, sweeps/, a v1.0-* folder with sample_data.json). "
            "Tell Siva before proceeding — do not guess further."
        )
        sys.exit(1)
    print(f"\nTop-level entries under DATAROOT:")
    for entry in sorted(config.DATAROOT.iterdir()):
        print(f"  {entry.name}")
    return config.DATAROOT


def step2_load_and_report(dataroot: Path):
    section("2. Load NuScenes and report table sizes")
    from nuscenes.nuscenes import NuScenes

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(dataroot), verbose=True)
    print(f"\nscenes       : {len(nusc.scene)} (expect 10 for v1.0-mini)")
    print(f"samples      : {len(nusc.sample)}")
    print(f"sample_data  : {len(nusc.sample_data)}")
    if len(nusc.scene) != 10:
        print("WARNING: scene count != 10 for v1.0-mini — check DATAROOT/VERSION above.")
    return nusc


def step3_sweeps_check(nusc, dataroot: Path):
    section("3. Sweeps check (critical, G3)")
    by_channel_keyframe = defaultdict(int)
    by_channel_sweep = defaultdict(int)
    for sd in nusc.sample_data:
        channel = nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])
        channel = nusc.get("sensor", channel["sensor_token"])["channel"]
        if sd["is_key_frame"]:
            by_channel_keyframe[channel] += 1
        else:
            by_channel_sweep[channel] += 1

    print(f"{'channel':25s} {'keyframes':>10s} {'sweeps':>10s}")
    all_channels = sorted(set(by_channel_keyframe) | set(by_channel_sweep))
    for ch in all_channels:
        print(f"{ch:25s} {by_channel_keyframe[ch]:>10d} {by_channel_sweep[ch]:>10d}")

    missing = [ch for ch in all_channels if by_channel_sweep[ch] == 0]
    if missing:
        print(f"\nSTOP: these channels have ZERO non-keyframe sample_data records: {missing}")
        print("Native-rate processing (G3) is the core of this project. Tell Siva before proceeding.")
        sys.exit(1)

    print("\nRandom-sample file-existence check (20 sweep records per channel):")
    rng = random.Random(0)
    sweep_records = defaultdict(list)
    for sd in nusc.sample_data:
        if sd["is_key_frame"]:
            continue
        channel = nusc.get("sensor", nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])["sensor_token"])["channel"]
        sweep_records[channel].append(sd)

    any_missing_file = False
    for ch in all_channels:
        records = sweep_records.get(ch, [])
        sample_n = min(20, len(records))
        sample = rng.sample(records, sample_n) if sample_n else []
        missing_files = [r["filename"] for r in sample if not (dataroot / r["filename"]).exists()]
        status = "OK" if not missing_files else f"MISSING {len(missing_files)}/{sample_n}"
        print(f"  {ch:25s} checked {sample_n:2d} -> {status}")
        if missing_files:
            any_missing_file = True

    if any_missing_file:
        print("\nSTOP: some sweep files referenced in sample_data.json are absent on disk.")
        print("Tell Siva before proceeding.")
        sys.exit(1)


def step4_devkit_introspection(nusc):
    section("4. Devkit version and API signatures")
    import importlib.metadata

    try:
        devkit_version = importlib.metadata.version("nuscenes-devkit")
    except importlib.metadata.PackageNotFoundError:
        devkit_version = "(package metadata not found)"
    print(f"nuscenes-devkit version: {devkit_version}")

    from nuscenes.nuscenes import NuScenes
    from nuscenes.utils.geometry_utils import transform_matrix
    from nuscenes.utils.data_classes import LidarPointCloud, RadarPointCloud

    targets = {
        "NuScenes.box_velocity": NuScenes.box_velocity,
        "NuScenes.get_sample_data_path": NuScenes.get_sample_data_path,
        "geometry_utils.transform_matrix": transform_matrix,
        "LidarPointCloud.from_file": LidarPointCloud.from_file,
        "RadarPointCloud.from_file": RadarPointCloud.from_file,
    }
    for name, obj in targets.items():
        try:
            print(f"  {name}{inspect.signature(obj)}")
        except (TypeError, ValueError) as e:
            print(f"  {name}: could not introspect signature ({e})")

    print("\nlist_categories():")
    nusc.list_categories()
    print("\nlist_attributes():")
    nusc.list_attributes()


def step5_native_rates(nusc):
    section("5. Native rate per channel (median timestamp gap)")
    by_channel_ts = defaultdict(list)
    for sd in nusc.sample_data:
        channel = nusc.get("sensor", nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])["sensor_token"])["channel"]
        by_channel_ts[channel].append(sd["timestamp"])

    print(f"{'channel':25s} {'median_gap_ms':>15s} {'approx_hz':>10s}")
    for ch in sorted(by_channel_ts):
        ts = sorted(by_channel_ts[ch])
        gaps_us = [t2 - t1 for t1, t2 in zip(ts, ts[1:]) if t2 > t1]
        if not gaps_us:
            print(f"{ch:25s} {'n/a':>15s} {'n/a':>10s}")
            continue
        med_gap_ms = median(gaps_us) / 1000.0
        hz = 1000.0 / med_gap_ms if med_gap_ms > 0 else float("nan")
        print(f"{ch:25s} {med_gap_ms:15.2f} {hz:10.2f}")


def step6_environment():
    section("6. Environment")
    print(f"Python version: {platform.python_version()} ({sys.executable})")
    try:
        import torch  # noqa: F401

        print("torch: present (NOT expected/installed by this project — investigate)")
    except ImportError:
        print("torch: not installed (expected — README_MASTER.md says ask before installing)")

    try:
        import subprocess

        out = subprocess.run(
            ["nvidia-smi"], capture_output=True, text=True, timeout=5
        )
        if out.returncode == 0:
            print("GPU: nvidia-smi present, a GPU may be available (not required by this project)")
        else:
            print("GPU: nvidia-smi not usable — assume no GPU visible")
    except Exception:
        print("GPU: nvidia-smi not found — assume no GPU visible")


def main():
    dataroot = step1_discover_dataroot()
    nusc = step2_load_and_report(dataroot)
    step3_sweeps_check(nusc, dataroot)
    step4_devkit_introspection(nusc)
    step5_native_rates(nusc)
    step6_environment()
    section("Preflight complete")
    print("No STOP conditions triggered. See reports/step00_report.md for the summarised findings.")


if __name__ == "__main__":
    main()
