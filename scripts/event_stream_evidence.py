"""
scripts/event_stream_evidence.py — STEP 01 §6: real-data evidence for Siva.

Produces:
  - outputs/figs/step01/raster_1s.png: event arrivals over 1s per channel.
  - outputs/tables/step01_interarrival.csv: inter-arrival mean/std per channel.
  - printed: ms offsets between channels within one keyframe sample.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf import config  # noqa: E402
from ttcf.data.event_stream import scene_channel_events  # noqa: E402

OUT_FIGS = Path("outputs/figs/step01")
OUT_TABLES = Path("outputs/tables")


def section(title: str) -> None:
    print(f"\n{'=' * 70}\n{title}\n{'=' * 70}")


def raster_plot(nusc, scene_token: str, channels: list[str]) -> None:
    section("1. Event arrivals over 1 second, per channel (raster)")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    scene = nusc.get("scene", scene_token)
    first_sample = nusc.get("sample", scene["first_sample_token"])
    t0 = int(first_sample["timestamp"])
    window_end = t0 + 1_000_000  # 1 second window

    fig, ax = plt.subplots(figsize=(10, 3.5))
    for i, ch in enumerate(channels):
        events = [e for e in scene_channel_events(nusc, scene_token, ch) if t0 <= e.t_us <= window_end]
        t_ms = [(e.t_us - t0) / 1000.0 for e in events]
        ax.scatter(t_ms, [i] * len(t_ms), marker="|", s=200)
    ax.set_yticks(range(len(channels)))
    ax.set_yticklabels(channels)
    ax.set_xlabel("time since scene start (ms)")
    ax.set_xlim(0, 1000)
    ax.set_title(f"{scene['name']}: event arrivals over 1s, native rate per channel")
    fig.tight_layout()
    OUT_FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_FIGS / "raster_1s.png", dpi=130)
    plt.close(fig)
    print(f"saved outputs/figs/step01/raster_1s.png ({scene['name']}, first 1s)")


def interarrival_table(nusc, scene_token: str, channels: list[str]) -> pd.DataFrame:
    section("2. Inter-arrival mean/std per channel")
    rows = []
    scene = nusc.get("scene", scene_token)
    for ch in channels:
        ts = [e.t_us for e in scene_channel_events(nusc, scene_token, ch)]
        gaps_ms = np.diff(ts) / 1000.0
        rows.append(
            {
                "channel": ch,
                "n_events": len(ts),
                "mean_interarrival_ms": float(np.mean(gaps_ms)),
                "std_interarrival_ms": float(np.std(gaps_ms)),
                "implied_hz": 1000.0 / float(np.mean(gaps_ms)),
            }
        )
    df = pd.DataFrame(rows)
    print(f"scene: {scene['name']}")
    print(df.to_string(index=False))
    OUT_TABLES.mkdir(parents=True, exist_ok=True)
    df.to_csv(OUT_TABLES / "step01_interarrival.csv", index=False)
    return df


def keyframe_offsets(nusc, scene_token: str, channels: list[str]) -> None:
    section("3. Millisecond offsets between channels within one keyframe sample")
    scene = nusc.get("scene", scene_token)
    sample = nusc.get("sample", scene["first_sample_token"])
    ts = {ch: nusc.get("sample_data", sample["data"][ch])["timestamp"] for ch in channels}
    ref = ts["LIDAR_TOP"]
    for ch in channels:
        print(f"  {ch:20s} offset from LIDAR_TOP: {(ts[ch] - ref) / 1000.0:+.3f} ms")


def main():
    from nuscenes.nuscenes import NuScenes

    if not config._has_nuscenes_layout(config.DATAROOT):
        print(f"STOP: {config.DATAROOT} does not have the expected nuScenes layout.")
        sys.exit(1)

    nusc = NuScenes(version=config.NUSCENES_VERSION, dataroot=str(config.DATAROOT), verbose=False)
    channels = config.ACTIVE_CHANNELS.value
    scene_token = nusc.scene[0]["token"]

    raster_plot(nusc, scene_token, channels)
    interarrival_table(nusc, scene_token, channels)
    keyframe_offsets(nusc, scene_token, channels)


if __name__ == "__main__":
    main()
