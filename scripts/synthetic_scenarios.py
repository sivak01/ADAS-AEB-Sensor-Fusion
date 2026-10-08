"""
scripts/synthetic_scenarios.py — reusable synthetic scenario generators
for step06 (tracker) and later steps (step08, step09, ...).

Detections are already in the GLOBAL frame (G1) -- the tracker never
touches ego state directly, so "ego moves while an object stays
stationary" is expressed simply as a zero-velocity object; ego motion
doesn't enter the tracker's own math at all, by design.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from ttcf.types import Detection


def constant_velocity_scans(
    objects: list[tuple[float, float, float, float]],
    t0_us: int,
    dt_us: int,
    n_scans: int,
    channels: list[str] = ("SENSOR_A", "SENSOR_B", "SENSOR_C"),
    sigmas: list[float] = (0.1, 0.5, 1.5),
    rng: Optional[np.random.Generator] = None,
    cls: str = "vehicle.car",
) -> list[tuple[int, list[Detection]]]:
    """objects: list of (x0, y0, vx, vy) true trajectories, global frame.
    Returns n_scans scans, one per time step, each a (t_us, list[Detection])
    pair -- one Detection per object, all in that scan observed by the
    SAME alternating channel (round-robin through `channels`/`sigmas`),
    each independently noised by that channel's own sigma. Channels
    alternate so a single object gets a mix of tight- and loose-R
    updates over its life, exactly the scenario step06 §5 asks for."""
    rng = rng if rng is not None else np.random.default_rng(0)
    scans = []
    for i in range(n_scans):
        t_us = t0_us + i * dt_us
        dt_s = (t_us - t0_us) / 1e6
        ch_idx = i % len(channels)
        channel, sigma = channels[ch_idx], sigmas[ch_idx]
        R = np.diag([sigma**2, sigma**2])
        dets = []
        for (x0, y0, vx, vy) in objects:
            true_xy = np.array([x0 + vx * dt_s, y0 + vy * dt_s])
            noisy_xy = true_xy + rng.normal(scale=sigma, size=2)
            dets.append(
                Detection(
                    t_us=t_us,
                    channel=channel,
                    modality="synthetic",
                    xy_global=noisy_xy,
                    R_global=R,
                    ref_point_kind="nearest_surface_to_ego",
                    cls=cls,
                    n_points=None,
                    aux={},
                )
            )
        scans.append((t_us, dets))
    return scans


def single_object_scans(
    x0: float,
    y0: float,
    vx: float,
    vy: float,
    t0_us: int = 0,
    dt_us: int = 100_000,
    n_scans: int = 20,
    channels: list[str] = ("SENSOR_A", "SENSOR_B", "SENSOR_C"),
    sigmas: list[float] = (0.1, 0.5, 1.5),
    rng: Optional[np.random.Generator] = None,
) -> list[tuple[int, list[Detection]]]:
    """Convenience wrapper: one constant-velocity object, alternating
    sensors."""
    return constant_velocity_scans([(x0, y0, vx, vy)], t0_us, dt_us, n_scans, channels, sigmas, rng)


def true_position(x0: float, y0: float, vx: float, vy: float, t_us: int, t0_us: int) -> np.ndarray:
    dt_s = (t_us - t0_us) / 1e6
    return np.array([x0 + vx * dt_s, y0 + vy * dt_s])
