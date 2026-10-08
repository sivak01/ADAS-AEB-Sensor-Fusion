"""
src/ttcf/filtering/kalman.py — the ONE constant-velocity Kalman filter (G5, G6).

State x = [x, y, vx, vy], global frame. Measurement z = [x, y] (H below).
Used twice: for tracked objects (step06) and for the ego vehicle's own
velocity (step03's EgoStateEstimator) -- same primitive, same discipline,
never a finite-difference velocity anywhere (what-not-to-do.md #3).

Ported from the predecessor project's v2/kalman_track.py: the F/Q matrix
formulas (standard constant-velocity / white-noise-acceleration model) and
the R-per-update()-call / sigma_a-fixed-at-construction split, both already
correct there. NOT ported: v2 used the SIMPLE covariance update; this class
uses Joseph form + re-symmetrisation per step03's explicit spec (more
numerically robust, never stress-tested enough to need it in v2). v2's
gating lived in a separate gating.py module operating on externally-passed
(x_pred, P_pred); step03 puts innovation_cov()/mahalanobis_sq() as methods
on this class instead -- same S = P_pred + R formula, validated in v2 by a
real measured finding: omitting R from the gate inflated LiDAR/radar/
camera_mono single-sensor track counts by +99%/+58%/+1%, an ordering that
tracked exactly with each sensor's own R (v2/gating.py module docstring).
"""
from __future__ import annotations

import copy

import numpy as np

_H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
_I4 = np.eye(4)
# Full-state (position + velocity) observation matrix -- exported for
# callers building a combined measurement update (e.g. the radar-Doppler
# ablation, G17): z=[x,y,vx,vy], H4=I4 measures the whole state directly.
H4 = _I4


class ConstantVelocityKF:
    """Sensor-agnostic: zero references to sensor names, modalities, or
    channels anywhere in this class. R arrives per update() call; sigma_a
    is fixed once at construction (a property of the tracked thing's own
    motion, not of whichever sensor is currently reporting on it)."""

    def __init__(self, sigma_a: float):
        self.sigma_a = float(sigma_a)
        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        # Internal clock, integer MICROSECONDS -- not accumulated float
        # seconds. predict() only receives dt (seconds), but advancing
        # this clock via repeated `+= dt` in float seconds drifts by a
        # few hundred nanoseconds after enough calls (found in step03's
        # real-data tests: state_at() called with a t_us exactly equal to
        # the last processed pose produced a dt of -1.2e-6 instead of
        # exactly 0.0, tripping the dt>=0 guard below). Keeping the clock
        # as an exact integer and round-tripping dt through it on each
        # predict() call eliminates the drift instead of just tolerating it.
        self._t_us: int | None = None

    def init(self, t_us: int, z, R, v0_std: float) -> None:
        """Position from z; velocity 0; covariance = R for position,
        v0_std^2 for velocity -- deliberately large/uninformative, since a
        single point carries no velocity information at all."""
        z = np.asarray(z, dtype=float)
        R = np.asarray(R, dtype=float)
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = R
        self.P[2, 2] = float(v0_std) ** 2
        self.P[3, 3] = float(v0_std) ** 2
        self._t_us = int(t_us)

    @staticmethod
    def _F(dt: float) -> np.ndarray:
        return np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def _Q(self, dt: float) -> np.ndarray:
        q = self.sigma_a**2
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        block = np.array([[dt4 / 4.0, dt3 / 2.0], [dt3 / 2.0, dt2]]) * q
        Q = np.zeros((4, 4))
        Q[np.ix_([0, 2], [0, 2])] = block  # x, vx
        Q[np.ix_([1, 3], [1, 3])] = block  # y, vy
        return Q

    def predict(self, dt: float) -> None:
        """Advance state/covariance by dt (seconds), mutating in place.
        Also advances this filter's own internal clock, so state_at() can
        later compute how far forward it needs to peek -- the public
        signature stays dt-based per step03's spec; the absolute-time
        bookkeeping is purely internal, and rounds dt to the nearest
        microsecond when updating the clock so repeated calls can't drift
        (see the _t_us field comment in __init__)."""
        if dt < -1e-6:
            raise ValueError(f"predict(dt={dt}) -- dt must be >= 0 (G15: no looking backward)")
        dt = max(dt, 0.0)
        if dt == 0:
            return
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        self._t_us += round(dt * 1e6)

    def update(self, z, R, H=None) -> None:
        """Correct the current (already-predicted) state with a
        measurement. R is an argument of the call, never stored (G6). H
        defaults to the position-only observation matrix (_H below) --
        every official run in this project (every adapter, every report
        through step11) uses exactly this default, unchanged. A caller
        may pass a different H (with matching z/R dimension) for a
        genuine multi-channel measurement -- e.g. the radar-Doppler-
        velocity ablation (G17, RADAR_DOPPLER_ABLATION), which measures
        [x,y,vx,vy] at once via H=_H4 (module-level identity). One
        update() implementation for both cases, not two parallel ones.

        Gating (innovation_cov/mahalanobis_sq below) always stays
        position-only regardless of what H an update() call uses --
        association is a position question, never conditioned on which
        measurement channels a specific update happens to carry."""
        H = _H if H is None else np.asarray(H, dtype=float)
        z = np.asarray(z, dtype=float)
        R = np.asarray(R, dtype=float)

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        # Joseph form: P stays symmetric PSD by construction even if K is
        # not the exact optimal gain (e.g. under R misspecification) --
        # more numerically robust than the simple (I - K.H) @ P form.
        A = _I4 - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = (self.P + self.P.T) / 2.0  # re-symmetrise (step03 spec)

    def state_at(self, t_us: int) -> "ConstantVelocityKF":
        """Returns a COPY predicted to t_us, without mutating self -- used
        for gating (peek before committing to a match) and later for
        extrapolation (step09)."""
        clone = copy.deepcopy(self)
        dt = (int(t_us) - clone._t_us) / 1e6
        clone.predict(dt)
        return clone

    def innovation_cov(self, R) -> np.ndarray:
        """S = H . P . H^T + R -- the correct gating quantity (G7), never P alone."""
        R = np.asarray(R, dtype=float)
        return _H @ self.P @ _H.T + R

    def mahalanobis_sq(self, z, R) -> float:
        """Squared Mahalanobis distance of measurement z from the current
        predicted state, scaled by the innovation covariance S = P + R."""
        z = np.asarray(z, dtype=float)
        S = self.innovation_cov(R)
        y = z - _H @ self.x
        return float(y @ np.linalg.inv(S) @ y)

    @property
    def position(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:].copy()
