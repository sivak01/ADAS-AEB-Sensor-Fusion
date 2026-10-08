"""
tests/test_kalman.py — step03_kf_ego_state.md §6, items 1-8 (synthetic).
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from ttcf.filtering.kalman import ConstantVelocityKF, H4

SIGMA_A = 1.0
V0_STD = 10.0


def _new_kf(t0_us, z0, R0, sigma_a=SIGMA_A, v0_std=V0_STD):
    kf = ConstantVelocityKF(sigma_a=sigma_a)
    kf.init(t0_us, z0, R0, v0_std)
    return kf


# ── Test 1: noise-free constant-velocity input -> velocity converges ─────

def test_noise_free_constant_velocity_converges():
    true_v = np.array([5.0, -3.0])
    dt = 0.1
    R = np.diag([1e-8, 1e-8])  # ~noise-free measurement
    pos = np.array([0.0, 0.0])
    kf = _new_kf(0, pos, R)

    t_us = 0
    for _ in range(30):
        t_us += int(dt * 1e6)
        pos = pos + true_v * dt
        kf.predict(dt)
        kf.update(pos, R)

    np.testing.assert_allclose(kf.velocity, true_v, atol=1e-3)


# ── Test 2: noisy input -> velocity error shrinks with more updates ──────

def test_noisy_input_velocity_error_shrinks_with_updates():
    true_v = np.array([4.0, 2.0])
    dt = 0.1
    sigma_meas = 0.3
    R = np.diag([sigma_meas**2, sigma_meas**2])
    checkpoints = [2, 5, 20]
    rng = np.random.default_rng(1)
    n_runs = 200

    errors_at = {n: [] for n in checkpoints}
    for _run in range(n_runs):
        pos_true = np.array([0.0, 0.0])
        kf = _new_kf(0, pos_true + rng.normal(scale=sigma_meas, size=2), R)
        for i in range(1, max(checkpoints) + 1):
            pos_true = pos_true + true_v * dt
            z = pos_true + rng.normal(scale=sigma_meas, size=2)
            kf.predict(dt)
            kf.update(z, R)
            if i in checkpoints:
                errors_at[i].append(np.linalg.norm(kf.velocity - true_v))

    mean_errors = {n: float(np.mean(errors_at[n])) for n in checkpoints}
    print(f"\nmean velocity error by update count: {mean_errors}")

    assert mean_errors[20] < mean_errors[5] < mean_errors[2]


# ── Test 3: predict-only grows position variance; update shrinks it ──────

def test_predict_grows_variance_update_shrinks_it():
    R = np.diag([0.1, 0.1])
    kf = _new_kf(0, [0.0, 0.0], R)
    var_before_predict = kf.P[0, 0]

    kf.predict(1.0)
    var_after_predict = kf.P[0, 0]
    assert var_after_predict > var_before_predict

    kf.update(np.array([0.05, 0.0]), R)
    var_after_update = kf.P[0, 0]
    assert var_after_update < var_after_predict


# ── Test 4: huge R barely moves the state; tiny R snaps to measurement ───

def test_huge_r_barely_updates_tiny_r_snaps():
    R_small_init = np.diag([0.1, 0.1])
    kf_huge = _new_kf(0, [0.0, 0.0], R_small_init)
    kf_huge.predict(0.1)
    x_before = kf_huge.x.copy()
    kf_huge.update(np.array([100.0, 100.0]), np.diag([1e8, 1e8]))
    assert np.linalg.norm(kf_huge.x[:2] - x_before[:2]) < 1e-2

    kf_tiny = _new_kf(0, [0.0, 0.0], R_small_init)
    kf_tiny.predict(0.1)
    kf_tiny.update(np.array([100.0, 100.0]), np.diag([1e-10, 1e-10]))
    np.testing.assert_allclose(kf_tiny.x[:2], [100.0, 100.0], atol=1e-3)


# ── Test 5: P stays symmetric PSD after 1000 steps ────────────────────────

def test_p_stays_symmetric_psd_after_1000_steps():
    rng = np.random.default_rng(2)
    R = np.diag([0.2, 0.2])
    kf = _new_kf(0, [0.0, 0.0], R)
    for _ in range(1000):
        kf.predict(0.05)
        if rng.random() < 0.7:  # not every step has a real update
            kf.update(kf.position + rng.normal(scale=0.2, size=2), R)

        np.testing.assert_allclose(kf.P, kf.P.T, atol=1e-8)
        eigvals = np.linalg.eigvalsh(kf.P)
        assert np.all(eigvals >= -1e-8), f"P not PSD: eigenvalues {eigvals}"


# ── Test 6: S = P + R gating -- reproduce the worked example ──────────────

def test_innovation_covariance_gating_worked_example():
    # Prediction at 20.0 m, P_xx = 0.09 (sigma 0.3); detection at 21.0 m,
    # R_xx = 0.64 (sigma 0.8). y-axis is set to match exactly (residual 0)
    # with an arbitrary nonzero variance, so it contributes 0 to the
    # quadratic form regardless of its value -- isolating the test to the
    # x-axis exactly as the 1-D worked example intends.
    kf = ConstantVelocityKF(sigma_a=SIGMA_A)
    kf.x = np.array([20.0, 0.0, 0.0, 0.0])
    kf.P = np.diag([0.09, 1.0, 100.0, 100.0])
    kf._t_us = 0

    z = np.array([21.0, 0.0])
    R = np.diag([0.64, 1.0])

    # The WRONG old behaviour: gate on P alone (residual^2 / P_xx).
    d2_p_alone = (z[0] - kf.x[0]) ** 2 / kf.P[0, 0]
    # The correct behaviour: gate on S = P + R (via the class's own method).
    d2_s = kf.mahalanobis_sq(z, R)

    limit_1d = chi2.ppf(0.99, df=1)  # ~6.635

    assert d2_p_alone == pytest.approx(11.111, abs=0.01)
    assert d2_s == pytest.approx(1.370, abs=0.01)
    assert d2_p_alone > limit_1d, "gating on P alone should REJECT this match"
    assert d2_s <= limit_1d, "gating on S = P + R should ACCEPT this match"

    S = kf.innovation_cov(R)
    np.testing.assert_allclose(S, kf.P[:2, :2] + R)


# ── Test 7: NEES consistency over many Monte-Carlo runs ──────────────────

def test_nees_consistency_monte_carlo():
    rng = np.random.default_rng(3)
    sigma_a = 1.5
    sigma_meas = 0.4
    R = np.diag([sigma_meas**2, sigma_meas**2])
    dt = 0.1
    n_steps = 15
    n_runs = 300
    state_dim = 4

    nees_values = []
    for _run in range(n_runs):
        x_true = np.array([0.0, 0.0, 0.0, 0.0])
        z0 = x_true[:2] + rng.normal(scale=sigma_meas, size=2)
        kf = _new_kf(0, z0, R, sigma_a=sigma_a)

        for _ in range(n_steps):
            F = kf._F(dt)
            Q = kf._Q(dt)
            # Process noise sampled from the FILTER's own Q -- validates
            # the filter's internal self-consistency (Q/R match what the
            # update math assumes), not whether the discretisation
            # approximates real continuous dynamics.
            w = rng.multivariate_normal(np.zeros(state_dim), Q)
            x_true = F @ x_true + w

            kf.predict(dt)
            z = x_true[:2] + rng.normal(scale=sigma_meas, size=2)
            kf.update(z, R)

        err = x_true - kf.x
        nees = float(err @ np.linalg.inv(kf.P) @ err)
        nees_values.append(nees)

    mean_nees = float(np.mean(nees_values))
    print(f"\nmean NEES over {n_runs} runs: {mean_nees:.3f} (state dim = {state_dim})")
    # Loose band: exact convergence to 4.0 has real Monte-Carlo variance;
    # this checks the filter isn't wildly over- or under-confident.
    assert 2.5 < mean_nees < 6.0


# ── Test 8: state_at does not mutate the filter ───────────────────────────

def test_state_at_does_not_mutate():
    R = np.diag([0.2, 0.2])
    kf = _new_kf(0, [1.0, 2.0], R)
    kf.predict(0.5)
    kf.update(np.array([1.2, 2.1]), R)

    x_before = kf.x.copy()
    P_before = kf.P.copy()
    t_before = kf._t_us

    peeked = kf.state_at(int(5e6))  # 5 seconds forward

    np.testing.assert_array_equal(kf.x, x_before)
    np.testing.assert_array_equal(kf.P, P_before)
    assert kf._t_us == t_before
    assert not np.allclose(peeked.x, kf.x)  # actually moved forward


# ── Test 9: default update() (H=None) is numerically identical to before ──
# (G17 ablation support must not change a single existing call site's result)

def test_default_update_unchanged_by_optional_h_param():
    R = np.diag([0.3, 0.3])
    kf_a = _new_kf(0, [1.0, 2.0], R)
    kf_b = _new_kf(0, [1.0, 2.0], R)
    kf_a.predict(0.5)
    kf_b.predict(0.5)

    kf_a.update(np.array([1.3, 2.2]), R)          # old call style
    kf_b.update(np.array([1.3, 2.2]), R, H=None)  # new default, explicit

    np.testing.assert_array_equal(kf_a.x, kf_b.x)
    np.testing.assert_array_equal(kf_a.P, kf_b.P)


# ── Test 10: combined position+velocity update (G17 Doppler ablation) ────

def test_combined_position_velocity_update_moves_velocity_toward_measurement():
    """A fresh track's velocity starts at exactly 0 (KF.init's own
    convention). A single combined update with a genuine velocity
    measurement should pull the estimate noticeably toward that
    measurement -- something a position-only update could never do in
    one step (position alone carries no velocity information at all)."""
    R_pos = np.diag([0.3, 0.3])
    kf = _new_kf(0, [10.0, 0.0], R_pos, v0_std=10.0)
    assert kf.x[2] == 0.0 and kf.x[3] == 0.0  # starts at zero velocity

    kf.predict(0.1)
    z = np.array([10.0, 0.0, 8.0, 0.0])  # position unchanged, vx=8 m/s measured directly
    R_vel = np.diag([1.0, 1.0])
    R_combined = np.zeros((4, 4))
    R_combined[:2, :2] = R_pos
    R_combined[2:, 2:] = R_vel
    kf.update(z, R_combined, H=H4)

    assert kf.x[2] > 4.0  # velocity pulled substantially toward the 8 m/s measurement
    assert abs(kf.x[3]) < 1.0  # vy measurement was 0 -- should stay near 0


def test_combined_update_matches_sequential_logic_on_a_simple_case():
    """Sanity check against hand expectation: with equal, large position
    noise and a precise velocity measurement, a combined update should
    trust the velocity measurement almost completely (final vx close to
    the measured 8.0, not stuck near the 0.0 prior)."""
    R_pos = np.diag([100.0, 100.0])  # deliberately uninformative position R
    kf = _new_kf(0, [10.0, 0.0], R_pos, v0_std=10.0)
    kf.predict(0.1)
    z = np.array([10.0, 0.0, 8.0, 0.0])
    R_combined = np.zeros((4, 4))
    R_combined[:2, :2] = R_pos
    R_combined[2:, 2:] = np.diag([0.01, 0.01])  # very precise velocity measurement
    kf.update(z, R_combined, H=H4)

    assert kf.x[2] == pytest.approx(8.0, abs=0.5)
