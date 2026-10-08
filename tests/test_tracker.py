"""
tests/test_tracker.py — step06_tracker.md §5 (synthetic scenarios only).
"""
from __future__ import annotations

import copy
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.stats import chi2

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scripts"))

from ttcf import config
from ttcf.tracking.tracker import ShortMemoryTracker
from ttcf.types import Detection

from synthetic_scenarios import constant_velocity_scans, single_object_scans, true_position

OUT_FIGS = Path("outputs/figs/step06")


def _run(tracker, scans):
    all_snapshots = []
    for t_us, dets in scans:
        snaps = tracker.process_scan(dets, t_us)
        all_snapshots.append((t_us, snaps))
    return all_snapshots


# ── Test 1: single object, alternating sensors -> one track, converges ───

def test_single_object_converges_to_true_velocity_and_saves_figure():
    x0, y0, vx, vy = 0.0, 0.0, 5.0, 2.0
    t0_us, dt_us, n_scans = 0, 100_000, 30
    scans = single_object_scans(x0, y0, vx, vy, t0_us, dt_us, n_scans, rng=np.random.default_rng(42))

    tracker = ShortMemoryTracker()
    history = _run(tracker, scans)

    final_t, final_snaps = history[-1]
    assert len(final_snaps) == 1
    snap = final_snaps[0]
    assert np.linalg.norm(snap.x[2:4] - np.array([vx, vy])) < 1.0

    # Figure: true vs estimated trajectory.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    true_xy = np.array([true_position(x0, y0, vx, vy, t, t0_us) for t, _ in scans])
    est_xy = []
    for t_us, snaps in history:
        if snaps:
            est_xy.append(snaps[0].x[:2])
    est_xy = np.array(est_xy)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(true_xy[:, 0], true_xy[:, 1], "o-", label="true", color="black")
    ax.plot(est_xy[:, 0], est_xy[:, 1], "x-", label="tracked", color="crimson")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title("Test 1: single object, alternating sensors -- true vs tracked")
    ax.axis("equal")
    OUT_FIGS.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_FIGS / "test1_trajectory.png", dpi=130)
    plt.close(fig)


# ── Test 2: S = P + R regression, with a parametrised "wrong" variant ────

def test_s_equals_p_plus_r_accepts_high_r_detection_after_tight_track():
    tracker = ShortMemoryTracker()
    # Two low-R (sigma=0.1) updates first, to make the track's velocity
    # (and hence predicted position) genuinely tight.
    tracker.process_scan(
        [Detection(0, "TIGHT", "synthetic", np.array([10.0, 0.0]), np.diag([0.01, 0.01]),
                    "nearest_surface_to_ego", None, None, {})],
        0,
    )
    tracker.process_scan(
        [Detection(100_000, "TIGHT", "synthetic", np.array([10.5, 0.0]), np.diag([0.01, 0.01]),
                    "nearest_surface_to_ego", None, None, {})],
        100_000,
    )
    (tid, track) = next(iter(tracker._tracks.items()))
    peeked = track.kf.state_at(200_000)
    p_pred_diag = peeked.P[0, 0]
    print(f"\nP_pred after 2 tight updates: {p_pred_diag:.4f}")

    # A high-R (sigma=1.5) detection, offset enough to fail a P-alone
    # gate but pass the correct S = P + R gate.
    offset = 1.2
    z = peeked.x[:2] + np.array([offset, 0.0])
    R_high = np.diag([1.5**2, 1.5**2])

    d2_s = peeked.mahalanobis_sq(z, R_high)  # the CORRECT quantity (S = P + R)
    d2_p_alone = float(offset**2 / p_pred_diag)  # the WRONG quantity, computed for comparison only

    gate = config.CHI2_GATE.value
    print(f"d2 (S=P+R) = {d2_s:.3f}, d2 (P alone) = {d2_p_alone:.3f}, gate = {gate:.3f}")

    assert d2_s <= gate, "correct gate (S=P+R) should ACCEPT this detection"
    assert d2_p_alone > gate, "the WRONG gate (P alone) should REJECT it -- proving this test can fail"

    # And confirm the real tracker (which always uses S=P+R) actually matches it.
    snaps = tracker.process_scan(
        [Detection(200_000, "LOOSE", "synthetic", z, R_high, "nearest_surface_to_ego", None, None, {})],
        200_000,
    )
    assert len(snaps) == 1  # matched, not spawned as a second track
    assert snaps[0].n_updates == 3


# ── Test 3: two objects 10m apart -> two tracks ───────────────────────────

def test_two_objects_ten_meters_apart_give_two_tracks():
    objects = [(0.0, 0.0, 5.0, 0.0), (10.0, 0.0, 5.0, 0.0)]
    scans = constant_velocity_scans(objects, 0, 100_000, 20, rng=np.random.default_rng(1))
    tracker = ShortMemoryTracker()
    history = _run(tracker, scans)
    final_snaps = history[-1][1]
    assert len(final_snaps) == 2


# ── Test 4: crowd conflation -- named open limitation, NOT fixed ─────────

def test_crowd_conflation_named_open_limitation():
    """Two same-class objects ~0.8m apart, moving similarly. This is a
    KNOWN, NAMED limitation (what-not-to-do.md §4, lessons-from-v1-v2.md
    item C.3) -- position+class alone cannot always tell apart two close
    objects, and short memory does not prevent a bad FIRST match. This
    test documents the observed behaviour; it does not assert a 'correct'
    outcome, and this limitation is NOT being fixed here.

    Object 1 gets a head start (an already-established track) before
    object 2 first appears 0.8m away -- the actual failure mode this
    limitation describes is a NEW nearby object's first detection being
    absorbed into an EXISTING track, not two simultaneously-spawned
    tracks swapping later (which Hungarian's global optimum handles
    fairly well). A version of this test without the head start (both
    objects present from scan 0) did NOT reproduce conflation with this
    generator/seed -- also a legitimate, worth-recording observation
    about when this risk does and doesn't bite, not a reason to keep
    tuning the scenario until it "fails" on demand."""
    rng = np.random.default_rng(7)
    sigma = 0.5
    R = np.diag([sigma**2, sigma**2])
    tracker = ShortMemoryTracker()

    def _det(t_us, xy):
        z = xy + rng.normal(scale=sigma, size=2)
        return Detection(t_us, "S", "synthetic", z, R, "nearest_surface_to_ego", None, None, {})

    # Object 1 alone for 2 scans -- an established track before object 2 appears.
    for i in range(2):
        t_us = i * 100_000
        true1 = np.array([5.0 * (t_us / 1e6), 0.0])
        tracker.process_scan([_det(t_us, true1)], t_us)

    # From scan 2 on, object 2 appears 0.8m away, moving identically.
    # At its FIRST appearance (i==2), object 1 is NOT re-presented this
    # scan (simulating a missed detection that instant) -- this is the
    # actual ambiguous case: track1 has no closer alternative and may
    # absorb object 2's debut detection.
    original_track_id = "trk_000000"
    final_snaps = []
    final_t_us = None
    for i in range(2, 15):
        t_us = i * 100_000
        final_t_us = t_us
        true1 = np.array([5.0 * (t_us / 1e6), 0.0])
        true2 = np.array([5.0 * (t_us / 1e6), 0.8])
        dets = [_det(t_us, true2)] if i == 2 else [_det(t_us, true1), _det(t_us, true2)]
        final_snaps = tracker.process_scan(dets, t_us)

    true1_final = np.array([5.0 * (final_t_us / 1e6), 0.0])
    true2_final = np.array([5.0 * (final_t_us / 1e6), 0.8])
    original = next((s for s in final_snaps if s.track_id == original_track_id), None)

    print(
        f"\nCrowd conflation scenario: object 2 appears 0.8m from object 1's "
        f"ALREADY-ESTABLISHED track ({original_track_id}), with no competing "
        f"detection for object 1 at that instant. {len(final_snaps)} final track(s)."
    )
    if original is not None:
        d_true1 = float(np.linalg.norm(original.x[:2] - true1_final))
        d_true2 = float(np.linalg.norm(original.x[:2] - true2_final))
        print(
            f"  {original_track_id}'s final position is {d_true1:.2f}m from object 1's "
            f"true path and {d_true2:.2f}m from object 2's -- "
            f"{'HIJACKED onto object 2' if d_true2 < d_true1 else 'stayed on object 1'}."
        )
        print(
            "  Finding: NOT simply '1 track instead of 2' -- the original track's "
            "IDENTITY was captured by the wrong object at the first ambiguous match, "
            "and track count alone (still 2 here) does not surface this. Documented "
            "as the real observed failure mode; not fixed (what-not-to-do.md §4, "
            "lessons-from-v1-v2.md item C.3)."
        )
    # Documents whichever outcome occurs for this scenario/seed -- this is
    # a demonstration of a named, accepted limitation, not a correctness
    # requirement, so no assertion is made about WHICH object either
    # track ends up following. The observed fact (for this seed: identity
    # hijack) is reported above, not asserted as universal behaviour.
    assert len(final_snaps) in (1, 2)


# ── Test 5: eviction after EVICTION_GAP_S; later detection is a NEW track ─

def test_eviction_then_new_track_gets_new_id():
    tracker = ShortMemoryTracker()
    snaps = tracker.process_scan(
        [Detection(0, "A", "synthetic", np.array([10.0, 0.0]), np.diag([0.1, 0.1]),
                    "nearest_surface_to_ego", None, None, {})],
        0,
    )
    old_id = snaps[0].track_id

    gap_us = int(config.EVICTION_GAP_S.value * 1e6)
    past_gap_t = gap_us + 100_000  # comfortably past the eviction gap
    snaps_empty = tracker.process_scan([], past_gap_t)
    assert snaps_empty == []  # old track evicted, nothing left
    assert tracker.n_evictions == 1

    snaps_new = tracker.process_scan(
        [Detection(past_gap_t + 100_000, "A", "synthetic", np.array([10.0, 0.0]), np.diag([0.1, 0.1]),
                    "nearest_surface_to_ego", None, None, {})],
        past_gap_t + 100_000,
    )
    assert len(snaps_new) == 1
    assert snaps_new[0].track_id != old_id


# ── Test 6: one track updated at most once per scan ───────────────────────

def test_one_track_updated_at_most_once_per_scan():
    tracker = ShortMemoryTracker()
    tracker.process_scan(
        [Detection(0, "A", "synthetic", np.array([10.0, 0.0]), np.diag([0.1, 0.1]),
                    "nearest_surface_to_ego", None, None, {})],
        0,
    )
    snaps1 = tracker.process_scan(
        [Detection(100_000, "A", "synthetic", np.array([10.0, 0.0]), np.diag([0.1, 0.1]),
                    "nearest_surface_to_ego", None, None, {})],
        100_000,
    )
    n_updates_before = snaps1[0].n_updates

    # Two close detections in the SAME scan, both within gate of the track.
    dets = [
        Detection(200_000, "A", "synthetic", np.array([10.05, 0.0]), np.diag([0.1, 0.1]),
                   "nearest_surface_to_ego", None, None, {}),
        Detection(200_000, "A", "synthetic", np.array([9.95, 0.0]), np.diag([0.1, 0.1]),
                   "nearest_surface_to_ego", None, None, {}),
    ]
    snaps2 = tracker.process_scan(dets, 200_000)

    assert len(snaps2) == 2  # one matched track + one newly spawned track
    matched = [s for s in snaps2 if s.n_updates == n_updates_before + 1]
    assert len(matched) == 1


# ── Test 7: ego-motion -- stationary object -> track velocity ~= 0 ───────

def test_stationary_object_velocity_converges_to_zero():
    # Detections are already global-frame (G1); the tracker never touches
    # ego state at all, so "ego moves while this object doesn't" is
    # simply a zero-velocity object -- ego motion never enters this math.
    scans = single_object_scans(20.0, 0.0, 0.0, 0.0, 0, 100_000, 25, rng=np.random.default_rng(3))
    tracker = ShortMemoryTracker()
    history = _run(tracker, scans)
    final_snaps = history[-1][1]
    assert len(final_snaps) == 1
    assert np.linalg.norm(final_snaps[0].x[2:4]) < 0.5


# ── Test 8: aux stripped => identical outputs ─────────────────────────────

def test_aux_stripped_gives_identical_results():
    def build_scans(with_aux: bool):
        scans = single_object_scans(0.0, 0.0, 4.0, -1.0, 0, 100_000, 15, rng=np.random.default_rng(9))
        if with_aux:
            new_scans = []
            for t_us, dets in scans:
                new_dets = [
                    Detection(d.t_us, d.channel, d.modality, d.xy_global, d.R_global,
                               d.ref_point_kind, d.cls, d.n_points, {"doppler": 99.0, "junk": "x"})
                    for d in dets
                ]
                new_scans.append((t_us, new_dets))
            return new_scans
        return scans

    scans_with_aux = build_scans(True)
    scans_without_aux = build_scans(False)

    tracker_a = ShortMemoryTracker()
    tracker_b = ShortMemoryTracker()
    history_a = _run(tracker_a, scans_with_aux)
    history_b = _run(tracker_b, scans_without_aux)

    for (ta, snaps_a), (tb, snaps_b) in zip(history_a, history_b):
        assert ta == tb
        assert len(snaps_a) == len(snaps_b)
        for sa, sb in zip(snaps_a, snaps_b):
            np.testing.assert_array_equal(sa.x, sb.x)
            np.testing.assert_array_equal(sa.P, sb.P)


# ── Test 9: determinism -- same input -> same output ──────────────────────

def test_determinism_same_input_same_output():
    scans = single_object_scans(5.0, 5.0, -2.0, 3.0, 0, 100_000, 20, rng=np.random.default_rng(11))
    scans_copy = copy.deepcopy(scans)

    tracker_a = ShortMemoryTracker()
    tracker_b = ShortMemoryTracker()
    history_a = _run(tracker_a, scans)
    history_b = _run(tracker_b, scans_copy)

    for (ta, snaps_a), (tb, snaps_b) in zip(history_a, history_b):
        assert ta == tb
        assert len(snaps_a) == len(snaps_b)
        for sa, sb in zip(snaps_a, snaps_b):
            assert sa.track_id == sb.track_id
            np.testing.assert_array_equal(sa.x, sb.x)
            np.testing.assert_array_equal(sa.P, sb.P)


# ── Test 10: velocity jitter on a stationary object (informational) ──────

def test_velocity_jitter_on_stationary_object_informational():
    """Measures velocity jitter on a well-observed stationary synthetic
    object -- informs MIN_TRUSTED_SPEED_MPS / MIN_CLOSING_SPEED_MPS /
    MAX_VEL_STD_MPS, but these config values stay PLACEHOLDER until
    measured on REAL data (step06 §5 item 10) -- this is reporting only,
    not a config update."""
    scans = single_object_scans(
        15.0, 0.0, 0.0, 0.0, 0, 100_000, 40,
        sigmas=(0.1, 0.5, 1.5), rng=np.random.default_rng(21),
    )
    tracker = ShortMemoryTracker()
    history = _run(tracker, scans)

    tail_speeds = [np.linalg.norm(snaps[0].x[2:4]) for _, snaps in history[-15:] if snaps]
    mean_jitter = float(np.mean(tail_speeds))
    std_jitter = float(np.std(tail_speeds))
    print(
        f"\nStationary-object velocity jitter (last 15 snapshots): "
        f"mean={mean_jitter:.4f} m/s, std={std_jitter:.4f} m/s "
        f"(config MIN_TRUSTED_SPEED_MPS/MIN_CLOSING_SPEED_MPS/MAX_VEL_STD_MPS "
        f"stay PLACEHOLDER -- this is synthetic, not real-data, measurement)"
    )
    assert mean_jitter < 2.0  # loose sanity bound, not a config-setting threshold


# ── Test 11: G17 Doppler ablation -- combined update via _kf_update_args ──

def test_detection_without_velocity_field_is_position_only_update():
    """Default path (every official run): velocity_global/R_velocity_global
    both None -> _kf_update_args must hand back exactly (xy_global, R_global,
    H=None), i.e. the tracker's pre-G17 call shape, unchanged."""
    from ttcf.tracking.tracker import _kf_update_args

    det = Detection(
        0, "A", "synthetic", np.array([10.0, 5.0]), np.diag([0.1, 0.1]),
        "nearest_surface_to_ego", None, None, {},
    )
    z, R, H = _kf_update_args(det)
    np.testing.assert_array_equal(z, det.xy_global)
    np.testing.assert_array_equal(R, det.R_global)
    assert H is None


def test_detection_with_velocity_field_builds_combined_4d_measurement():
    """Doppler-ablation path: both velocity fields set -> a 4D z, a 4x4
    block-diagonal R (position block = R_global, velocity block =
    R_velocity_global, zero cross-terms -- position and Doppler are
    independent measurements), and H=H4 (full-state identity)."""
    from ttcf.filtering.kalman import H4
    from ttcf.tracking.tracker import _kf_update_args

    xy = np.array([10.0, 5.0])
    R_pos = np.diag([0.1, 0.1])
    vel = np.array([8.0, -1.0])
    R_vel = np.diag([1.0, 1.0])
    det = Detection(
        0, "RADAR_FRONT", "radar", xy, R_pos, "nearest_surface_to_ego", None, None, {},
        velocity_global=vel, R_velocity_global=R_vel,
    )
    z, R, H = _kf_update_args(det)

    np.testing.assert_array_equal(z, np.array([10.0, 5.0, 8.0, -1.0]))
    np.testing.assert_array_equal(H, H4)
    np.testing.assert_array_equal(R[:2, :2], R_pos)
    np.testing.assert_array_equal(R[2:, 2:], R_vel)
    np.testing.assert_array_equal(R[:2, 2:], np.zeros((2, 2)))
    np.testing.assert_array_equal(R[2:, :2], np.zeros((2, 2)))


def test_tracker_with_doppler_detection_converges_velocity_in_one_scan():
    """End-to-end through process_scan (not just the helper): a single
    scan carrying a genuine Doppler-bearing detection on a freshly spawned
    track updates that SAME scan (spawn uses position-only init, per
    step06's own spec -- the combined update only applies to an existing
    track's matched update). So this seeds a track on scan 1 (position
    only, as every spawn does), then on scan 2 feeds a Doppler detection
    consistent with the true velocity and checks the matched update pulls
    the KF's velocity noticeably toward the measured value -- far faster
    than position-only updates could over just one more scan."""
    tracker = ShortMemoryTracker()
    R_pos = np.diag([0.2, 0.2])
    tracker.process_scan(
        [Detection(0, "RADAR_FRONT", "radar", np.array([0.0, 0.0]), R_pos,
                    "nearest_surface_to_ego", None, None, {})],
        0,
    )

    R_vel = np.diag([0.5, 0.5])
    det2 = Detection(
        100_000, "RADAR_FRONT", "radar", np.array([1.0, 0.0]), R_pos,
        "nearest_surface_to_ego", None, None, {},
        velocity_global=np.array([10.0, 0.0]), R_velocity_global=R_vel,
    )
    snaps = tracker.process_scan([det2], 100_000)

    assert len(snaps) == 1
    assert snaps[0].x[2] > 5.0  # vx pulled well toward the 10 m/s Doppler measurement
