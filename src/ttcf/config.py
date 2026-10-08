"""
src/ttcf/config.py — single source of truth for every instant-ttc-fusion constant.

Every tunable is a Param(value, status, reason, tunable_on). status is one
of MEASURED, ASSUMED, or PLACEHOLDER (G12) — never let a guess masquerade
as a measurement (lessons-from-v1-v2.md item A.10). PLACEHOLDER additionally
names, in its reason, the later step that must replace it with a real
measurement. No numeric constant used by the pipeline lives outside this
file (G12) — this is the ONE place to change a value.
"""
from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import scipy.stats

Status = Literal["MEASURED", "ASSUMED", "PLACEHOLDER"]
TunableOn = Literal["tune_split", "never"]


@dataclass(frozen=True)
class Param:
    value: Any
    status: Status
    reason: str
    tunable_on: TunableOn = "never"


# Every Param, keyed by the dotted name used in describe()/freeze() and by
# tests/test_config.py's coverage checks. Populated by _param() below as
# each constant is defined — never edited directly.
_REGISTRY: dict[str, Param] = {}


def _param(name: str, value: Any, status: Status, reason: str, tunable_on: TunableOn = "never") -> Param:
    if not reason:
        raise ValueError(f"Param {name!r} has an empty reason")
    p = Param(value=value, status=status, reason=reason, tunable_on=tunable_on)
    _REGISTRY[name] = p
    return p


# ── Dataset root discovery ───────────────────────────────────────────────
# BASE_DIR anchors to this file's own location, not the caller's cwd — a
# notebook or script launched from a different working directory must not
# silently resolve DATAROOT differently (the pattern V1's config.py used
# for its own BASE_DIR, kept here for the same reason: it's a correctness
# fix independent of anything scope-specific).
BASE_DIR = Path(__file__).resolve().parent.parent.parent  # src/ttcf/config.py -> repo root


def _has_nuscenes_layout(candidate: Path) -> bool:
    """True if `candidate` directly contains samples/, sweeps/, and a
    v1.0-* version folder with sample_data.json — the on-disk shape
    step00_config_contracts.md §3 asks the preflight to confirm, checked
    here too since DATAROOT resolution must not assume it blindly."""
    if not candidate.is_dir():
        return False
    if not (candidate / "samples").is_dir() or not (candidate / "sweeps").is_dir():
        return False
    for entry in candidate.iterdir():
        if entry.is_dir() and entry.name.startswith("v1.0") and (entry / "sample_data.json").exists():
            return True
    return False


def discover_dataroot(search_root: Path, max_depth: int = 3) -> Path | None:
    """Breadth-first search up to `max_depth` levels below `search_root`
    for the folder with the nuScenes layout — the dataset may be a
    Kaggle-style download with a nested folder, so this does not assume
    `search_root` itself is the right folder (step00 §3.1)."""
    frontier = [search_root]
    for _ in range(max_depth + 1):
        next_frontier: list[Path] = []
        for candidate in frontier:
            if _has_nuscenes_layout(candidate):
                return candidate
            if candidate.is_dir():
                try:
                    next_frontier.extend(d for d in candidate.iterdir() if d.is_dir())
                except PermissionError:
                    continue
        frontier = next_frontier
    return None


_env_dataroot = os.environ.get("TTCF_DATAROOT")
if _env_dataroot:
    DATAROOT = Path(_env_dataroot)
else:
    _discovered = discover_dataroot(BASE_DIR / "archive")
    # Falls back to BASE_DIR / "archive" even if undiscovered so that
    # importing config.py never raises — preflight_dataset.py is the
    # place that STOPs loudly if the layout truly isn't there (step00 §3.3).
    DATAROOT = _discovered if _discovered is not None else (BASE_DIR / "archive")

NUSCENES_VERSION = "v1.0-mini"


# ── DEC-3: active channels ───────────────────────────────────────────────
ACTIVE_CHANNELS = _param(
    "ACTIVE_CHANNELS",
    ["LIDAR_TOP", "RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT", "CAM_FRONT"],
    "ASSUMED",
    "DEC-3, approved as proposed: forward corridor only (README_MASTER.md §6) — "
    "a forward-TTC/AEB design has no use for side/rear sensors.",
)

# ── DEC-2: GT category scope ──────────────────────────────────────────────
# Every vehicle.* and human.pedestrian.* leaf category in the INSTALLED
# devkit's full category taxonomy (nusc.category), verified at stepD0 —
# not just the subset with >=1 annotation in nuScenes-mini (list_categories()
# only prints populated ones; this is deliberately the full prefix match
# against the installed category.json, so it stays correct even against a
# larger split where a currently-empty leaf category, e.g.
# vehicle.emergency.ambulance, does have real annotations).
GT_CATEGORIES = _param(
    "GT_CATEGORIES",
    [
        "human.pedestrian.adult",
        "human.pedestrian.child",
        "human.pedestrian.construction_worker",
        "human.pedestrian.personal_mobility",
        "human.pedestrian.police_officer",
        "human.pedestrian.stroller",
        "human.pedestrian.wheelchair",
        "vehicle.bicycle",
        "vehicle.bus.bendy",
        "vehicle.bus.rigid",
        "vehicle.car",
        "vehicle.construction",
        "vehicle.emergency.ambulance",
        "vehicle.emergency.police",
        "vehicle.motorcycle",
        "vehicle.trailer",
        "vehicle.truck",
    ],
    "ASSUMED",
    "DEC-2: GT events scoped to vehicle.* + human.pedestrian.* (collision-relevant "
    "categories; movable_object.*/static_object.* are typically static clutter). "
    "vehicle.trailer is INCLUDED under the plain prefix rule — no scoped exclusion "
    "was requested, and a towed trailer is part of a large vehicle's collision "
    "envelope; excluding it would need a 'has a cab' check this project doesn't "
    "build. Class is recorded on every row but never gates candidate selection in "
    "any of the four comparison runs (symmetric — what-not-to-do.md §6).",
)

# ── Measurement noise (R) ────────────────────────────────────────────────
# Values are (sigma_long, sigma_lat) in metres, ego-frame, per sensor.
SENSOR_R_LIDAR = _param(
    "SENSOR_R_LIDAR",
    {"sigma_long_m": 0.3, "sigma_lat_m": 0.3},
    "ASSUMED",
    "Literature-typical LiDAR range accuracy. A direct measurement (matched-track "
    "residual vs. sample_annotation GT) was attempted for this exact sensor in the "
    "predecessor project and found untrustworthy: LiDAR's own track fragmentation "
    "there (4.1x vs. real object count) meant only ~86% of matched points were real "
    "inliers, well below the >97% a robust estimator needs, so the resulting number "
    "(~2.0m) was physically implausible for LiDAR (lessons-from-v1-v2.md item A.10 / "
    "10). Re-measurement here is only justified once this project independently "
    "confirms low fragmentation under its own short-memory tracker (step06).",
)
SENSOR_R_RADAR = _param(
    "SENSOR_R_RADAR",
    {"sigma_long_m": 0.5, "sigma_lat_m": 1.5},
    "ASSUMED",
    "Literature-typical radar accuracy (good range resolution, poor angular/lateral "
    "resolution). Same non-measurability rationale as SENSOR_R_LIDAR above.",
)
SENSOR_R_CAMERA = _param(
    "SENSOR_R_CAMERA",
    None,
    "PLACEHOLDER",
    "Camera R is range-dependent (ground-plane back-projection error grows with "
    "range^2), not a fixed (sigma_long, sigma_lat) pair like LIDAR/RADAR's own "
    "SENSOR_R params above -- there is deliberately no static value here. The "
    "function itself (adapters/camera.py's _camera_R) is defined at step04c, "
    "parametrized by CAMERA_PIXEL_SIGMA_U_PX/CAMERA_PIXEL_SIGMA_V_PX below plus "
    "each detection's own range and the event's own focal length/camera height.",
)

# ── Process noise ─────────────────────────────────────────────────────────
SIGMA_A = _param(
    "SIGMA_A",
    0.5,
    "ASSUMED",
    "Shared object process-noise std (m/s^2). Coincides numerically with v2's own "
    "PROCESS_NOISE_SIGMA_A default range, but NOT inherited from it — lessons-from-"
    "v1-v2.md item C.1 says this tradeoff genuinely changes once track lifetime "
    "drops to 2-4 updates. Selected 2026-09-24 by a log-spaced grid sweep "
    "({0.5,1,2,4,8}) across all four streams on the tune split, scored by DEC-7's "
    "own pre-declared criterion (mean correct_action_rate across lidar/radar/"
    "camera/fused, missed_brake_rate as tie-breaker) -- 0.5 won outright "
    "(mean_correct=0.9430 vs. 0.9335-0.9399 for the other four values), no "
    "tie-breaker needed. The single-sensor streams' own correct_action_rate was "
    "essentially flat across the whole grid (0.9494 for lidar/radar/camera at "
    "every value) -- SIGMA_A's effect is concentrated in the fused stream, which "
    "varied 0.886-0.924 across the grid; full sweep logged in outputs/tuning_log.csv. "
    "Labelled ASSUMED, not MEASURED (this project's own MEASURED status is reserved "
    "for direct empirical measurement like sensor R, not a grid-search selection).",
    tunable_on="tune_split",
)
SIGMA_A_EGO = _param(
    "SIGMA_A_EGO",
    1.0,
    "ASSUMED",
    "Ego vehicle's own process-noise std (m/s^2) for the shared KF used to derive "
    "ego velocity from ego_pose (G5/G6). Ego motion is smoother/less erratic than a "
    "tracked object's, hence lower than SIGMA_A, but not yet measured.",
)
EGO_POSE_STD_M = _param(
    "EGO_POSE_STD_M",
    0.05,
    "ASSUMED",
    "nuScenes ego_pose comes from a fused localization stack (GPS/IMU/wheel "
    "odometry), far more accurate than any tracked sensor — literature-typical "
    "~5cm automotive localization std, not independently measured here.",
)
V0_STD_MPS = _param(
    "V0_STD_MPS",
    10.0,
    "ASSUMED",
    "Initial velocity std (m/s) used to initialise a new track's KF covariance — "
    "deliberately large/uninformative so the first real update dominates.",
)

# ── Gating ────────────────────────────────────────────────────────────────
_GATE_CONFIDENCE_LEVEL = 0.99
CHI2_GATE = _param(
    "CHI2_GATE",
    scipy.stats.chi2.ppf(_GATE_CONFIDENCE_LEVEL, df=2),
    "ASSUMED",
    f"Chi-square quantile at {_GATE_CONFIDENCE_LEVEL:.0%} confidence, df=2 — computed "
    "from scipy, not typed, so it can never silently drift from the confidence level "
    "it claims. Replaces a fixed Euclidean gate per G7/lessons item A.4 (gate on the "
    "innovation covariance S = P_pred + R, including the incoming detection's own "
    "noise). The 0.99 confidence level itself is an assumption, not measured.",
)

# ── Track lifecycle ──────────────────────────────────────────────────────
EVICTION_GAP_S = _param(
    "EVICTION_GAP_S",
    0.3,
    "ASSUMED",
    "~3-4 inter-arrival gaps of the slowest active sensor; identical in all four "
    "runs (G11). Deliberately NOT v2's MAX_MISSED_SECONDS=1.5 — this project's "
    "tracks are short-lived by design (2-4 updates), so lessons-from-v1-v2.md item "
    "C.1 says the old eviction tradeoff does not carry over unexamined.",
)
MAX_MEMORY_UPDATES = _param(
    "MAX_MEMORY_UPDATES",
    4,
    "ASSUMED",
    "DEC-5: sensors_in_estimate (step06) is the sensors of only the last "
    "MAX_MEMORY_UPDATES real updates -- a bounded recent-window set, not a "
    "lifetime-growing one. v2's CentralTracker tracked sensors_seen as a "
    "lifetime set; deliberately not carried over here (G11 — 'which sensors "
    "contributed to the CURRENT estimate', not a lifetime property).",
)
MIN_UPDATES_FOR_TTC = _param(
    "MIN_UPDATES_FOR_TTC",
    2,
    "ASSUMED",
    "Fixed by physics, not tunable: a rate (velocity) cannot be computed from fewer "
    "than 2 temporally-linked detections of the same object, regardless of sensor "
    "or algorithm (CLAUDE.md 'Does require').",
)
MAX_VEL_STD_MPS = _param(
    "MAX_VEL_STD_MPS",
    3.90,
    "ASSUMED",
    "Velocity-trust limit (m/s) — a track's velocity estimate is only trusted below "
    "this std. Measured 2026-09-24 (scripts/tune.py, step11 §4.6): the KF's own "
    "reported velocity std (worse of the two axis stds, matching "
    "is_velocity_eligible's own convention) across every real update of every "
    "track with >=MIN_UPDATES_FOR_TTC updates, pooled across all 4 streams on the "
    "tune split at SIGMA_A=0.5 (n=47,336). The 75th percentile is used -- trusts "
    "the majority of reasonably-mature tracks while excluding the clearly still-"
    "noisy tail (freshly-spawned/sparsely-updated tracks); this same threshold was "
    "also used to clean up the MIN_TRUSTED_SPEED_MPS/MIN_CLOSING_SPEED_MPS "
    "measurement (see their own reason fields). Labelled ASSUMED (a percentile "
    "choice from the KF's own reported uncertainty), not MEASURED against an "
    "independent ground truth.",
    tunable_on="tune_split",
)

# ── TTC deadbands ─────────────────────────────────────────────────────────
MIN_TRUSTED_SPEED_MPS = _param(
    "MIN_TRUSTED_SPEED_MPS",
    1.99,
    "ASSUMED",
    "Below this raw speed, a track's motion is treated as sensor/clustering jitter "
    "on a stationary object rather than real motion (G8). Measured 2026-09-24 "
    "(scripts/tune.py, step11 §4.6): matched real, stationary tune-split GT objects "
    "(box_velocity < 0.5 m/s) to the tracker's own KF speed estimate at each "
    "keyframe, pooled across all 4 streams at the chosen SIGMA_A=0.5 (n=38, after "
    "two G13 fixes below). Two real bugs found and fixed in the measurement itself "
    "before trusting it: (1) the raw sample was contaminated by freshly-spawned "
    "tracks at exactly MIN_UPDATES_FOR_TTC updates, where measurement noise over a "
    "tiny native-rate dt implies enormous meaningless velocities -- fixed by "
    "restricting to samples whose own KF vel_std already clears MAX_VEL_STD_MPS "
    "(exactly the population the real eligibility gate lets through); (2) even "
    "after that fix, a small residual tail remained from this diagnostic's own "
    "crude nearest-position GT-to-track matching occasionally picking up an "
    "unrelated, genuinely-moving nearby track in a crowded scene (confirmed "
    "directly: a track with 7 real updates and a modest vel_std still reporting "
    "~10 m/s next to a parked car) -- fixed with a tighter (0.75m) and unambiguous "
    "(next-nearest track >=1m farther) association. The resulting cleaned sample is "
    "visibly bimodal (a tight cluster <2 m/s, a gap, then a handful of still-"
    "residual outliers) -- the 80th percentile, sitting at that natural break, is "
    "used rather than a 95th percentile that a sample this size (n=38) cannot "
    "support robustly. Labelled ASSUMED, not MEASURED, given the small N and the "
    "residual association uncertainty stated above.",
    tunable_on="tune_split",
)
MIN_CLOSING_SPEED_MPS = _param(
    "MIN_CLOSING_SPEED_MPS",
    1.79,
    "ASSUMED",
    "Closing speed <= this returns TTC = inf rather than an astronomically large "
    "but meaningless finite value (G8). Distinct from MIN_TRUSTED_SPEED_MPS above — "
    "this gates the RADIAL-to-ego component, not raw speed magnitude. Measured "
    "2026-09-24 alongside MIN_TRUSTED_SPEED_MPS, with one further G13 fix: the full "
    "closing-speed formula ((ego_v - obj_v)*los/dist) is NOT pure jitter for a "
    "stationary object -- ego's own real motion relative to a fixed point is a "
    "genuine geometric signal (often several m/s), not noise, and ego's own "
    "velocity is precisely known (EGO_POSE_STD_M ~5cm), not itself jittery. The "
    "actual noise contribution is only the object's own estimated velocity "
    "(exactly 0 for a truly stationary object) projected onto the line-of-sight -- "
    "using the full formula instead gave a contaminated median of ~5 m/s dominated "
    "by real ego motion, not sensor noise. Same 80th-percentile-at-the-natural-"
    "break reasoning as MIN_TRUSTED_SPEED_MPS applies to the corrected sample.",
    tunable_on="tune_split",
)

# ── Forward-path corridor ─────────────────────────────────────────────────
CORRIDOR_HALF_WIDTH_M = _param(
    "CORRIDOR_HALF_WIDTH_M",
    1.2,
    "ASSUMED",
    "Half-width (m) of the ego vehicle's forward path corridor used for the "
    "forward-path relevance filter (step05).",
)
CORRIDOR_MAX_RANGE_M = _param(
    "CORRIDOR_MAX_RANGE_M",
    60,
    "ASSUMED",
    "Maximum forward range (m) considered for path relevance — beyond this, TTC "
    "is not meaningfully actionable at typical urban/highway speeds.",
)
CANDIDATE_EXTRA_MARGIN_M = _param(
    "CANDIDATE_EXTRA_MARGIN_M",
    1.5,
    "ASSUMED",
    "Extra margin (m) added to the corridor when deciding which detections compete "
    "as track candidates, so an object about to enter the corridor is not missed.",
)
ENTERING_HORIZON_S = _param(
    "ENTERING_HORIZON_S",
    1.5,
    "ASSUMED",
    "Time horizon (s) used to admit an object that is clearly about to enter the "
    "corridor even though it is not in it yet.",
)

# ── Debounce / action tiers ───────────────────────────────────────────────
DEBOUNCE_N = _param(
    "DEBOUNCE_N",
    3,
    "ASSUMED",
    "Number of consecutive real updates the danger condition must hold before an "
    "action tier fires (G16). Tune split only.",
    tunable_on="tune_split",
)
TTC_GRADUAL_S = _param(
    "TTC_GRADUAL_S",
    2.5,
    "ASSUMED",
    "TTC at or below this triggers the GRADUAL action tier. Tune split only.",
    tunable_on="tune_split",
)
TTC_AEB_S = _param(
    "TTC_AEB_S",
    1.2,
    "ASSUMED",
    "TTC at or below this triggers the AEB action tier. Tune split only.",
    tunable_on="tune_split",
)

# ── Distance definition (DEC-1) ───────────────────────────────────────────
DISTANCE_DEFINITION = _param(
    "DISTANCE_DEFINITION",
    "nearest_surface_to_ego",
    "ASSUMED",
    "DEC-1, confirmed by Siva at step05 Part A (option B, README_MASTER.md §6): "
    "a partially-seen vehicle's visible centroid is biased ~1-2m from the GT box "
    "centre; nearest-surface is what TTC physically needs and stays consistent "
    "across sensors. Used identically by tracker output and ground truth (G9).",
)

# ── Ground-truth observability thresholds ─────────────────────────────────
MIN_LIDAR_PTS = _param(
    "MIN_LIDAR_PTS",
    5,
    "ASSUMED",
    "A GT object with fewer than this many LiDAR points inside its box is treated "
    "as outside LiDAR's effective view, not a tracker failure (what-not-to-do.md §6).",
)
MIN_RADAR_PTS = _param(
    "MIN_RADAR_PTS",
    1,
    "ASSUMED",
    "Same purpose as MIN_LIDAR_PTS, for radar's num_radar_pts.",
)
MIN_VISIBILITY_LEVEL = _param(
    "MIN_VISIBILITY_LEVEL",
    2,
    "ASSUMED",
    "Minimum nuScenes visibility level (of 4: 0-40%, 40-60%, 60-80%, 80-100%) for a "
    "GT object to count as observable at all, camera-side.",
)
MIN_COUNT_FOR_RATE = _param(
    "MIN_COUNT_FOR_RATE",
    10,
    "ASSUMED",
    "Below this many events in a denominator, a reported rate is labelled "
    "'indicative only' rather than a solid statistic (what-not-to-do.md §7).",
)
MAX_ASSOC_DIST_M = _param(
    "MAX_ASSOC_DIST_M",
    2.0,
    "ASSUMED",
    "Distance threshold (m) for Hungarian-assigning a system track to a GT object "
    "when diagnosing which GT object a false brake was about (step09).",
)

# ── LiDAR adapter (step04a) ────────────────────────────────────────────────
LIDAR_MIN_RANGE_M = _param(
    "LIDAR_MIN_RANGE_M",
    3.0,
    "ASSUMED",
    "Points closer than this range (m) to the LiDAR are excluded from the ROI "
    "entirely -- excludes ego-vehicle self-returns (the sensor hitting its own "
    "roof/mount/roof-rack). Found necessary on real data at step04a: a keyframe's "
    "adapter output included a 9805-point cluster at x=0-1.4m, z=1.4-1.8m -- "
    "exactly the LiDAR's own mount height at point-blank range, not a real "
    "external object. 3.0m comfortably exceeds a typical passenger vehicle's own "
    "body extent from a roof-mounted LiDAR.",
)
LIDAR_ROI_SLACK_M = _param(
    "LIDAR_ROI_SLACK_M",
    5.0,
    "ASSUMED",
    "Extra margin (m) added beyond the candidate gate's own range/half-width for "
    "the LiDAR adapter's coarse ROI crop — compute-only, not a decision filter "
    "(step05's candidate gate is the real decision boundary).",
)
LIDAR_ROI_Z_MIN_M = _param(
    "LIDAR_ROI_Z_MIN_M",
    -3.0,
    "ASSUMED",
    "Ego-frame z lower bound (m) for the LiDAR adapter's coarse ROI crop — a "
    "generous band meant only to drop obviously-irrelevant points (sky returns, "
    "deep underpasses) before ground removal, not to shape the final detection.",
)
LIDAR_ROI_Z_MAX_M = _param(
    "LIDAR_ROI_Z_MAX_M",
    3.0,
    "ASSUMED",
    "Ego-frame z upper bound (m) for the LiDAR adapter's coarse ROI crop — see "
    "LIDAR_ROI_Z_MIN_M.",
)
LIDAR_GROUND_FIT_RANGE_M = _param(
    "LIDAR_GROUND_FIT_RANGE_M",
    20.0,
    "ASSUMED",
    "RANSAC fits the ground plane using only NEAR-FIELD points within this range "
    "(m) — avoids far-range points (more likely off-plane due to road slope, or "
    "already low point-density) contaminating the fit. The fitted plane is then "
    "applied to the whole ROI, not just this near-field subset (step04a §3 step 4).",
)
LIDAR_GROUND_FIT_Z_MAX_M = _param(
    "LIDAR_GROUND_FIT_Z_MAX_M",
    0.5,
    "ASSUMED",
    "RANSAC fits the ground plane using only LOW points below this ego-frame z "
    "(m) — well below typical vehicle roofs, generous enough for genuine ground "
    "bumps/curbs, meant to bias the fitting sample toward likely-ground points.",
)
GROUND_RANSAC_DIST_THRESH_M = _param(
    "GROUND_RANSAC_DIST_THRESH_M",
    0.2,
    "ASSUMED",
    "Max distance (m) from the fitted ground plane to count as a ground point. "
    "Ported from V1's own tuned RANSAC_DISTANCE_THRESHOLD, measured on this exact "
    "dataset (Step_2_1_LiDAR_Processing.ipynb) — a real starting point, not a "
    "fresh guess, though re-validated on this project's own real-data check "
    "(step04a) since the pipeline around it has changed substantially.",
    tunable_on="tune_split",
)
GROUND_RANSAC_N_POINTS = _param(
    "GROUND_RANSAC_N_POINTS",
    3,
    "ASSUMED",
    "Points sampled per RANSAC iteration (the minimum to define a plane) — fixed "
    "by geometry, not really a free parameter; matches V1's own value.",
)
GROUND_RANSAC_ITERS = _param(
    "GROUND_RANSAC_ITERS",
    100,
    "ASSUMED",
    "RANSAC iterations to search for the best-fit ground plane. Ported from V1's "
    "own tuned value on this exact dataset.",
)
LIDAR_CLUSTER_EPS_M = _param(
    "LIDAR_CLUSTER_EPS_M",
    0.7,
    "ASSUMED",
    "DBSCAN eps (m) for LiDAR obstacle clustering. Ported from V1's own tuned "
    "value on this exact dataset (Step_2_1_LiDAR_Processing.ipynb) — re-validated "
    "on this project's own real-data check (step04a) since this project reads "
    "native-rate sweeps, not the same point density V1 processed.",
    tunable_on="tune_split",
)
LIDAR_CLUSTER_MIN_SAMPLES = _param(
    "LIDAR_CLUSTER_MIN_SAMPLES",
    10,
    "ASSUMED",
    "DBSCAN min_samples for LiDAR obstacle clustering. Ported from V1's own tuned "
    "value; re-validated on this project's own real-data check (step04a) for the "
    "same reason as LIDAR_CLUSTER_EPS_M.",
    tunable_on="tune_split",
)
NEAREST_SURFACE_RADIUS_M = _param(
    "NEAREST_SURFACE_RADIUS_M",
    0.3,
    "ASSUMED",
    "DEC-1 nearest-surface reference point: a cluster's reference point is the "
    "mean of its points within this radius (m) of the single nearest point by "
    "range — not the bare minimum point alone (too noisy). Starting value "
    "proposed directly by step04a_lidar_adapter.md §3. Shared across sensors "
    "(no per-sensor prefix) — DEC-1 is explicitly a project-wide, cross-sensor "
    "rule (README_MASTER.md §6: 'stays consistent across sensors'), reused "
    "as-is by the radar adapter (step04b) rather than given its own value.",
)

# ── Radar adapter (step04b) ───────────────────────────────────────────────
RADAR_CLUSTER_EPS_M = _param(
    "RADAR_CLUSTER_EPS_M",
    1.5,
    "ASSUMED",
    "DBSCAN eps (m) for radar clustering, in xy only (z ignored -- unreliable, "
    "step04b §3). Ported from V1/V2's own tuned value on this exact dataset "
    "(V1 Step_3_2_Radar_Fusion.ipynb; V2 config.py RADAR_CLUSTER_EPS_M) -- "
    "2x LiDAR's eps (0.7m) since radar returns are sparser and spread wider "
    "per real object.",
    tunable_on="tune_split",
)
RADAR_CLUSTER_MIN_SAMPLES = _param(
    "RADAR_CLUSTER_MIN_SAMPLES",
    1,
    "ASSUMED",
    "DBSCAN min_samples for radar clustering. Ported from V1/V2's own tuned "
    "value: radar rarely returns more than 1-2 points per real object per "
    "scan, so requiring min_samples>1 (as LiDAR does, =10) would silently "
    "drop most real single-blip detections. NOTE: with min_samples=1, "
    "sklearn's DBSCAN never labels any point as noise (label=-1) -- every "
    "radar point that survives the devkit's own point filter becomes part "
    "of SOME cluster/detection, ghost returns included (step04b decision, "
    "2026-09-23: ghost-return suppression is the downstream pipeline's job, "
    "not this adapter's -- see docs/decisions.md).",
    tunable_on="tune_split",
)

# ── Radar Doppler ablation (G17, post-step11) ──────────────────────────────
RADAR_DOPPLER_ABLATION = _param(
    "RADAR_DOPPLER_ABLATION",
    False,
    "ASSUMED",
    "Whether radar's own Doppler velocity (vx_comp/vy_comp, already parsed "
    "in adapters/radar.py's aux dict) is fed into the tracker as a genuine "
    "second measurement channel (position AND velocity updated together, "
    "via ConstantVelocityKF.update()'s optional H=kalman.H4), instead of "
    "being ignored outside aux (the project's default since step04b, G17). "
    "FALSE for every official result already reported in this project "
    "(every adapter, step04a-step11's own frozen config) -- Doppler-ON is "
    "a separate, explicitly-approved ablation (Siva, 2026-10-08, prompted "
    "by the predecessor project's own measured finding that this was its "
    "single largest accuracy win), never silently the default. Tune split "
    "only; does not touch configs/frozen_config.json.",
)
RADAR_VELOCITY_NOISE_VAR_MPS2 = _param(
    "RADAR_VELOCITY_NOISE_VAR_MPS2",
    1.0,
    "ASSUMED",
    "Variance (m/s)^2 of radar's own Doppler velocity measurement noise, "
    "ego-frame, isotropic (no long/lat split -- Doppler is fundamentally a "
    "radial measurement along the sensor's own line-of-sight, not neatly "
    "long/lat like position R; an isotropic starting model is simpler and "
    "matches the predecessor project's own analogous parameter, arrived at "
    "independently, not ported). Only used when RADAR_DOPPLER_ABLATION=True.",
)

# ── Camera adapter (step04c) ────────────────────────────────────────────────
CAMERA_DETECTOR_MODEL = _param(
    "CAMERA_DETECTOR_MODEL",
    "yolov8n",
    "ASSUMED",
    "Ultralytics YOLO weight name -- ported from V1's own choice (V1's "
    "Step_2_3_YOLOv5.ipynb-family notebooks used Ultralytics YOLOv8n, CPU "
    "inference). Confirmed with Siva 2026-09-24: our own GPU (GeForce GT 710, "
    "2GB VRAM) can't meaningfully accelerate a heavier model, so a lightweight "
    "nano model matters for processing ~2,400 CAM_FRONT frames on CPU.",
)
CAMERA_DETECTOR_CONF_THRESH = _param(
    "CAMERA_DETECTOR_CONF_THRESH",
    0.35,
    "ASSUMED",
    "YOLOv8n confidence threshold. Ported from V1's own tuned value on this "
    "exact dataset/model.",
    tunable_on="tune_split",
)
CAMERA_DETECTOR_IOU_THRESH = _param(
    "CAMERA_DETECTOR_IOU_THRESH",
    0.45,
    "ASSUMED",
    "YOLOv8n NMS IoU threshold. Ported from V1's own tuned value.",
    tunable_on="tune_split",
)
CAMERA_RELEVANT_COCO_CLASSES = _param(
    "CAMERA_RELEVANT_COCO_CLASSES",
    {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus", 7: "truck"},
    "ASSUMED",
    "COCO class ids kept from the detector's raw output (person/bicycle/car/"
    "motorcycle/bus/truck) -- ported from V1's own filter, added there after "
    "an earlier version wastefully reported benches/potted plants/traffic "
    "lights as detections. Recorded on Detection.cls but never used for "
    "gating in any run (DEC-2).",
)
CAMERA_TRUNCATED_BOTTOM_MARGIN_PX = _param(
    "CAMERA_TRUNCATED_BOTTOM_MARGIN_PX",
    2,
    "ASSUMED",
    "A box whose bottom edge sits within this many pixels of the image's own "
    "bottom edge is dropped, not converted to a Detection -- its ground-"
    "contact point is unknown (truncated by the image boundary, step04c §3.1). "
    "Dropped boxes are counted in the report, never silently discarded.",
)
GROUND_Z_EGO = _param(
    "GROUND_Z_EGO",
    0.0,
    "ASSUMED",
    "Flat-ground-plane assumption (ego-frame z, metres) for camera ground-"
    "plane back-projection (DEC-6, confirmed 2026-09-24 -- no V1/V2 "
    "precedent exists for this method, built fresh). Diagnosed against real "
    "GT box bottoms on the tune split (step04c §3.4); a genuine limitation, "
    "not a hidden one -- fails on slopes/pitching, and back-projection range "
    "error grows roughly with range^2.",
    tunable_on="tune_split",
)
CAMERA_PIXEL_SIGMA_U_PX = _param(
    "CAMERA_PIXEL_SIGMA_U_PX",
    7.0,
    "ASSUMED",
    "Assumed pixel localization noise (horizontal, px) for a detector box's "
    "bottom-centre point -- step04c §3.3's own suggested starting range "
    "(5-10px), used by _camera_R's sigma_lat formula.",
)
CAMERA_PIXEL_SIGMA_V_PX = _param(
    "CAMERA_PIXEL_SIGMA_V_PX",
    7.0,
    "ASSUMED",
    "Assumed pixel localization noise (vertical, px) for a detector box's "
    "bottom edge -- same starting range as CAMERA_PIXEL_SIGMA_U_PX, used by "
    "_camera_R's sigma_long formula (which grows with range^2, since a fixed "
    "pixel error at the box's bottom edge maps to a growing ground-distance "
    "error as range increases and the ray becomes more grazing).",
)


# ── Required functions (step00 §4) ────────────────────────────────────────
def describe() -> str:
    """Markdown table of every registered Param — value/status/reason —
    used verbatim in report footnotes (step00 §4)."""
    lines = ["| Name | Value | Status | Tunable on | Reason |", "|---|---|---|---|---|"]
    for name in sorted(_REGISTRY):
        p = _REGISTRY[name]
        value_str = str(p.value).replace("|", "\\|")
        reason_str = p.reason.replace("|", "\\|").replace("\n", " ")
        lines.append(f"| `{name}` | {value_str} | {p.status} | {p.tunable_on} | {reason_str} |")
    return "\n".join(lines)


def _serializable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    return value


def freeze(path: str | Path) -> str:
    """Write every registered Param's value/status/reason/tunable_on as
    JSON to `path`, plus DATAROOT/NUSCENES_VERSION, and return the
    SHA-256 hash of that JSON (stable across calls unless a value
    actually changes — step00 §7)."""
    payload = {
        "DATAROOT": str(DATAROOT),
        "NUSCENES_VERSION": NUSCENES_VERSION,
        "params": {
            name: {
                "value": _serializable(_REGISTRY[name].value),
                "status": _REGISTRY[name].status,
                "reason": _REGISTRY[name].reason,
                "tunable_on": _REGISTRY[name].tunable_on,
            }
            for name in sorted(_REGISTRY)
        },
    }
    serialized = json.dumps(payload, sort_keys=True, indent=2)
    digest = hashlib.sha256(serialized.encode("utf-8")).hexdigest()
    payload["sha256"] = digest
    Path(path).write_text(json.dumps(payload, sort_keys=True, indent=2), encoding="utf-8")
    return digest


def current_config_hash() -> str:
    """The same whole-config SHA-256 freeze() computes and writes to disk,
    without writing anything -- step11 §5.2's eval-run-time check compares
    THIS against configs/frozen_config.json's own stored hash, so the
    comparison itself must never mutate that file."""
    payload = {
        "DATAROOT": str(DATAROOT),
        "NUSCENES_VERSION": NUSCENES_VERSION,
        "params": {
            name: {
                "value": _serializable(_REGISTRY[name].value),
                "status": _REGISTRY[name].status,
                "reason": _REGISTRY[name].reason,
                "tunable_on": _REGISTRY[name].tunable_on,
            }
            for name in sorted(_REGISTRY)
        },
    }
    serialized = json.dumps(payload, sort_keys=True, indent=2)
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()


# Params that are genuinely allowed to differ between the four stream runs
# (lidar/radar/camera/fused) -- everything else must be byte-identical
# (step11 §3's run-equivalence test). Prefixes cover each sensor's own
# adapter-parameter block and SENSOR_R_*; GROUND_Z_EGO is camera-specific
# but doesn't share the CAMERA_ prefix, so it's listed explicitly.
_PER_SENSOR_EXCLUDE_PREFIXES = ("LIDAR_", "RADAR_", "CAMERA_", "SENSOR_R_")
_PER_SENSOR_EXCLUDE_EXACT = ("GROUND_Z_EGO",)


def shared_config_hash() -> str:
    """SHA-256 over every registered Param EXCEPT the per-sensor-adapter
    blocks named above -- step11 §3: the four stream runs must share this
    hash exactly, proving they differ ONLY by which adapter/channels are
    enabled and each sensor's own unavoidable adapter parameters/R."""
    shared = {
        name: {
            "value": _serializable(_REGISTRY[name].value),
            "status": _REGISTRY[name].status,
            "tunable_on": _REGISTRY[name].tunable_on,
        }
        for name in sorted(_REGISTRY)
        if not name.startswith(_PER_SENSOR_EXCLUDE_PREFIXES) and name not in _PER_SENSOR_EXCLUDE_EXACT
    }
    serialized = json.dumps(shared, sort_keys=True, indent=2)
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()
