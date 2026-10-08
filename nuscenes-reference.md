# nuScenes reference — for instant-ttc-fusion

No single official document covers all of this — it's assembled from the
schema page, the devkit README, and hard lessons from a prior project on
this same dataset. Verify anything version-sensitive against your
actual installed devkit; this is a reference to orient from, not a
substitute for reading the real source when something matters.

---

## 1. What nuScenes is

A large-scale autonomous-driving dataset (Motional, formerly nuTonomy),
1,000 20-second driving scenes across Boston and Singapore, captured with
a full AV sensor suite: 6 cameras (360° coverage), 1 LiDAR, 5 radars, IMU
+ GPS. Ground-truth 3D boxes are hand-annotated for every object, at 2Hz.

**Versions:**
- `v1.0-mini` — 10 scenes, for prototyping (what this project uses).
- `v1.0-trainval` — the full ~850-scene training/validation set.
- `v1.0-test` — held out, no public ground truth.

**Citation:** Caesar et al., "nuScenes: A multimodal dataset for
autonomous driving," CVPR 2020.

## 2. On-disk folder structure

```
dataroot/
├── samples/        # sensor data at 2Hz KEYFRAMES only (annotated)
│   ├── CAM_FRONT/, CAM_BACK/, ... (6 cameras)
│   ├── LIDAR_TOP/
│   └── RADAR_FRONT/, RADAR_BACK_LEFT/, ... (5 radars)
├── sweeps/         # sensor data at each sensor's NATIVE rate (unannotated)
│   └── (same channel folders as above)
├── maps/           # rasterized + vectorized semantic maps
└── v1.0-mini/      # JSON tables — the actual database
    ├── sample.json, sample_data.json, sample_annotation.json,
    │   ego_pose.json, calibrated_sensor.json, sensor.json,
    │   scene.json, log.json, instance.json, category.json,
    │   attribute.json, visibility.json, map.json
```

**Filenames encode a timestamp** (e.g. `..._CAM_FRONT_1533151603512404.jpg`)
— readable for debugging, but never parse it as the authoritative
timestamp. The authoritative value is the `timestamp` field inside that
file's `sample_data` JSON record.

## 3. Data schema — the tables and how they connect

Everything is a JSON table of records, each with a unique `token`.
Records reference each other by token — there is no SQL, just
dict-of-dicts joins (the devkit's `.get()` does this for you; see §6).

| Table | What it is | Key fields |
|---|---|---|
| `log` | One recording session (vehicle, date, location) | `logfile`, `vehicle`, `date_captured`, `location` |
| `scene` | One continuous 20s clip | `name`, `log_token`, `nbr_samples`, `first_sample_token`, `last_sample_token` |
| `sample` | One synchronized annotated instant (~2Hz) | `timestamp`, `scene_token`, `next`, `prev` |
| `sample_data` | ONE sensor's file at one instant | `sample_token`, `ego_pose_token`, `calibrated_sensor_token`, `filename`, `timestamp`, `is_key_frame`, `next`, `prev` |
| `ego_pose` | Vehicle's global position + rotation at a sensor's exact capture instant | `translation`, `rotation`, `timestamp` |
| `calibrated_sensor` | How one sensor is mounted on this vehicle | `sensor_token`, `translation`, `rotation`, `camera_intrinsic` |
| `sensor` | A sensor type/channel name | `channel` (e.g. `"CAM_FRONT"`), `modality` (`"camera"`/`"lidar"`/`"radar"`) |
| `instance` | ONE real object's whole life in a scene | `category_token`, `nbr_annotations`, `first_annotation_token`, `last_annotation_token` |
| `sample_annotation` | ONE ground-truth 3D box, at one sample | `sample_token`, `instance_token`, `translation`, `size`, `rotation`, `attribute_tokens`, `visibility_token`, `num_lidar_pts`, `num_radar_pts`, `next`, `prev` |
| `category` | Object class taxonomy | `name` (e.g. `"vehicle.car"`), `description` |
| `attribute` | A state flag on an annotation | `name` (e.g. `"vehicle.moving"`, `"pedestrian.standing"`) |
| `visibility` | How occluded an object is (across all 6 cameras) | `level` (0-40%, 40-60%, 60-80%, 80-100%) |
| `map` | Top-down semantic map (binary mask) | `filename`, `category`, `log_tokens` |

**The relationship that matters most for this project:**
`instance` = one real object's entire identity across the scene.
`sample_annotation` = one snapshot of that object at one instant, linked
via `instance_token`. Follow `sample_annotation.next`/`.prev` tokens to
walk one object's ground-truth trajectory sample-by-sample.

## 4. Ground truth — what you actually need for TTC evaluation

**Category taxonomy** (`category.name`) is hierarchical, dot-separated —
roughly 23 leaf classes under groups including:
- `vehicle.car`, `vehicle.truck`, `vehicle.bus.*`, `vehicle.motorcycle`,
  `vehicle.bicycle`, `vehicle.construction`, `vehicle.trailer`
- `human.pedestrian.adult`, `.child`, `.construction_worker`, `.police_officer`
- `movable_object.barrier`, `.trafficcone`, `.debris`, `.pushable_pullable`
- `static_object.bicycle_rack`
- `animal`

For a forward-path relevance / AEB-style filter, the collision-relevant
categories are almost certainly `vehicle.*` (excluding maybe
`vehicle.trailer` alone with no cab) and `human.pedestrian.*` —
`movable_object.*` and `static_object.*` are typically static clutter,
not things closing on the ego vehicle. **Verify the exact category list
against your installed `category.json`** — don't hardcode from memory
without checking, the exact leaf set has had minor changes across devkit
versions.

**Attribute** (`attribute.name`) tells you moving vs. stationary —
e.g. `vehicle.moving`, `vehicle.parked`, `vehicle.stopped`,
`pedestrian.moving`, `pedestrian.standing`, `pedestrian.sitting_lying_down`.
Useful for sanity-checking whether a GT object *should* produce a
meaningful closing speed at all.

**Ground-truth velocity — check `NuScenes.box_velocity()` before writing
your own.** The devkit ships a method that estimates an annotation's
velocity from its neighboring `sample_annotation` records via the
`prev`/`next` instance chain (roughly: position delta over the
`sample`-to-`sample` time gap, with a `max_time_diff` guard). This is
conceptually the same thing V1/V2 built by hand
(`compute_ground_truth_ttc()`'s finite-difference approach) — check the
devkit's own implementation before re-deriving it, since it already
handles some of the same edge cases (missing prev/next, gaps). Confirm
its exact method signature against your installed version; it has moved
between modules across devkit releases.

**`num_lidar_pts` / `num_radar_pts` on each `sample_annotation`** tell you
how many real sensor points landed inside that box — useful for judging
whether a GT instance was actually observable by a given sensor at that
instant (an object with `num_lidar_pts: 0` was outside LiDAR's effective
view, not a tracker failure).

## 5. Coordinate frames — the part that's caused real bugs before

Three nested frames, and every sensor reading starts in the first one:

```
sensor's own frame → ego vehicle's frame → global (world) frame
     (raw point)      (calibrated_sensor)      (ego_pose)
```

- `calibrated_sensor.translation`/`.rotation` = fixed, sensor-to-ego
  transform (how this sensor is mounted — doesn't change over time).
- `ego_pose.translation`/`.rotation` = time-varying, ego-to-global
  transform (where the car was at one specific instant).

**Every `sample_data` record has its OWN `calibrated_sensor_token` AND
its OWN `ego_pose_token`** — not shared across channels, even within one
`sample`. A LiDAR sweep and a camera frame nominally "in the same
sample" were captured at slightly different real instants, so the
vehicle moved between them, so they have different `ego_pose` records.
**Reusing one channel's `ego_pose` (or timestamp) for another channel's
points is a real, previously-confirmed bug** — always look up each
`sample_data` record's own tokens, never borrow a sibling channel's.

Rotations are quaternions (`pyquaternion.Quaternion`), not Euler angles
or rotation matrices directly — the devkit's `geometry_utils` module
(`transform_matrix`, etc.) handles the conversions; don't hand-roll
quaternion math.

## 6. `samples/` vs `sweeps/` — read this before writing any adapter

- `samples/` — only the 2Hz annotated keyframes. `is_key_frame: true`.
  This is what `sample.data[channel]` points to directly.
- `sweeps/` — every other reading, at each sensor's true native rate
  (LiDAR ~20Hz, cameras ~12Hz, radars ~13Hz). `is_key_frame: false`.
  Not directly attached to a `sample` — reach them by walking
  `sample_data.next`/`.prev` token chains from a keyframe.

**For this project specifically: default to `sweeps/`, not just
`samples/`.** "Freshest possible detection" is the whole point of an
instant-TTC system — throttling every sensor down to a shared 2Hz tick
throws away exactly the timing resolution this design exists to use.
(V1/V2 both started with `samples/`-only and this was later shown to
distort filter tuning in ways that wouldn't occur at true native rate —
don't repeat that deferral here.)

## 7. `nuscenes-devkit` — core API

```python
from nuscenes.nuscenes import NuScenes
nusc = NuScenes(version='v1.0-mini', dataroot='/path/to/dataset', verbose=True)
```

Loading this builds an in-memory token → record index for every table —
after this, every `.get()` call is a dict lookup, not a file re-read or
re-scan.

**Core lookups:**
```python
record = nusc.get('sample_data', token)      # any table, by its token
tokens = nusc.field2token('sample_annotation', 'instance_token', inst_tok)
```

**Common helper methods** (verify exact names/signatures against your
installed version — these are stable but have moved between modules
across releases):
- `nusc.get_sample_data(sample_data_token)` — returns the file path,
  the annotated boxes visible from it, and camera intrinsics if
  applicable, in one call.
- `nusc.box_velocity(sample_annotation_token, max_time_diff=1.5)` —
  GT velocity estimate, see §4.
- `nusc.list_categories()`, `nusc.list_attributes()` — print the real,
  current taxonomy from your installed dataset rather than trusting a
  hardcoded list.
- `NuScenesExplorer` (`nusc.explorer` or a separate import depending on
  version) — rendering/visualization helpers, useful for spot-checking
  a scene visually, not needed for the pipeline itself.

**Point cloud classes** (`nuscenes.utils.data_classes`):
- `LidarPointCloud.from_file(path)` — loads a `.pcd.bin` into an array.
- `RadarPointCloud.from_file(path)` — loads radar; **note the devkit's
  own class has built-in filtering/multi-sweep-aggregation options** —
  check whether its defaults already do some of what your own
  `radar_events()`-style per-scan clustering was built to handle, so you
  don't duplicate logic the devkit already offers.

## 8. Known gotchas — confirmed bugs from prior work on this exact dataset

- **Per-channel timestamp AND per-channel `ego_pose` are both
  independent** — confirmed twice as real bugs (radar reusing LIDAR_TOP's
  timestamp; the same risk existed for `ego_pose` and was separately
  verified). Always resolve both from the specific `sample_data` record
  you're working with.
- **`sample_annotation` ground truth exists only at 2Hz** — even if your
  detection pipeline runs on `sweeps/` at native rate, GT comparison
  points only exist at keyframe instants. If you need a track's state
  compared against GT, extrapolate the track's own state forward to the
  GT timestamp (position via last known velocity) — don't just grab the
  nearest track update.
- **Radar returns multiple points per real object per scan** — bumper,
  wheel well, panel reflections. Not pre-clustered by the dataset. Any
  per-scan radar processing needs to cluster before treating points as
  independent detections, or a filter update will treat several points
  on one object as repeated noisy measurements of a single point.
- **`nuScenes-mini` has 10 scenes, not 6** — a factual detail that's
  easy to misremember from early dissertation drafts.
- **The ego vehicle itself is not annotated as an object** — don't
  expect to find "self" in `sample_annotation`; ego state comes only
  from `ego_pose`.

## 9. Suggestions specific to this project

- Build the sensor→global transform as one shared, tested utility used
  by every adapter — never re-derive it per sensor.
- Default to `sweeps/` from the start (§6) — don't defer this the way
  prior work did.
- Filter to collision-relevant categories (§4) before anything reaches
  the tracker — most of nuScenes' 23 classes are irrelevant to an AEB
  decision.
- Check `box_velocity()` before writing a custom GT-velocity estimator —
  it may already do what you need.
- Treat `num_lidar_pts`/`num_radar_pts` as a cheap sanity check when
  a sensor "misses" a GT object — confirm the object was actually
  observable by that sensor at that instant before treating it as a
  detection failure.

## 10. Libraries

| Library | Purpose |
|---|---|
| `nuscenes-devkit` | The dataset SDK itself — table loading, lookups, eval code |
| `pyquaternion` | Quaternion math for all rotation handling (a devkit dependency) |
| `numpy` | Point cloud / transform math |
| `opencv-python` (`cv2`) | Image loading, used internally by devkit render methods |
| `matplotlib` | Devkit's built-in visualization/rendering |
| `Pillow` (`PIL`) | Image I/O, devkit dependency |
| `scipy` | Not a devkit dependency — your own KF/Hungarian-assignment code |

## 11. References

- Schema documentation: https://www.nuscenes.org/nuscenes#data-format
- Devkit repository: https://github.com/nutonomy/nuscenes-devkit
- Download / terms of use: https://www.nuscenes.org/download
- Paper: Caesar et al., "nuScenes: A multimodal dataset for autonomous
  driving," CVPR 2020 — https://arxiv.org/abs/1903.11027
