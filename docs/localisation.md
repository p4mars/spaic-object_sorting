# Localisation

> Adapted from the original `localisation_pkg` README (author: Reia Ramkumar).
> These nodes now live in `src/mirte_sorting/`.

The localisation stack answers *"where am I?"* by fusing several pose sources
into `/robot_pose`:

- **AMCL** — laser Monte-Carlo localisation on a pre-built map.
- **EKF** (`robot_localization`) — fuses wheel odometry + IMU into smooth odometry.
- **AprilTag corrector** — re-seeds AMCL when a known tag is spotted.
- **RTAB-Map** (optional) — visual localisation against a prior database.

```
/scan ─────────────────────────────────► AMCL ──────────────────────────►─┐
/odom ───► EKF ───► /odometry/filtered ─────────────────────────────────►─┤
/imu/data ──►                                                              │
semantic_map.yaml ──► AprilTagCorrector ──► /initialpose ──► AMCL ─────►  │
/camera/* ─────────────────────────────► RTAB-Map (optional) ──────────►─┤
                                                      HybridLocaliser
                                                             ▼
                                                      /robot_pose
```

## Nodes

| Node | Role |
|------|------|
| `localisation_input_node` | Sends initial pose to AMCL (fixed or AprilTag-derived). |
| `localisation_output_node` | Reads `/robot_pose`, exposes `(x, y, yaw)`. |
| `hybrid_localiser` | Weighted fusion of AMCL + RTAB + EKF → `/robot_pose`. |
| `apriltag_corrector` | Re-seeds AMCL from `semantic_map.yaml` tag positions. |

## AMCL

Maintains weighted particles, reweights/resamples on motion+scan, publishes
`/amcl_pose`. KLD-sampling grows/shrinks particle count with uncertainty. AMCL is
a **lifecycle node** driven by `nav2_lifecycle_manager`; it publishes nothing
until Active. Dependency chain: `map_server` Active and `/map` published → AMCL
receives `/initialpose` → particle cloud seeds → localising.

### QoS gotcha (fixed)

`nav2_amcl` publishes `/amcl_pose` `TRANSIENT_LOCAL + RELIABLE` (latched). A
default `VOLATILE` subscriber receives **nothing**. `hybrid_localiser` subscribes
with:

```python
QoSProfile(depth=1,
           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
           reliability=QoSReliabilityPolicy.RELIABLE)
```

## EKF

Fuses `/odom` + `/imu/data` → `/odometry/filtered` in the `odom` frame.
`world_frame: odom` keeps odometry continuous (AMCL supplies `odom → map`
separately, so AMCL jumps never pollute the filter). `publish_tf: false`.
See [configuration.md](configuration.md#ekfyaml).

## Hybrid fuser

Each source weighted by covariance:

```python
uncertainty = sqrt(abs(covariance[0]))
weight = max(0.1, 1.0 - uncertainty * 2)
```

Position = weighted average; orientation = most-confident source. Falls back to
raw EKF odometry (with a warning) when no map source is fresh. Output at 10 Hz.

## AprilTag corrector

Tags sit at known map-frame positions (`semantic_map.yaml`). On seeing a tag, the
corrector combines the measured camera→tag transform with the tag's known world
pose to recover the robot pose, publishing it to `/initialpose` with tight,
distance-dependent covariance — AMCL collapses its cloud around the correction.

```
robot_world = tag_world ∘ inv(cam→tag) ∘ inv(base_link→camera)
```

A ~3 s cooldown prevents spamming `/initialpose`; detections beyond ~1.5 m are
ignored. Parameters (`semantic_map_path`, `tag_frame_prefix`, `camera_frame`,
`base_frame`, `correction_cooldown`, `max_detection_distance`) are set in
`navigation.launch.py`. Confirm your tag prefix with
`ros2 topic echo /tf --once | grep child_frame_id`.

## Topics

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/map` | `OccupancyGrid` | map_server/SLAM | AMCL, input_node |
| `/initialpose` | `PoseWithCovarianceStamped` | input_node, apriltag_corrector | AMCL |
| `/odom` | `Odometry` | base controller (relayed) | EKF |
| `/imu/data` | `Imu` | IMU | EKF |
| `/odometry/filtered` | `Odometry` | EKF | hybrid_localiser, RTAB-Map |
| `/amcl_pose` | `PoseWithCovarianceStamped` | AMCL | hybrid_localiser |
| `/rtabmap/localization_pose` | `PoseWithCovarianceStamped` | RTAB-Map | hybrid_localiser |
| `/robot_pose` | `PoseWithCovarianceStamped` | hybrid_localiser | output_node, mission system |

## Known limitations

- Hybrid fuser output covariance is not propagated (left zeroed).
- AprilTag corrector uses 2-D SE(2) only (ignores camera tilt / tag height).
- EKF yaw accuracy depends on IMU calibration.
- `tag_frame_prefix` (default `tag36h11:`) must match your `apriltag_ros` output.

## Bugs fixed during development

1. **YAML corruption in `amcl_config.yaml`** — a stray `cd ~` pasted after
   `initial_pose_a: 0.0` made AMCL reject the config and crash. Fixed to a clean
   float.
2. **QoS mismatch on `/amcl_pose`** — volatile subscribers silently dropped all
   messages from AMCL's latched publisher. Fixed with the `TRANSIENT_LOCAL`
   profile above.
