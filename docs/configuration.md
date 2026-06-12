# Configuration

All tunables live in `src/mirte_sorting/config/*.yaml`.

| File | Used by | Note |
|------|---------|------|
| `station_locations.yaml` | mission_planner | **edit per arena** |
| `apriltag.yaml` | apriltag_node | stable |
| `slam_params.yaml` | slam_toolbox | stable |
| `ekf.yaml` | ekf_filter_node | stable |
| `rtabmap.yaml` | rtabmap (optional) | localisation-mode |
| `semantic_map_config.yaml` | semantic_map_node | stable |
| `nav2_params.yaml` | Nav2 + AMCL + map_server | map-based |

## station_locations.yaml

Map-frame approach poses (metres / radians; yaw 0=+X, 1.57=+Y). Fill in after
mapping:

```yaml
mission_planner_node:
  ros__parameters:
    map_frame: map
    source_station:      { x: 1.00, y: 0.50, yaw: 0.00 }
    destination_station: { x: 3.00, y: 0.50, yaw: 1.57 }
    start_pose:          { x: 0.00, y: 0.00, yaw: 0.00 }
    bins:
      heart:    { x: 3.20, y:  0.80, yaw: 1.57 }   # yellow
      triangle: { x: 3.20, y:  0.50, yaw: 1.57 }   # red
      hexagon:  { x: 3.20, y:  0.20, yaw: 1.57 }   # green
      l_shape:  { x: 3.20, y: -0.10, yaw: 1.57 }   # blue
      default:  { x: 3.20, y: -0.40, yaw: 1.57 }
```

Get coordinates from RViz **2D Nav Goal** + `ros2 topic echo /goal_pose --once`
(`yaw = 2*atan2(z, w)`).

## apriltag.yaml

`36h11`, 8 cm tags, PnP. Tag 1/2 = stations; 10–13 = drop boxes (heart/triangle/
hexagon/l_shape).

## slam_params.yaml

Ceres solver, online mapping. Key: `base_frame: base_footprint`, `scan_topic:
/scan`, `resolution: 0.05`, `min_laser_range: 0.20`, `minimum_travel_distance:
0.10`.

## ekf.yaml

Fuses `/odom` + `/imu/data` → `/odometry/filtered`; `world_frame: odom`,
`base_link_frame: base_footprint`, `publish_tf: false`. Do not change topics.

## rtabmap.yaml

RGB-D visual localisation, `Mem/IncrementalMemory: "false"`. Publishes
`/rtabmap/localization_pose`. Enable with `use_rtabmap:=true`.

## nav2_params.yaml

Nav2 servers (DWB controller, NavFn planner, costmaps) **plus** `amcl` and
`map_server` blocks. AMCL is tuned for the Pi 4 (`min/max_particles 1000/5000`,
`likelihood_field_prob`, `base_frame_id: base_footprint`). The `bt_navigator`
behaviour-tree path is injected by `navigation.launch.py` (not hard-coded in the
YAML). Speed limits: `max_vel_x 0.16`, `max_vel_theta 0.8`; `inflation_radius
0.18`; goal tolerances `0.10 m / 0.20 rad`.
