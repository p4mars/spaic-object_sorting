# Mapping

Two maps are built in one session:

1. **Occupancy grid** (`sorting_map.yaml` + `.pgm`) via **SLAM Toolbox**.
2. **Semantic map** (`maps/semantic_map.yaml`) — map-frame `(x, y, yaw)` of every
   AprilTag — via `semantic_map_node`.

Do this once per arena.

## Launch

```bash
ros2 launch mirte_sorting mapping.launch.py
# optional: use_rtabmap_mapping:=true   (also build the RTAB visual database)
```

Starts: EKF, SLAM Toolbox (async), optional RTAB-Map SLAM, odom relay, compat
static TFs, camera-info sync, AprilTag detector, semantic map node.

## Driving

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Drive slowly, cover the whole arena, and make sure the camera sees every tag.

## Saving

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/abs/path/maps/sorting_map'}}"
ros2 service call /save_semantic_map std_srvs/srv/Trigger
cat maps/semantic_map.yaml   # verify station_1/2 + dropbox_1..4
```

## Semantic map config

`config/semantic_map_config.yaml`:

```yaml
semantic_map_node:
  ros__parameters:
    map_frame: map
    sample_window: 15
    time_interval: 0.5
    tag_names: ["1:station_1","2:station_2","10:dropbox_1","11:dropbox_2","12:dropbox_3","13:dropbox_4"]
```

## After mapping

Measure the approach point ~0.3 m in front of each station/bin and enter it in
[`station_locations.yaml`](configuration.md#station_locationsyaml). SLAM tuning:
[configuration.md](configuration.md#slam_paramsyaml).
