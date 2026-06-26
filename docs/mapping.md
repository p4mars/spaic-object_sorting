# Mapping

Build the arena maps once, before running the sorting mission ([navigation.launch.py](https://claude.ai/chat/navigation.launch.py)). Two maps come out of a single session:

1. **Occupancy grid** (`sorting_map.yaml` + `sorting_map.pgm`) via **SLAM Toolbox** — the 2-D map Nav2 plans on.
2. **Semantic map** (`semantic_map.yaml`) — the map-frame `(x, y, yaw)` of every AprilTag (2 stations + 4 drop boxes) via `semantic_map_node`. Consumed later by `apriltag_corrector` during navigation.

## Before you start

`mapping.launch.py` starts **only** the mapping stack — it does _not_ bring up any hardware. 
The MIRTE base, lidar, IMU, and camera drivers **must** already be running and publishing `/mirte_base_controller/odom`, `/scan`, `/imu/data`, and `/camera/*`, plus the `odom → base_footprint` TF. If those topics/TFs aren't live, SLAM produces nothing.

Make sure your ROS 2 environment is sourced and `ROS_DOMAIN_ID` matches the robot — see [commands.md](https://claude.ai/chat/commands.md). 

This is a multi-terminal workflow: **launch**, **teleop**, **RViz**, then **save**.

## Terminal 1 — Launch the mapping stack

```bash
ros2 launch mirte_sorting mapping.launch.py
```

Starts: EKF, SLAM Toolbox (online async), odom relay (`/mirte_base_controller/odom → /odom`), compatibility static TFs, camera-info sync, the AprilTag detector and the semantic map node.

## Terminal 2 — Drive

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

(or `ros2 launch mirte_teleop teleop_key.launch.py`)

Drive **slowly** and cover the whole arena so the occupancy grid closes loops cleanly. For the semantic map, **pause on each AprilTag**: `semantic_map_node` needs `sample_window` (10) samples at `time_interval` 0.2 s — about **2-3 s of steady, continuous visibility per tag** — before it commits one. A quick glance won't register. Confirm each tag with the log line:

```
Tag '<label>' locked in from N samples.
```

You want all six: `station_1`, `station_2`, `dropbox_1` … `dropbox_4`.

## Terminal 3 — RViz (recommended)

```bash
rviz2 -d $(ros2 pkg prefix mirte_sorting)/share/mirte_sorting/rviz/Rviz_Settings_Mapping.rviz
```

Watch the grid build and the green semantic-map arrows/labels appear. Verify tag detection before saving:

```bash
ros2 topic hz /apriltag/image_rect          # detector receiving synced frames?
ros2 run tf2_ros tf2_echo map station_1_tag # tag TF resolving in the map frame?
```

## Terminal 4 — Save (with the stack still running)

Both saves must happen while the mapping nodes are alive.

```bash
# 1. Occupancy grid → <prefix>.yaml + <prefix>.pgm. Use an ABSOLUTE path.
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/abs/path/maps/sorting_map'}}"

# 2. Semantic map (the node logs the absolute path it wrote)
ros2 service call /save_semantic_map std_srvs/srv/Trigger
```

Note the occupancy-grid path you chose — it's the exact string you'll pass to `navigation.launch.py map:=...` in Phase 2.

**Verify the semantic map is populated** (not the empty `stations: {}` placeholder). It's written to the package _share_ directory, not your source tree, so cat the absolute path — easiest via the prefix:

```bash
cat $(ros2 pkg prefix mirte_sorting)/share/mirte_sorting/maps/semantic_map.yaml
```

You should see `station_1`/`station_2` under `stations:` and `dropbox_1`…`dropbox_4` under `drop_boxes:`, each with a `tag_id` and a `pose: {x, y, yaw}`. (With `--symlink-install` this path may resolve back to `src/.../maps/semantic_map.yaml`; either way, read the path the service logged.)

## Semantic map config

`config/semantic_map_config.yaml` (loaded by `mapping.launch.py`):

```yaml
semantic_map_node:
  ros__parameters:
    map_frame: map
    sample_window: 10     # samples required before a tag is locked in
    time_interval: 0.2    # seconds between samples → ~2-3 s steady view per tag
    tag_names: ["1:station_1","2:station_2","10:dropbox_1","11:dropbox_2","12:dropbox_3","13:dropbox_4"]
```

The tag IDs here must match `config/apriltag.yaml` (1,2 = stations; 10–13 = boxes).

## After mapping

Measure the approach point ~0.3 m in front of each station/bin and enter it in [`station_locations.yaml`](https://claude.ai/chat/configuration.md#station_locationsyaml) (RViz **2D Nav Goal** → `ros2 topic echo /goal_pose --once`, `yaw = 2*atan2(z, w)`). SLAM tuning: [configuration.md](https://claude.ai/chat/configuration.md#slam_paramsyaml). Then proceed to Phase 2: [navigation.launch.py](https://claude.ai/chat/navigation.launch.py).