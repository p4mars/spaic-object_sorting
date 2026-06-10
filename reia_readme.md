# mirte_sorting — Reia's Guide

MIRTE Master autonomous object-sorting robot.  
Full pipeline: EKF + AMCL + RTAB-Map localisation → Nav2 navigation → YOLO detection → arm pick-and-place.

---

## Prerequisites

```bash
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-apriltag-ros \
  ros-humble-topic-tools \
  ros-humble-tf2-tools
```

---

## Install

```bash
cd ~/mirte_ws/src
# copy or clone the mirte_sorting package here
cd ~/mirte_ws
colcon build --packages-select mirte_sorting
source install/setup.bash
```

---

## Quick-start (two steps every demo day)

### Step 1 — Build the map

```bash
ros2 launch mirte_sorting mapping.launch.py
```

Drive the robot around the full arena until the map looks complete in RViz.  
Then save:

```bash
# Save 2-D occupancy map
ros2 service call /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap "{name: {data: '/home/mirte/sorting_map'}}"

# Save AprilTag positions
ros2 service call /save_semantic_map std_srvs/srv/Trigger
```

Optional — also build the RTAB-Map visual database (needed for `use_rtabmap:=true`):

```bash
ros2 launch mirte_sorting mapping.launch.py use_rtabmap_mapping:=true
```

---

### Step 2 — Run the sorting mission

Edit `config/station_locations.yaml` with the real map coordinates first, then:

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml
```

With RTAB-Map visual localisation on top of AMCL:

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  use_rtabmap:=true
```

With AprilTag auto-start (robot detects tag ID 0 at startup, no manual pose needed):

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  use_apriltag_init:=true \
  home_tag_id:=0 \
  home_tag_map_x:=0.0 \
  home_tag_map_y:=0.0 \
  home_tag_map_yaw:=0.0
```

---

## Testing localisation

### 1 — Check all sources are publishing

```bash
# Should all be active after the map loads (within ~5 s)
ros2 topic hz /odom                  # wheel encoder
ros2 topic hz /imu/data              # MPU-9250
ros2 topic hz /odometry/filtered     # EKF output
ros2 topic hz /amcl_pose             # AMCL particle filter
ros2 topic hz /robot_pose            # hybrid_localiser fused output
```

### 2 — Check which source is active

```bash
ros2 topic echo /localisation/source
# Should print: AMCL  (or RTAB+AMCL if use_rtabmap:=true, or ODOM as fallback)
```

### 3 — Check the fused pose value

```bash
ros2 topic echo /robot_pose --once
# pose.pose.position.x/y should match where the robot physically is on the map
```

### 4 — Check AMCL converged

```bash
ros2 topic echo /amcl_pose --once
# covariance[0] and [7] (x and y variance) should drop below ~0.1 once localised
# If they stay large, the particle cloud hasn't converged — send a better initial pose
```

### 5 — Send a manual initial pose (if AMCL hasn't converged)

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.5,0,0,0,0,0, 0,0.5,0,0,0,0,
                   0,0,0,0,0,0,   0,0,0,0,0,0,
                   0,0,0,0,0,0,   0,0,0,0,0,0.1]
    }}"
```

Or click **2D Pose Estimate** in RViz — easier.

### 6 — Check the TF tree is complete

```bash
ros2 run tf2_tools view_frames
# Expected chain: map → odom → base_footprint → laser
#                                             └→ camera_color_optical_frame → tag_N
```

### 7 — EKF-only test (no map needed)

To verify the EKF is fusing odom + IMU correctly before you have a map:

```bash
ros2 launch mirte_sorting mapping.launch.py   # SLAM not required to be active
ros2 topic echo /odometry/filtered
# Drive the robot — x/y/yaw should update smoothly
# Compare with raw /odom — filtered should be less jittery
```

---

## Testing mapping

### 1 — Confirm SLAM is building the map

```bash
# Check map is publishing
ros2 topic hz /map      # should be ~0.5 Hz (updates every 2 s)

# Open RViz to see it live
ros2 run rviz2 rviz2
# Add: Map (topic /map), TF, LaserScan (topic /scan)
```

### 2 — Check LiDAR is working

```bash
ros2 topic hz /scan                  # should be ~10 Hz
ros2 topic echo /scan --once | head  # check ranges are non-zero
```

### 3 — Check odom relay

```bash
ros2 topic hz /mirte_base_controller/odom   # source
ros2 topic hz /odom                          # relayed — must also be active
```

### 4 — Check AprilTag detection during mapping

```bash
ros2 topic echo /tf --once | grep tag   # tag_N frames should appear when in view
ros2 service call /get_semantic_map std_srvs/srv/Trigger  # if service exists
```

### 5 — Check EKF feeds SLAM Toolbox

```bash
ros2 topic hz /odometry/filtered    # must be active for SLAM to use smooth odom
```

### 6 — Verify the saved map

```bash
# After saving with the slam_toolbox service
ls -lh /home/mirte/sorting_map.yaml  # should exist and be non-empty
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/mirte/sorting_map.yaml
ros2 topic hz /map   # confirm map_server serves it successfully
```

---

## Localisation pipeline

```
/odom ──────────┐
                ├──► ekf_node ──► /odometry/filtered ──► AMCL (odom feedback)
/imu/data ──────┘                                    └──► hybrid_localiser fallback

/scan + /map ──► AMCL ──────────────────────────────► /amcl_pose ──┐
                                                                     ├──► hybrid_localiser ──► /robot_pose
/camera (RGB-D) ──► RTAB-Map (use_rtabmap:=true) ──► /rtabmap/localization_pose ──┘
```

**Fallback order** (hybrid_localiser):
1. RTAB-Map + AMCL weighted fusion (both active, lowest covariance wins more weight)
2. AMCL only (RTAB not enabled or stale)
3. EKF odometry (no map loaded — drifts over time, acceptable for short runs)

Watch `/localisation/source` to see which tier is active at any moment.

---

## Auto start position (AprilTag)

Place a known AprilTag (e.g. ID 0) at the robot's start position during mapping.  
The semantic map records its coordinates. On the next boot with `use_apriltag_init:=true`:

1. `localisation_input_node` waits for `/map` to load
2. Looks for `tag_0` in the TF tree (published by `apriltag_ros`)
3. Chains `base_footprint → tag_0` with the tag's known map pose to compute the robot's exact map pose
4. Publishes that as `/initialpose` — AMCL converges immediately
5. Falls back to configured `(initial_x, initial_y, initial_yaw)` if no tag seen within 5 s

Pass the tag's map pose (read from `maps/semantic_map.yaml` after mapping):

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  use_apriltag_init:=true \
  home_tag_id:=0 \
  home_tag_map_x:=<x from semantic_map.yaml> \
  home_tag_map_y:=<y from semantic_map.yaml> \
  home_tag_map_yaw:=<yaw from semantic_map.yaml>
```

---

## Topic contract

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/odom` | Odometry | odom relay | EKF, Nav2 |
| `/odometry/filtered` | Odometry | EKF | AMCL, hybrid_localiser, velocity_smoother |
| `/amcl_pose` | PoseWithCovarianceStamped | AMCL | hybrid_localiser |
| `/rtabmap/localization_pose` | PoseWithCovarianceStamped | RTAB-Map | hybrid_localiser |
| `/robot_pose` | PoseWithCovarianceStamped | hybrid_localiser | localisation_output_node |
| `/localisation/source` | String | hybrid_localiser | (monitoring) |
| `/initialpose` | PoseWithCovarianceStamped | localisation_input_node | AMCL |
| `/cmd_vel_smoothed` | Twist | velocity_smoother | cmd_vel_relay |
| `/mirte_base_controller/cmd_vel_unstamped` | Twist | cmd_vel_relay | MIRTE base |
| `/nav/status` | String | mission_planner | pickup_node, dropoff_node |
| `/object_class` | String | perception_node | mission_planner |
| `/pick_complete` | Bool | pickup_node | mission_planner |
| `/drop_complete` | Bool | dropoff_node | mission_planner |

---

## File reference

```
mirte_sorting/
├── config/
│   ├── nav2_params.yaml          Nav2 stack (AMCL, planner, controller, costmaps)
│   ├── ekf.yaml                  robot_localization EKF (odom + IMU fusion)
│   ├── rtabmap.yaml              RTAB-Map localization params
│   ├── slam_params.yaml          SLAM Toolbox mapping params
│   ├── station_locations.yaml    ← EDIT THIS after mapping
│   ├── apriltag.yaml             AprilTag tag family / size
│   └── semantic_map_config.yaml  Semantic map node settings
├── launch/
│   ├── mapping.launch.py         Step 1 — build map
│   └── navigation.launch.py      Step 2 — run mission
├── behavior_trees/
│   └── nav_to_pose_simple.xml    Nav2 BT with 4-retry recovery
├── mirte_sorting/
│   ├── hybrid_localiser.py       Fuses RTAB+AMCL+EKF → /robot_pose
│   ├── localisation_input_node.py Seeds AMCL initial pose (AprilTag or fixed)
│   ├── localisation_output_node.py Logs /robot_pose
│   ├── mission_planner_node.py   19-state FSM
│   ├── nav2_client.py            Nav2 wrapper with 3-layer safety
│   ├── station_manager.py        Loads station/bin poses from YAML
│   ├── perception_node.py        YOLO shape detection
│   ├── pickup_node.py            Arm pick
│   ├── dropoff_node.py           Arm drop
│   ├── semantic_map_node.py      Records AprilTag map positions
│   └── interfaces.py             Topic names + TeamBridge node
└── maps/
    └── semantic_map.yaml         Saved AprilTag positions (written by semantic_map_node)
```

---

## Common errors

| Error | Cause | Fix |
|---|---|---|
| `lifecycle_manager: Waiting for map_server/get_state` | No valid map file passed | Pass `map:=/home/mirte/sorting_map.yaml` |
| `[FAIL] RPLidar stale` | `/scan` not publishing | Check RPLidar driver is running |
| `[FAIL] Odometry stale` | odom relay not running | Check `use_odom_relay:=true` (default) |
| `/localisation/source` stuck on `NONE` | AMCL not converged + no EKF | Send `/initialpose` manually |
| `RCLError: invalid allocator` (sonar) | Duplicate topic type on same node | Already fixed — rebuild with `colcon build` |
| Bot doesn't move at all | `/cmd_vel_smoothed` not relayed | `cmd_vel_relay` node must be running (it is in navigation.launch.py) |
| AMCL covariance stays large | Wrong initial pose | Use RViz **2D Pose Estimate** or `use_apriltag_init:=true` |
