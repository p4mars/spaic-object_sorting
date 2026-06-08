# mirte_sorting — AE4ASM527 Group 2

Unified ROS 2 package for the autonomous bolt and screw sorting robot built on the **MIRTE Master** platform.

Covers the full pipeline: **shape detection → AprilTag localisation → SLAM mapping → Nav2 navigation with obstacle avoidance → arm pick-and-place**.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Node Communication Diagram](#2-node-communication-diagram)
3. [Prerequisites](#3-prerequisites)
4. [Installation on MIRTE](#4-installation-on-mirte)
5. [Configuration](#5-configuration)
6. [Step 1 — Mapping Session](#6-step-1--mapping-session)
7. [Step 2 — Autonomous Sorting Mission](#7-step-2--autonomous-sorting-mission)
8. [Testing Without the Real Robot](#8-testing-without-the-real-robot)
9. [Topic Contract (Inter-node Interface)](#9-topic-contract-inter-node-interface)
10. [Troubleshooting](#10-troubleshooting)
11. [File Reference](#11-file-reference)

---

## 1. System Architecture

```
╔══════════════════════════════════════════════════════════════════════╗
║                     MIRTE SORTING ROBOT SYSTEM                       ║
╠══════════════════════════════════════════════════════════════════════╣
║                                                                      ║
║  ┌─────────────────────────────────────────────────────────────┐    ║
║  │                    SENSORS (Hardware)                        │    ║
║  │  RPLidar C1       IMU MPU-9250    RealSense / USB Camera     │    ║
║  │  /scan            /imu/data       /camera/color/image_raw    │    ║
║  │  (360° laser)     (orientation)   /camera/depth/image_raw    │    ║
║  │  HC-SR04 sonars                   /camera/color/camera_info  │    ║
║  │  /mirte/distance/left|right                                  │    ║
║  └────────────────────┬────────────────────────┬───────────────┘    ║
║                       │                        │                     ║
║  ┌────────────────────▼────────┐   ┌───────────▼───────────────┐   ║
║  │     MAPPING & LOCALISATION  │   │   PERCEPTION & DETECTION   │   ║
║  │                             │   │                            │   ║
║  │  slam_toolbox               │   │  camera_info_sync_node     │   ║
║  │  → publishes /map           │   │  → syncs image timestamps  │   ║
║  │                             │   │                            │   ║
║  │  apriltag_node              │   │  apriltag_node             │   ║
║  │  → TF: map→tag frames       │   │  → detects station tags    │   ║
║  │                             │   │                            │   ║
║  │  hybrid_localiser           │   │  perception_node (YOLO)    │   ║
║  │  → fuses AMCL + RTAB-Map   │   │  → detects shapes          │   ║
║  │  → publishes /robot_pose    │   │  → publishes /arm_grasp_pose│  ║
║  │                             │   │  → publishes /object_class │   ║
║  │  semantic_map_node          │   │  → publishes /source_empty │   ║
║  │  → averages tag positions   │   └──────────────┬────────────┘   ║
║  │  → saves semantic_map.yaml  │                  │                  ║
║  └────────────────────┬────────┘   ┌─────────────▼────────────┐    ║
║                       │            │      ARM CONTROL          │    ║
║                       │            │                           │    ║
║                       │            │  pickup_node (MoveIt2)    │    ║
║                       │            │  → executes grasp         │    ║
║                       │            │  → publishes /pick_complete│   ║
║                       │            │                           │    ║
║                       │            │  dropoff_node (MoveIt2)   │    ║
║                       │            │  → executes drop          │    ║
║                       │            │  → publishes /drop_complete│   ║
║                       │            └──────────────┬────────────┘   ║
║                       │                           │                  ║
║  ┌────────────────────▼───────────────────────────▼────────────┐   ║
║  │                  NAVIGATION & PLANNING                        │   ║
║  │                                                               │   ║
║  │  nav2_client (wraps BasicNavigator)                          │   ║
║  │  ├─ Layer 1: Pre-flight checks (lidar fresh? odom fresh?)    │   ║
║  │  ├─ Layer 2: In-flight monitoring (obstacle/tilt/timeout)    │   ║
║  │  └─ Layer 3: Recovery (mecanum strafe escape)                │   ║
║  │                                                               │   ║
║  │  mission_planner_node (19-state FSM)                         │   ║
║  │  INIT → SAVE_START → WAIT_NAV2 → NAV_SOURCE → DETECT        │   ║
║  │  → PICK → NAV_DEST → DETECT_BIN → DROP → CHECK → RETURN     │   ║
║  │                                                               │   ║
║  └───────────────────────────────────────────────────────────── ┘   ║
║                                                                      ║
╚══════════════════════════════════════════════════════════════════════╝
```

---

## 2. Node Communication Diagram

```
                ┌─────────────────────────────────────────────┐
                │            mission_planner_node              │
                │                                              │
                │  Publishes:  /nav/status (String)            │
                │  Subscribes: /pick_complete  /drop_complete  │
                │              /object_class   /source_empty   │
                │              /map            (map ready flag)│
                └──────────────┬──────────────────────────────┘
                               │  /nav/status
          ┌────────────────────┼───────────────────────┐
          ▼                    ▼                        ▼
┌──────────────────┐  ┌─────────────────┐   ┌────────────────────┐
│  perception_node │  │   pickup_node   │   │   dropoff_node     │
│                  │  │                 │   │                    │
│ Activates on:    │  │ Activates on:   │   │ Activates on:      │
│  "AT_SOURCE"     │  │  "AT_SOURCE"    │   │  "AT_DESTINATION"  │
│                  │  │  + grasp_pose   │   │                    │
│ Publishes:       │  │                 │   │ Publishes:         │
│  /arm_grasp_pose │→ │  (subscribes)   │   │  /drop_complete    │
│  /object_class   │  │                 │   │  /drop_failed      │
│  /source_empty   │  │ Publishes:      │   └────────────────────┘
└──────────────────┘  │  /pick_complete │
                      │  /pick_failed   │
                      │  /object_class  │ (refined close-up class)
                      └─────────────────┘

Nav2 Stack:
  /map ──────────────────────────────────────────→ mission_planner
  /scan ─────┐
  /odom ─────┼──→ nav2_client (internal monitoring)
  /imu/data ─┘       │
                      └──→ BasicNavigator → /mirte_base_controller/cmd_vel_unstamped

TF Tree:
  map → odom → base_footprint → base_link → camera_depth_optical_frame
                                           → gripper_link
  map → station_1_tag, station_2_tag, dropbox_1_tag … dropbox_4_tag
```

**Data flow for one sort cycle:**
```
  [Mission Planner]  →  "GOING_TO_SOURCE"
  [Nav2]             →  drives robot to source station
  [Mission Planner]  →  "AT_SOURCE"
  [Perception]       →  detects shape with YOLO
                     →  publishes /arm_grasp_pose + /object_class
  [Pickup Arm]       →  receives grasp_pose, executes pick
                     →  publishes /pick_complete (Bool True)
  [Mission Planner]  →  reads class ("heart"), navigates to yellow bin
                     →  "AT_DESTINATION"
  [Dropoff Arm]      →  drops into bin
                     →  publishes /drop_complete (Bool True)
  [Mission Planner]  →  checks /source_empty, loops or returns home
```

---

## 3. Prerequisites

### On the MIRTE robot (Ubuntu 22.04, ROS 2 Humble)

```bash
# Core ROS 2 packages
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-apriltag-ros \
  ros-humble-nav2-simple-commander \
  ros-humble-tf2-ros \
  ros-humble-cv-bridge \
  ros-humble-topic-tools \
  python3-opencv \
  python3-yaml

# Python packages
pip install ultralytics pymoveit2

# Optional: RTAB-Map (only needed for hybrid localiser RTAB path)
sudo apt install -y ros-humble-rtabmap-ros
```

> **Note on pymoveit2:** If the MIRTE arm uses a different control interface, set it up to publish `/pick_complete` and `/drop_complete` Bool topics directly. The pickup and dropoff nodes will still work — they gracefully degrade if pymoveit2 is missing and just simulate the arm motion.

### YOLO model

Place your trained `best.pt` model file on the robot. You can either:
- Copy it to `src/mirte_sorting/mirte_sorting/best.pt` (auto-detected), or
- Pass the full path with `model_path:=/home/mirte/best.pt` at launch time.

---

## 4. Installation on MIRTE

```bash
# On the robot, create or enter your ROS 2 workspace
cd ~/ros2_ws/src   # (or wherever your workspace is)

# Clone / copy the package
cp -r /path/to/AE4ASM527-Group-2---Object-Sorting/src/mirte_sorting .

# Build
cd ~/ros2_ws
colcon build --packages-select mirte_sorting --symlink-install

# Source
source install/setup.bash

# Add to .bashrc so you don't need to source every time
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 5. Configuration

### 5.1 Station locations — `config/station_locations.yaml`

**Edit this file with the real map coordinates after mapping.**

```yaml
mission_planner_node:
  ros__parameters:
    source_station:
      x:   1.00    # metres from map origin
      y:   0.50
      yaw: 0.00    # radians (0 = facing +X)

    destination_station:
      x:   3.00
      y:   0.50
      yaw: 1.57    # facing +Y (toward bins)

    bins:
      heart:    { x: 3.20, y:  0.80, yaw: 1.57 }  # yellow bin
      triangle: { x: 3.20, y:  0.50, yaw: 1.57 }  # red bin
      hexagon:  { x: 3.20, y:  0.20, yaw: 1.57 }  # green bin
      l_shape:  { x: 3.20, y: -0.10, yaw: 1.57 }  # blue bin
```

**How to get the coordinates:**
1. Build the map (Step 6 below)
2. Open RViz2 → click **2D Nav Goal** at each station entrance
3. In a terminal: `ros2 topic echo /goal_pose --once`
4. Copy `x`, `y`, and calculate `yaw` from `orientation.z` and `orientation.w`:
   `yaw = 2 * atan2(orientation.z, orientation.w)`

### 5.2 AprilTag IDs — `config/apriltag.yaml`

Default mapping:
| Tag ID | Role        | Physical location     |
|--------|-------------|----------------------|
| 1      | station_1   | Source station        |
| 2      | station_2   | Spare / second source |
| 10     | dropbox_1   | Yellow bin (heart)    |
| 11     | dropbox_2   | Red bin (triangle)    |
| 12     | dropbox_3   | Green bin (hexagon)   |
| 13     | dropbox_4   | Blue bin (l_shape)    |

Print tags from the `36h11` family. Physical size is 8 cm (set in `apriltag.yaml`).

### 5.3 SLAM parameters — `config/slam_params.yaml`

Key values for tuning:
```yaml
min_laser_range: 0.20    # Filters out robot's own body from lidar scans
resolution: 0.05          # Map cell size in metres (5 cm)
minimum_travel_distance: 0.10  # Add scan after moving 10 cm
```

### 5.4 Nav2 parameters — `config/nav2_params.yaml`

Key values for tuning:
```yaml
# Speed limits for MIRTE's JGB37-520 motors
max_vel_x:  0.20    # m/s forward (increase if robot too slow)
max_vel_y:  0.15    # m/s sideways (mecanum)
max_vel_theta: 0.8  # rad/s rotation

# Obstacle inflation — increase if robot clips walls
inflation_radius: 0.30   # metres

# Goal tolerance
xy_goal_tolerance:  0.15  # metres
yaw_goal_tolerance: 0.25  # radians (~14°)
```

---

## 6. Step 1 — Mapping Session

Do this once per arena layout. You will drive the robot around manually to build the map.

### 6.1 Launch the mapping stack

```bash
ros2 launch mirte_sorting mapping.launch.py
```

This starts: SLAM Toolbox, AprilTag detection, camera sync, semantic map node, hybrid localiser.

### 6.2 Drive the robot

Use a joystick or keyboard:
```bash
# Keyboard teleop (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

Drive slowly around the full arena, covering all areas at least once. Watch RViz for the map building up.

### 6.3 Save the map

Once the map looks complete:

```bash
# Save the occupancy grid map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/mirte/sorting_map'}}"
# Creates: /home/mirte/sorting_map.yaml + /home/mirte/sorting_map.pgm

# Save the semantic map (AprilTag positions)
ros2 service call /save_semantic_map std_srvs/srv/Trigger
# Creates: maps/semantic_map.yaml in the package
```

### 6.4 Verify the map

```bash
# Check which stations were detected
cat maps/semantic_map.yaml
```

You should see `station_1`, `station_2`, and the 4 `dropbox_*` entries with their x/y/yaw values.

### 6.5 Update station coordinates

Use the AprilTag positions from `semantic_map.yaml` as a reference, then measure the precise approach points in front of each station/bin and enter them in `config/station_locations.yaml`.

---

## 7. Step 2 — Autonomous Sorting Mission

### 7.1 Before launching

- [ ] `config/station_locations.yaml` has been filled with real coordinates
- [ ] YOLO model `best.pt` is available on the robot
- [ ] Map files exist at `/home/mirte/sorting_map.yaml` (and `.pgm`)
- [ ] All 6 AprilTags are printed and placed in the arena
- [ ] Objects (bolts/screws) are placed at Station 1

### 7.2 Launch

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  model_path:=/home/mirte/best.pt
```

**Optional arguments:**
```bash
# Disable RViz (faster on low-power robot)
rviz:=false

# Different YOLO model
model_path:=/home/mirte/my_model.pt
```

### 7.3 Watch the mission in RViz

Add these displays in RViz:
| Display type  | Topic                   | Purpose                     |
|---------------|-------------------------|-----------------------------|
| OccupancyGrid | `/map`                  | Arena map                   |
| LaserScan     | `/scan`                 | Live lidar readings          |
| Path          | `/plan`                 | Nav2 planned path            |
| Map           | `/local_costmap/costmap_raw` | Obstacle inflation       |
| TF            | *(check all frames)*    | Robot + tag positions        |
| PoseStamped   | `/arm_grasp_pose`       | Where arm will grasp         |

### 7.4 Monitor mission state

```bash
# Watch nav status changes
ros2 topic echo /nav/status

# Watch what shape was detected
ros2 topic echo /object_class

# Watch pick/drop signals
ros2 topic echo /pick_complete
ros2 topic echo /drop_complete
```

### 7.5 Emergency stop

```bash
# Kill velocity commands immediately
ros2 topic pub /mirte_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}" --once
```

---

## 8. Testing Without the Real Robot

### 8.1 Test individual nodes with mock publishers

**Test perception only (check if YOLO detects correctly):**
```bash
# Terminal 1 — start perception node
ros2 run mirte_sorting perception_node --ros-args -p model_path:=/home/mirte/best.pt

# Terminal 2 — activate it
ros2 topic pub /nav/status std_msgs/msg/String "data: 'AT_SOURCE'" --once

# Terminal 3 — watch detections
ros2 topic echo /object_class
ros2 topic echo /arm_grasp_pose
```

**Test mission planner logic (simulate arm + vision with mock publishers):**
```bash
# Terminal 1 — start planner (needs Nav2 running, use sim if available)
ros2 run mirte_sorting mission_planner_node \
  --ros-args --params-file config/station_locations.yaml

# Terminal 2 — simulate "pick succeeded" after AT_SOURCE
ros2 topic pub /pick_complete std_msgs/msg/Bool "data: true" --once

# Terminal 3 — simulate "class detected"
ros2 topic pub /object_class std_msgs/msg/String "data: 'heart'" --once

# Terminal 4 — simulate "drop succeeded" after AT_DESTINATION
ros2 topic pub /drop_complete std_msgs/msg/Bool "data: true" --once

# Terminal 5 — simulate "no more objects"
ros2 topic pub /source_empty std_msgs/msg/Bool "data: true" --once
```

**Test navigation only:**
```bash
# Start nav stack + planner with rviz
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml rviz:=true

# Send a fake pick complete to advance the state
ros2 topic pub /pick_complete std_msgs/msg/Bool "data: true" --once
ros2 topic pub /object_class std_msgs/msg/String "data: 'triangle'" --once
```

### 8.2 Test the state machine in isolation

```python
# Run from workspace root (no ROS needed):
python3 -c "
from src.mirte_sorting.mirte_sorting.task_state_machine import StateMachine, MissionState
sm = StateMachine()
sm.transition(MissionState.SAVE_START_POSE)
sm.transition(MissionState.WAIT_FOR_NAV2)
sm.transition(MissionState.NAVIGATE_TO_SOURCE)
print(sm.state)  # Should print: MissionState.NAVIGATE_TO_SOURCE
"
```

### 8.3 Smoke-test imports

```bash
# Check all modules import without errors
cd ~/ros2_ws
source install/setup.bash
python3 -c "
from mirte_sorting.interfaces import TeamBridge, STATUS_AT_SOURCE
from mirte_sorting.task_state_machine import StateMachine, MissionState
from mirte_sorting.station_manager import StationManager
from mirte_sorting.detection_functions import cluster_detections, fuse_clusters, pick_best
print('All imports OK')
"
```

---

## 9. Topic Contract (Inter-node Interface)

All topic names are defined in `mirte_sorting/interfaces.py`. **Never use raw strings for topic names in your code — import the constants.**

### Topics published by mission_planner_node

| Topic        | Type   | Values |
|--------------|--------|--------|
| `/nav/status` | String | `"GOING_TO_SOURCE"` `"AT_SOURCE"` `"GOING_TO_DESTINATION"` `"AT_DESTINATION"` `"RETURNING_HOME"` `"DONE"` `"ABORTED"` |

### Topics consumed by mission_planner_node

| Topic           | Type   | Who publishes        | When                         |
|-----------------|--------|----------------------|------------------------------|
| `/pick_complete` | Bool   | pickup_node          | After successful grasp        |
| `/pick_failed`   | Bool   | pickup_node          | After failed grasp            |
| `/drop_complete` | Bool   | dropoff_node         | After successful release      |
| `/drop_failed`   | Bool   | dropoff_node         | After failed release          |
| `/object_class`  | String | perception_node / pickup_node | Shape name after pick  |
| `/source_empty`  | Bool   | perception_node      | When search sweep exhausted   |
| `/map`           | OccupancyGrid | SLAM / Nav2 | Continuously                 |

### Shape class names (must match exactly)

| Class name | Bin colour | AprilTag |
|------------|------------|---------|
| `heart`    | Yellow     | 10      |
| `triangle` | Red        | 11      |
| `hexagon`  | Green      | 12      |
| `l_shape`  | Blue       | 13      |
| `default`  | *(fallback)* | —     |

> Names are normalised to lowercase in `TeamBridge._class_cb()`, so `"Heart"` and `"HEART"` are both accepted.

---

## 10. Troubleshooting

### Robot not moving

```bash
# Check Nav2 is active
ros2 lifecycle list /bt_navigator

# Check map is received
ros2 topic echo /map --once

# Check lidar is publishing
ros2 topic hz /scan
```

### YOLO not detecting

```bash
# Check camera is publishing
ros2 topic hz /camera/color/image_raw

# Check model path in node log
ros2 run mirte_sorting perception_node --ros-args -p model_path:=/home/mirte/best.pt
```

### AprilTags not detected

```bash
# Verify camera info is being published
ros2 topic hz /camera/color/camera_info

# Check synced topics
ros2 topic hz /apriltag/image_rect
ros2 topic hz /apriltag/camera_info

# Check TF frames are appearing
ros2 run tf2_ros tf2_echo map station_1_tag
```

### Pick/drop timeouts in mission planner

The planner waits 30 s for a pick and 30 s for a drop. If pymoveit2 is not installed, the arm nodes log a warning and **immediately publish success** (simulation mode) — so the mission still advances. If you see timeout errors, check:

```bash
# Is pickup_node running?
ros2 node list | grep pickup

# Is it receiving the status?
ros2 topic echo /nav/status
```

### Robot spins/gets stuck

Nav2's built-in behaviour tree handles most stuck situations automatically (spin in place, back up, replan). The `nav2_client` adds a second safety layer with sonar-based escape. If stuck persistently:
- Increase `inflation_radius` in `nav2_params.yaml` to give the robot more space
- Check that `/odom` is publishing (run the odom relay)
- Check TF: `ros2 run tf2_ros tf2_echo map base_footprint`

### Odometry problems

```bash
# Check the relay is running
ros2 topic hz /mirte_base_controller/odom
ros2 topic hz /odom

# If /odom is not publishing, start the relay manually:
ros2 run topic_tools relay /mirte_base_controller/odom /odom
```

---

## 11. File Reference

```
mirte_sorting/
│
├── mirte_sorting/                    Python package
│   ├── __init__.py
│   ├── detection_functions.py        YOLO result clustering + fusion helpers
│   ├── perception_node.py            YOLO detection → grasp pose + class
│   ├── pickup_node.py                MoveIt2 pick execution
│   ├── dropoff_node.py               MoveIt2 drop execution
│   ├── hybrid_localiser.py           Fuses AMCL + RTAB-Map → /robot_pose
│   ├── localisation_input_node.py    Sends initial pose to AMCL
│   ├── localisation_output_node.py   Logs AMCL pose
│   ├── camera_info_sync_node.py      Syncs camera_info timestamps for AprilTag
│   ├── semantic_map_node.py          Averages AprilTag TF frames → YAML
│   ├── interfaces.py                 ALL topic name constants + TeamBridge
│   ├── task_state_machine.py         19-state mission FSM + transition table
│   ├── station_manager.py            Loads bin/station poses from parameters
│   ├── nav2_client.py                3-layer safety wrapper around BasicNavigator
│   └── mission_planner_node.py       Central mission brain
│
├── config/
│   ├── station_locations.yaml        ← EDIT THIS before running (station/bin coords)
│   ├── apriltag.yaml                 AprilTag family + tag IDs + physical size
│   ├── slam_params.yaml              SLAM Toolbox mapping parameters
│   ├── nav2_params.yaml              Nav2 AMCL + DWB planner + costmap params
│   └── semantic_map_config.yaml      Tag ID → name mapping for semantic map
│
├── launch/
│   ├── mapping.launch.py             STEP 1: SLAM + AprilTag + semantic map
│   └── navigation.launch.py          STEP 2: Nav2 + full sorting mission
│
├── maps/
│   └── semantic_map.yaml             Auto-generated; do not edit manually
│
├── package.xml
├── setup.py
├── setup.cfg
└── README.md                         ← You are here
```

---

## Quick-start Cheatsheet

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select mirte_sorting --symlink-install
source install/setup.bash

# Mapping (first time, or new arena)
ros2 launch mirte_sorting mapping.launch.py
# ... drive robot around ...
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/mirte/sorting_map'}}"
ros2 service call /save_semantic_map std_srvs/srv/Trigger
# ... edit config/station_locations.yaml ...

# Mission run
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  model_path:=/home/mirte/best.pt

# Emergency stop
ros2 topic pub /mirte_base_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{}" --once
```
