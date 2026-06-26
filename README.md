# MIRTE Sorting Robot — AE4ASM527 Group 2

Autonomous bolt-and-screw sorting robot built on the **MIRTE Master** platform
(ROS 2 Humble). The robot maps an arena, localises itself, drives to a source
station, detects an object's shape with YOLO, picks it up, navigates to the
matching coloured bin, and drops it — repeating until the source is empty.

> **Pipeline:** shape detection → AprilTag localisation → SLAM mapping →
> Nav2 navigation with obstacle avoidance → arm pick-and-place.

## Documentation

| Doc | Covers |
|-----|--------|
| [Architecture](docs/architecture.md)       | System overview, node graph, data flow |
| [Perception](docs/perception.md)           | YOLO shape detection + arm pick/drop |
| [Localisation](docs/localisation.md)       | AMCL + EKF + AprilTag correction + hybrid fuser |
| [Mapping](docs/mapping.md)                 | SLAM Toolbox + semantic (AprilTag) map |
| [Navigation](docs/navigation.md)           | Nav2 + the mission-planner state machine |
| [Configuration](docs/configuration.md)     | Every `config/*.yaml` explained |
| [Operations](docs/operations.md)           | Run a mapping session and a sorting mission |
| [Topic contract](docs/topics.md)           | Inter-node topic interface |
| [Commands](docs/commands.md)               | Command cheatsheet |
| [Troubleshooting](docs/troubleshooting.md) | Common failures and fixes |
| [Roadmap & status](docs/roadmap.md)        | What works, what's missing |

## Quickstart

```bash
# 1. Build (workspace root)
colcon build --packages-select mirte_sorting --symlink-install
source install/setup.bash

# 2. Map the arena once (drive it around manually)
ros2 launch mirte_sorting mapping.launch.py
#    ... save the map + semantic map (see docs/operations.md)

# 3. Run the sorting mission
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  semantic_map:=/home/mirte/semantic_map.yaml \
  model_path:=/home/mirte/best.pt
```

## Repository layout

```
spaic-object_sorting/              ← colcon workspace root
├── README.md
├── docs/                          ← all detailed documentation
├── src/mirte_sorting/             ← the integrated ROS 2 package
├── testing/                       ← mocks + prototypes (no robot needed)
└── archive/                       ← superseded code kept for history
```

## Hardware

| Component | Part | Topic(s) |
|-----------|------|----------|
| Lidar  | RPLidar C1 (360°)         | `/scan` |
| IMU    | MPU-9250                  | `/imu/data` |
| Camera | RealSense / USB RGB-D     | `/camera/color/image_raw`, `/camera/depth/image_raw` |
| Sonars | HC-SR04 ×2                | `/mirte/distance/left\|right` |
| Base   | Mecanum, JGB37-520 motors | `/mirte_base_controller/cmd_vel_unstamped`, `/odom` |
| Arm    | MIRTE arm (MoveIt2/pymoveit2) | — |

Platform: MIRTE Master, Raspberry Pi 4 (ARM64), Ubuntu 22.04, ROS 2 Humble.
