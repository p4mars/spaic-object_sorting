# Operations

## Prerequisites (robot: Ubuntu 22.04, ROS 2 Humble)

```bash
sudo apt install -y \
  ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-apriltag-ros \
  ros-humble-nav2-simple-commander ros-humble-robot-localization \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-cv-bridge \
  ros-humble-topic-tools python3-opencv python3-yaml
pip install ultralytics pymoveit2
# optional: sudo apt install -y ros-humble-rtabmap-ros
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select mirte_sorting --symlink-install
source install/setup.bash
```

## Phase 1 — Mapping

See [mapping.md](mapping.md). Summary: launch `mapping.launch.py`, drive, save the
map + semantic map, edit `station_locations.yaml`.

## Phase 2 — Sorting mission

Checklist: coords filled · `best.pt` present · map files present · 6 tags placed ·
objects at source.

```bash
ros2 launch mirte_sorting navigation.launch.py \
  map:=/home/mirte/sorting_map.yaml \
  semantic_map:=/home/mirte/semantic_map.yaml \
  model_path:=/home/mirte/best.pt
# optional: rviz:=false
```

Monitor:
```bash
ros2 topic echo /nav/status
ros2 topic echo /object_class
ros2 topic echo /pick_complete
ros2 topic echo /drop_complete
```

Emergency stop:
```bash
ros2 topic pub /mirte_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{}" --once
```

## Testing without the robot

Mock-drive the planner with `ros2 topic pub` on `/object_class`,
`/pick_complete`, `/drop_complete`, `/source_empty`. A Nav2-without-hardware mock
(`mock_mirte_base.py`) is in `testing/`. Smoke-test imports:

```bash
python3 -c "
from mirte_sorting.interfaces import TeamBridge, STATUS_AT_SOURCE
from mirte_sorting.task_state_machine import StateMachine, MissionState
from mirte_sorting.station_manager import StationManager
from mirte_sorting.nav2_client import Nav2Client
print('All imports OK')"
```
