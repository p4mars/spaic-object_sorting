# Perception & Arm

`perception_node`, `pickup_node`, `dropoff_node` are gated by the mission
planner's `/nav/status` broadcasts — idle until the robot is in position.

## perception_node

YOLO shape detector. Activates on `/nav/status == "AT_SOURCE"`, runs the model on
the RGB stream, projects detections via the depth image, and publishes:

| Topic | Type | Meaning |
|-------|------|---------|
| `/arm_grasp_pose` | `PoseStamped` | Exact object position for the arm |
| `/object_class` | `String` | YOLO class name |
| `/source_empty` | `Bool` | `True` when the search sweep finds nothing |
| `/detected_object/pos` | `PoseStamped` | Nav waypoint (informational) |

Clustering/fusion helpers: `detection_functions.py`
(`cluster_detections`, `fuse_clusters`, `pick_best`).

### Shape → bin mapping

| Class | Bin colour | AprilTag |
|-------|-----------|----------|
| `heart` | Yellow | 10 |
| `triangle` | Red | 11 |
| `hexagon` | Green | 12 |
| `l_shape` | Blue | 13 |
| `default` | fallback | — |

Class names are lowercased in `TeamBridge._class_cb()`. They must otherwise match
the `bins:` keys in [`station_locations.yaml`](configuration.md#station_locationsyaml).

## YOLO model

Place the trained YOLO model at:

```bash
models/best.pt
```

or launch with an explicit path:

```bash
model_path:=/absolute/path/to/best.pt
```

Training files are stored in:

```bash
archive/Detection_training_files/
```

## YOLO Python environment

The YOLO nodes use a separate virtual environment because `ultralytics` requires `numpy>=1.23`, while ROS 2 Humble must stay on NumPy 1.x. Do **not** let `pip` install NumPy 2.x, since this can break ROS modules such as `cv_bridge`.

Install YOLO dependencies inside the `yolo_ros` venv with:

```bash
pip install "numpy>=1.23,<2" ultralytics
```

Before running the perception nodes, make sure user-site packages do not override the venv:

```bash
export PYTHONNOUSERSITE=1
```

After each `colcon build`, rewrite the installed executable shebang so `ros2 run` uses the venv Python:

```bash
sed -i '1s|.*|#!/absolute/path/to/venvs/yolo_ros/bin/python|' \
  install/mirte_sorting/lib/mirte_sorting/perception_node
```

If `pickup_node` is run standalone, apply the same fix:

```bash
sed -i '1s|.*|#!/absolute/path/to/venvs/yolo_ros/bin/python|' \
  install/mirte_sorting/lib/mirte_sorting/pickup_node
```

Use the full absolute venv path from `commands.md`.

If detection does not start, first check the logs for the loaded model path, NumPy errors, or `cv_bridge` import errors.


## pickup_node

Activates on `AT_SOURCE` once `/arm_grasp_pose` is available. Subscribes
`/nav/status`, `/arm_grasp_pose`; publishes `/pick_complete`, `/pick_failed`,
`/object_class` (refined close-up class).

## dropoff_node

Activates on `AT_DESTINATION`. Subscribes `/nav/status`; publishes
`/drop_complete`, `/drop_failed`.

## Arm backend (pymoveit2)

The arm nodes use `pymoveit2`/MoveIt2 but **degrade gracefully**: if `pymoveit2`
is missing they log a warning and immediately publish success (simulation mode),
so the mission still advances. To use a different controller, publish
`/pick_complete` and `/drop_complete` `Bool` directly. The planner waits 30 s for
each pick/drop.

## Testing without the robot

```bash
ros2 run mirte_sorting perception_node --ros-args -p model_path:=/abs/best.pt
ros2 topic pub /nav/status std_msgs/msg/String "data: 'AT_SOURCE'" --once
ros2 topic echo /object_class
ros2 topic echo /arm_grasp_pose
```
