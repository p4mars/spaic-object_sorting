# Architecture

One autonomous mission split across four cooperating subsystems. A central
**mission planner** drives a state machine; everything else reacts to its
`/nav/status` broadcasts and reports back over the [topic contract](topics.md).

## Subsystem map

```
SENSORS:  RPLidar /scan   IMU /imu/data   RGB-D /camera/*   sonars
   ↓
   ├── MAPPING & LOCALISATION
   │    slam_toolbox → /map
   │    apriltag_node → tag TFs
   │    semantic_map_node → semantic_map.yaml
   │    hybrid_localiser → /robot_pose   (fuses EKF, AMCL, AprilTag)
   │
   ├── PERCEPTION & ARM
   │    perception_node (YOLO) → /arm_grasp_pose /object_class /source_empty
   │    pickup_node  → /pick_complete /pick_failed
   │    dropoff_node → /drop_complete /drop_failed
   │
   └── NAVIGATION & PLANNING
        Nav2 (BasicNavigator) → /mirte_base_controller/cmd_vel_unstamped
        mission_planner_node → 19-state FSM, publishes /nav/status
```

## Node communication

```
            mission_planner_node
  publishes /nav/status ; subscribes /pick_complete /drop_complete
            /object_class /source_empty /map
                 ↓ /nav/status
     ┌───────────┼────────────────┐
     ▼           ▼               ▼
 perception   pickup_node    dropoff_node
 on AT_SOURCE on AT_SOURCE   on AT_DESTINATION
 → grasp_pose + grasp_pose   → /drop_complete
   /object_class → /pick_complete
   /source_empty   /object_class(refined)
```

Topic names are defined once in `mirte_sorting/interfaces.py`; the
[topic contract](topics.md) is authoritative — never hard-code raw topic strings.

## TF tree

```
map → odom → base_footprint → base_link → camera_depth_optical_frame
                                         → gripper_link
map → station_1_tag, station_2_tag, dropbox_1_tag … dropbox_4_tag
```

- `map → odom` owned by AMCL / SLAM.
- `odom → base_footprint` from wheel odometry, improved by the EKF.
- `map → *_tag` from the AprilTag detector, averaged into the semantic map.

## Data flow for one sort cycle

```
GOING_TO_SOURCE → Nav2 drives to source → AT_SOURCE
→ perception detects shape → /arm_grasp_pose + /object_class
→ pickup executes → /pick_complete
→ planner reads class → navigates to matching bin → AT_DESTINATION
→ dropoff drops → /drop_complete
→ planner checks /source_empty → loop or RETURNING_HOME
```

Full state machine: [navigation.md](navigation.md#mission-state-machine).
