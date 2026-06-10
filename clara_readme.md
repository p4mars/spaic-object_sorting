# Nav2 Checking Procedure — Clara's Guide

Everything you need to verify Nav2 is working correctly on the MIRTE sorting robot,  
from startup checks through sending test goals and reading the costmaps.

---

## What Nav2 needs before it will work

| Requirement | Topic / Service | How to check |
|---|---|---|
| LiDAR publishing | `/scan` | `ros2 topic hz /scan` → ~10 Hz |
| Odometry publishing | `/odom` | `ros2 topic hz /odom` → ~20 Hz |
| EKF fused odom | `/odometry/filtered` | `ros2 topic hz /odometry/filtered` → ~30 Hz |
| Map loaded | `/map` | `ros2 topic hz /map` → >0 Hz |
| AMCL receiving initial pose | `/amcl_pose` | `ros2 topic hz /amcl_pose` → ~2 Hz |
| TF chain complete | `map→odom→base_footprint→laser` | `ros2 run tf2_tools view_frames` |

Run all at once to see what is and isn't alive:

```bash
ros2 topic hz /scan /odom /odometry/filtered /map /amcl_pose /robot_pose
```

---

## Step-by-step startup check

### 1 — Launch navigation

```bash
ros2 launch mirte_sorting navigation.launch.py map:=/home/mirte/sorting_map.yaml
```

### 2 — Check lifecycle nodes activated

Nav2 uses lifecycle nodes. They must all reach `active` state before the robot can navigate.  
Watch for this in the terminal output:

```
[lifecycle_manager_navigation]: All nodes are active
[lifecycle_manager_localization]: All nodes are active
```

If either line never appears, run:

```bash
ros2 lifecycle list
```

Every node in the list below should show `active`. If one is stuck at `unconfigured`
or `inactive`, it crashed — check its individual log.

```
/amcl
/bt_navigator
/controller_server
/map_server
/planner_server
/behavior_server
/smoother_server
/velocity_smoother
/waypoint_follower
```

Force-activate a stuck node manually:

```bash
ros2 lifecycle set /amcl activate          # replace with whichever node is stuck
```

### 3 — Check the TF tree

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf in current directory
# Required chain: map → odom → base_footprint → laser
```

Quick terminal check:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
# Should print a transform — if it errors, AMCL hasn't localised yet
```

### 4 — Check AMCL has received an initial pose

Without an initial pose, AMCL won't publish and Nav2 won't plan.

```bash
ros2 topic echo /amcl_pose --once
```

If nothing prints within 5 seconds, send a manual initial pose:

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {w: 1.0}
      },
      covariance: [0.5,0,0,0,0,0, 0,0.5,0,0,0,0,
                   0,0,0,0,0,0,   0,0,0,0,0,0,
                   0,0,0,0,0,0,   0,0,0,0,0,0.1]
    }}"
```

Or use RViz → **2D Pose Estimate** button (easier and more accurate).

### 5 — Check costmaps are being built

```bash
ros2 topic hz /global_costmap/costmap    # should publish ~1 Hz
ros2 topic hz /local_costmap/costmap     # should publish ~2 Hz
```

In RViz add:
- `Map` on topic `/global_costmap/costmap` — should show the arena with inflated obstacles
- `Map` on topic `/local_costmap/costmap` — should show a small rolling window around the robot

If costmaps are empty or not publishing, `/scan` is probably not arriving at Nav2.

### 6 — Check velocity output is reaching the robot

Nav2 sends velocity commands to `/cmd_vel_smoothed`.  
A relay node forwards these to the MIRTE base controller.

```bash
ros2 topic hz /cmd_vel_smoothed                          # Nav2 output
ros2 topic hz /mirte_base_controller/cmd_vel_unstamped   # what the robot receives
```

Both must be active while Nav2 is navigating. If the first is active but the second
is not, the `cmd_vel_relay` node has died — restart the launch.

---

## Sending a test navigation goal

### Option A — command line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 1.0, y: 0.0, z: 0.0},
        orientation: {w: 1.0}
      }
    }
  }"
```

Watch the action feedback:

```bash
# In another terminal
ros2 topic echo /navigate_to_pose/_action/feedback
# Shows distance_remaining and estimated_time_remaining
```

### Option B — RViz

1. Open RViz: `ros2 run rviz2 rviz2`
2. Load the Nav2 default config or add these displays manually:
   - **Map** → `/map`
   - **Map** → `/global_costmap/costmap`
   - **Map** → `/local_costmap/costmap`
   - **Path** → `/plan`
   - **TF**
   - **LaserScan** → `/scan`
   - **Odometry** → `/odometry/filtered`
3. Click **Nav2 Goal** (or **2D Nav Goal**) and click a point on the map.

### Option C — check Nav2 is ready before sending a goal

```bash
ros2 service call /bt_navigator/is_active std_srvs/srv/Trigger
# Should return success: true
```

---

## Checking the behavior tree

The custom BT (`behavior_trees/nav_to_pose_simple.xml`) runs 4 recovery retries:  
clear local costmap → clear global costmap → spin → backup → wait.

Watch what the BT navigator is doing:

```bash
ros2 topic echo /bt_navigator/transition_event
```

If Nav2 keeps failing without entering recovery, the BT XML path may be wrong.
Verify it was loaded:

```bash
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
# Should return the path ending in behavior_trees/nav_to_pose_simple.xml
```

---

## Checking the planner

```bash
# Trigger a path plan manually (does NOT move the robot)
ros2 service call /compute_path_to_pose nav2_msgs/srv/ComputePathToPose \
  "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}},
    goal:  {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}},
    planner_id: 'GridBased'}"

# Watch the planned path appear in RViz
ros2 topic echo /plan --once | head -20
```

If the planner returns an empty path, the global costmap has no map loaded or the
goal is inside an obstacle.

---

## Checking the controller

```bash
# See what velocity the controller wants to send (while navigating)
ros2 topic echo /cmd_vel --once
```

If the controller is publishing but the robot isn't moving, the velocity relay is broken
(check `/mirte_base_controller/cmd_vel_unstamped`).

---

## Full Nav2 health checklist

Run through this list before every demo:

```bash
# 1. Sensors
ros2 topic hz /scan              # LiDAR: ~10 Hz
ros2 topic hz /odom              # Wheel odom: ~20 Hz
ros2 topic hz /imu/data          # IMU: ~50 Hz

# 2. Localisation
ros2 topic hz /odometry/filtered # EKF: ~30 Hz
ros2 topic hz /amcl_pose         # AMCL: ~2 Hz — zero means not localised
ros2 topic echo /localisation/source --once  # AMCL / RTAB+AMCL / ODOM

# 3. Nav2 lifecycle
ros2 lifecycle list | grep -v unconfigured   # all should show active

# 4. Costmaps
ros2 topic hz /global_costmap/costmap        # ~1 Hz
ros2 topic hz /local_costmap/costmap         # ~2 Hz

# 5. TF
ros2 run tf2_ros tf2_echo map base_footprint  # must return a valid transform

# 6. Velocity pipeline
ros2 topic hz /cmd_vel_smoothed              # active during navigation
ros2 topic hz /mirte_base_controller/cmd_vel_unstamped  # must match above
```

---

## Nav2 parameter quick-reference

Key values in `config/nav2_params.yaml`:

| Parameter | Value | Where |
|---|---|---|
| Max linear speed | 0.16 m/s | `velocity_smoother.max_velocity[0]` |
| Max lateral speed | 0.12 m/s | `velocity_smoother.max_velocity[1]` |
| Max rotation speed | 0.8 rad/s | `velocity_smoother.max_velocity[2]` |
| Goal tolerance XY | 0.15 m | `controller_server.general_goal_checker.xy_goal_tolerance` |
| Goal tolerance yaw | 0.25 rad | `controller_server.general_goal_checker.yaw_goal_tolerance` |
| Obstacle hard-stop (lidar) | 0.30 m | `nav2_client.py LIDAR_HARD_STOP` |
| Obstacle warning (lidar) | 0.55 m | `nav2_client.py LIDAR_WARN` |
| Inflation radius | 0.30 m | `local/global_costmap.inflation_layer.inflation_radius` |
| Nav timeout | 120 s | `nav2_client.py NAV_TIMEOUT_SEC` |
| BT recovery retries | 4 | `behavior_trees/nav_to_pose_simple.xml RecoveryNode` |

---

## Common Nav2 failures

| Symptom | Likely cause | Fix |
|---|---|---|
| `Waiting for service controller_server/get_state` forever | `controller_server` crashed on startup | Check nav2_params.yaml for YAML syntax errors |
| `Waiting for service map_server/get_state` forever | Map file path wrong or file missing | Check `map:=` argument points to a real `.yaml` file |
| `Waiting for service behavior_server/get_state` forever | behavior_server crashed | Verify `behavior_plugins` names match installed Nav2 version |
| Path planned but robot doesn't move | Velocity relay down | Check `ros2 topic hz /mirte_base_controller/cmd_vel_unstamped` |
| Robot moves then suddenly stops | Lidar hard-stop triggered (obstacle < 0.30 m) | Clear obstacle or increase `LIDAR_HARD_STOP` in `nav2_client.py` |
| Nav2 goal immediately cancelled | AMCL not localised — bad TF `map→odom` | Send `/initialpose` first |
| Costmap always empty | `/scan` not reaching Nav2 | Check scan topic name matches `nav2_params.yaml` (`/scan`) |
| `All nodes active` but robot spins in place | Goal inside inflated obstacle | Reduce `inflation_radius` or recheck station coordinates |
| Recovery triggers every goal | Planner tolerance too tight | Increase `planner_server.GridBased.tolerance` (currently 0.2 m) |
