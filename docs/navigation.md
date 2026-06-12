# Navigation & Mission Planning

Nav2 drives; `mission_planner_node` decides where to go via a state machine.

## Nav2

Wraps `BasicNavigator` (`nav2_simple_commander`). Consumes `/map`, `/scan`,
`/odom`, `/imu/data`; drives `/mirte_base_controller/cmd_vel_unstamped`.
`nav2_client.py` adds three safety layers on top of Nav2's behaviour tree:
pre-flight (lidar/odom fresh?), in-flight timeout monitoring, and recovery
(cancel + let the FSM recover). Tuning: [configuration.md](configuration.md#nav2_paramsyaml).

## Mission state machine

`mission_planner_node` runs a 19-state FSM (`task_state_machine.py`). It publishes
`/nav/status` and consumes arm/perception feedback via `TeamBridge`.

**Happy path (12):**
```
INIT → SAVE_START_POSE → WAIT_FOR_NAV2 → NAVIGATE_TO_SOURCE → DETECT_OBJECT
→ PICK_OBJECT → NAVIGATE_TO_DESTINATION → DETECT_BIN → DROP_OBJECT
→ CHECK_REMAINING → (loop | RETURN_TO_START) → TASK_COMPLETE
```
**Failure/recovery (7):** `NAVIGATION_FAILED`, `OBJECT_NOT_FOUND`, `PICK_FAILED`,
`BIN_NOT_FOUND`, `DROP_FAILED`, `RECOVERY`, `ABORT`.

| From | Success | Failure |
|------|---------|---------|
| NAVIGATE_TO_SOURCE | DETECT_OBJECT | NAVIGATION_FAILED |
| DETECT_OBJECT | PICK_OBJECT | OBJECT_NOT_FOUND |
| PICK_OBJECT | NAVIGATE_TO_DESTINATION | PICK_FAILED |
| NAVIGATE_TO_DESTINATION | DETECT_BIN | NAVIGATION_FAILED |
| DETECT_BIN | DROP_OBJECT | BIN_NOT_FOUND |
| DROP_OBJECT | CHECK_REMAINING | DROP_FAILED |
| CHECK_REMAINING | NAVIGATE_TO_SOURCE (more) | RETURN_TO_START (empty) |
| RETURN_TO_START | TASK_COMPLETE | NAVIGATION_FAILED |

`StateMachine` rejects illegal transitions and tracks per-state retries.

### `/nav/status` strings

`GOING_TO_SOURCE`, `AT_SOURCE`, `GOING_TO_DESTINATION`, `AT_DESTINATION`,
`RETURNING_HOME`, `DONE`, `ABORTED`.

### Timeouts

30 s for a pick result, 30 s for a drop, 15 s for an object class, 60 s for `/map`.

## Station & bin poses

`station_manager.py` loads approach poses from
[`station_locations.yaml`](configuration.md#station_locationsyaml).

## Testing the FSM (no ROS)

```python
from mirte_sorting.task_state_machine import StateMachine, MissionState
sm = StateMachine()
sm.transition(MissionState.SAVE_START_POSE)
sm.transition(MissionState.WAIT_FOR_NAV2)
sm.transition(MissionState.NAVIGATE_TO_SOURCE)
print(sm)   # StateMachine(state=NAVIGATE_TO_SOURCE)
```
