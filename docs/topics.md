# Topic Contract

Authoritative inter-node interface. Names are defined in
`mirte_sorting/interfaces.py` — import the constants, never raw strings.

## Published by `mission_planner_node`

| Topic | Type | Values |
|-------|------|--------|
| `/nav/status` | `String` | `GOING_TO_SOURCE` · `AT_SOURCE` · `GOING_TO_DESTINATION` · `AT_DESTINATION` · `RETURNING_HOME` · `DONE` · `ABORTED` |

## Consumed by `mission_planner_node`

| Topic | Type | Publisher |
|-------|------|-----------|
| `/object_class` | `String` | perception / pickup |
| `/source_empty` | `Bool` | perception |
| `/pick_complete` / `/pick_failed` | `Bool` | pickup |
| `/drop_complete` / `/drop_failed` | `Bool` | dropoff |
| `/map` | `OccupancyGrid` | SLAM / Nav2 |

## Published by `perception_node`

`/arm_grasp_pose` (`PoseStamped`), `/object_class` (`String`), `/source_empty`
(`Bool`), `/detected_object/pos` (`PoseStamped`).

## Arm nodes

`pickup_node` subscribes `/nav/status`, `/arm_grasp_pose`; publishes
`/pick_complete`, `/pick_failed`, `/object_class`. `dropoff_node` subscribes
`/nav/status`; publishes `/drop_complete`, `/drop_failed`.

## QoS notes

- `TeamBridge` subscribes feedback `BEST_EFFORT/VOLATILE` (depth 10); publishes
  `/nav/status` `RELIABLE`.
- `/amcl_pose` is `TRANSIENT_LOCAL/RELIABLE` (latched) — match on subscribe (see
  [localisation.md](localisation.md#qos-gotcha-fixed)).

## Shape class names

`heart`(yellow/10) · `triangle`(red/11) · `hexagon`(green/12) · `l_shape`(blue/13)
· `default`(fallback). Lowercased on receipt.
