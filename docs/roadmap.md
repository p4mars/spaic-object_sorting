# Roadmap & Status

Tracks current-vs-target state.

## Implemented

- `mirte_sorting` package: nodes, FSM, interfaces, `nav2_client`.
- `mapping.launch.py` and `navigation.launch.py`.
- Config: apriltag, slam_params, ekf, rtabmap, semantic_map_config,
  station_locations, nav2_params (with amcl + map_server).
- Localisation integrated from `localisation_ws`: nodes, `apriltag_corrector`,
  `/amcl_pose` QoS fix, tuned AMCL params, helper nodes wired into navigation.

## Pending / future

- `nav2_client` recovery layer is minimal (timeout + cancel); a mecanum-strafe
  escape could be added.
- Hybrid fuser output covariance not propagated.
- AMCL/costmap frame consistency (`base_footprint` vs `base_link`) relies on an
  identity static TF — fine on flat floors.
- On-robot tuning of `nav2_params.yaml` (speeds, inflation, costmap layers).

## Structure

```
spaic-object_sorting/
├── README.md  docs/
├── src/mirte_sorting/
├── testing/    ← mock_mirte_base, rtab_room_map, move_arm
└── archive/    ← Detection_testing, Detection_training_files,
                  navigation_planning_demo, stubs (superseded)
```

The `localisation_ws` package was integrated into `src/mirte_sorting/` and
removed; its documentation became `docs/localisation.md`.
