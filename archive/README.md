# archive/

Superseded code kept for history. **Not built, not maintained.** Live code is in
`../src/mirte_sorting/`; see `../docs/roadmap.md`.

- `Detection_testing/` — older standalone detection package (`mirte_detectio`),
  superseded by `perception_node`/`pickup_node`/`dropoff_node`.
- `Detection_training_files/` — YOLO training run outputs (`train44/`, weights).
- `navigation_planning_demo/` — Nav2 demo; its nav2 config, behaviour tree and
  mock were integrated; its `sort_mission_planner.py` is superseded.
- `stubs/` — removed junk files (`CMakeList.txt`, `dropoff_node.ccp`) kept only
  for reference.

The former `localisation_ws/` package was integrated into `../src/mirte_sorting/`
and removed; its documentation is now `../docs/localisation.md`.
