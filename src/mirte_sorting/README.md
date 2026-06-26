# mirte_sorting (ROS 2 package)

The single integrated package for the MIRTE sorting robot. Build from the
workspace root:

```bash
colcon build --packages-select mirte_sorting --symlink-install
source install/setup.bash
```

Layout: `mirte_sorting/` (nodes), `config/` (params), `launch/`
(`mapping.launch.py`, `navigation.launch.py`), `behavior_trees/`, `maps/`,
`models/` (`best.pt`), `rviz/`. Full docs: [`../../docs/`](../../docs/).
