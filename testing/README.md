# testing/

Standalone dev helpers — **not** part of the `mirte_sorting` colcon package, not
built by `colcon`.

- `mock_mirte_base.py` — fake base (publishes odom, consumes cmd_vel) for Nav2
  testing on a laptop without hardware.
- `rtab_room_map.py` — non-ROS OpenCV visual-keyframe prototype.
- `move_arm.py` — standalone arm-motion helper (not a registered node).
