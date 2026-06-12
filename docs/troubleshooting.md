# Troubleshooting

## Robot not moving
```bash
ros2 lifecycle list /bt_navigator     # Nav2 active?
ros2 topic echo /map --once           # map received?
ros2 topic hz /scan                   # lidar publishing?
```

## YOLO not detecting
```bash
ros2 topic hz /camera/color/image_raw
# re-check model path in the node log
```

## AprilTags not detected
```bash
ros2 topic hz /camera/color/camera_info
ros2 topic hz /apriltag/image_rect
ros2 run tf2_ros tf2_echo map station_1_tag
```
If tag frames never appear, camera-info timestamps may be unsynced — that's what
`camera_info_sync_node` handles.

## Pick/drop timeouts
The planner waits 30 s each. With `pymoveit2` missing, the arm nodes log a warning
and publish success immediately (sim mode). Else:
```bash
ros2 node list | grep pickup
ros2 topic echo /nav/status
```

## Robot spins / stuck
Increase `inflation_radius` in `nav2_params.yaml`; confirm `/odom` publishes;
`ros2 run tf2_ros tf2_echo map base_footprint`.

## Odometry
```bash
ros2 topic hz /odom
ros2 run topic_tools relay /mirte_base_controller/odom /odom   # if missing
```

## Localisation
```bash
ros2 topic echo /amcl_pose --once    # hangs → AMCL not Active
ros2 topic echo /robot_pose --once
ros2 run tf2_tools view_frames       # need map → odom → base_footprint
```
`/amcl_pose` exists but subscribers get nothing → QoS mismatch
([localisation.md](localisation.md#qos-gotcha-fixed)).

## MIRTE platform
```bash
sudo systemctl stop mirte-shutdown
sudo systemctl restart mirte-ros
```
