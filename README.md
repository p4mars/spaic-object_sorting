# bolt_screw_nav2_demo

This package handles the navigation and planning part of bolts and screws sorting project. It sends the robot to the right spots, pick-up area, correct bin and back home by using Nav2.

It is kept separate from the YOLO classifier, SLAM Toolbox and localisation (AMCL), so the navigation can be tested and demonstrated on its own. SLAM and AMCL need to be integrated later to provide the `map --> odom` transform and a global costmap. The YOLO classifier will replace the hardcoded `object_queue`.



## Package structure

```
bolt_screw_nav2_demo/
├── bolt_screw_nav2_demo/
│   ├── sort_mission_planner.py   # Main mission node, sends Nav2 goals for each shape
│   └── mock_mirte_base.py        # Fake robot for laptop testing (odom, TF, scan)
├── behavior_trees/
│   └── nav_to_pose_simple.xml    # Behaviour tree used by bt_navigator
├── config/
│   ├── nav2_odom_only.yaml       # Nav2 parameters tuned for the MIRTE (odom frame only)
│   ├── stations.yaml             # Poses for pick area, bins and home
│   └── nav2_demo.rviz            # RViz config showing robot, path and costmaps
├── launch/
│   ├── sort_mission.launch.py    # Main launch file, starts Nav2, RViz and mission planner
│   └── nav2_odom_only.launch.py  # Nav2 only, without the mission planner
├── package.xml
└── setup.py
```


## Running on the robot

```bash
ros2 launch bolt_screw_nav2_demo sort_mission.launch.py use_mock_robot:=false
```

This starts Nav2, RViz and the mission planner all at once. The mission planner waits 30 seconds for Nav2 to fully activate before sending the first goal.

To verify velocity commands are being sent:

```bash
ros2 topic echo /mirte_base_controller/cmd_vel
```



## Running on a laptop 
For testing without the real hardware:

```bash
ros2 launch bolt_screw_nav2_demo sort_mission.launch.py use_mock_robot:=true
```

This starts a fake robot (`mock_mirte_base`) that publishes odometry, TF and an empty laser scan so Nav2 has the correct input. 



## Things that has to be implemented to integrate with SLAM, localisation and YOLO

- **No SLAM or AMCL**: the global frame is `odom`, not `map`. The robot has no global map and cannot recover from positional drift. SLAM Toolbox and AMCL needs to be added later and change `global_frame` to `map` in `nav2_odom_only.yaml`.
- **No obstacle avoidance**: the obstacle layer is disabled since no lidar is used for navigation in this demo. Only the inflation layer is active.
- **Object queue** : shapes are hardcoded in `stations.yaml`. This is where the YOLO classifier output should be plugged in when integrated.
- **Robot radius** is set to `0.13 m`: update in `nav2_odom_only.yaml` if the real footprint differs.
- **Max speed** is `0.16 m/s` forward and `0.8 rad/s` rotation, kept low on purpose for indoor use.

