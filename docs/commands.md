# Commands

### Problems
To kill the shutdown procedure:
``` 
sudo systemctl stop mirte-shutdown
```
Restart ros2: --> VERY helpful
```
sudo systemctl restart mirte-ros
```

### MIRTE - Source ROS and the workspace
```
cd mirte_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```
### Computer - Source ROS and the workspace
```
cd ~/projects/spatial-ai/ws
export ROS_DOMAIN_ID=3
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Rebuild:
```
cd ~/mirte_ws
colcon build --symlink-install
source install/setup.bash
```
Specific:
```
colcon build --symlink-install --packages-select
```
Delete all built files:
```
rm -rf install/ build/ log/
```
----
# Mapping commands:

### Terminal 1: Launch mapping:
```
ros2 launch mirte_sorting mapping.launch.py
```

### Terminal 2: Move with keyboard:
```
ros2 launch mirte_teleop teleop_key.launch.py
```
or:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

### Terminal 3: Rviz2
```
rviz2
```
## Save the map:
1. Saving the map to the **maps** folder
- sorting_map.yaml
- sorting_map.pgm
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/linus/projects/spatial-ai/ws/src/spaic-object_sorting/maps/sorting_map'}}"
```

2. Save the semantic map:
```
ros2 service call /save_semantic_map std_srvs/srv/Trigger
```

3. Verify:
```
ls -lh /home/linus/projects/spatial-ai/ws/src/spaic-object_sorting/maps
```
---
## Run the navigation:
```
ros2 launch mirte_sorting navigation.launch.py map:=/home/linus/projects/spatial-ai/ws/src/spaic-object_sorting/maps/sorting_map.yaml
```
with rviz:
```
ros2 launch mirte_sorting navigation.launch.py \  
map:=/home/linus/projects/spatial-ai/ws/src/spaic-object_sorting/maps/sorting_map.yaml \  
rviz:=true
```

# Check moment
## cmd_vel
```
ros2 topic echo /mirte_base_controller/cmd_vel
```

Check the status:
```
ros2 topic info /mirte_base_controller/cmd_vel
```
- Gives the subscribers and publishers

## Camera
```
ros2 topic hz /camera/color/image_raw
```

```
ros2 topic hz /camera/color/camera_info
```

# Perception :
### With build:
```
cd ~/projects/spatial-ai/ws

source /opt/ros/humble/setup.bash
source ~/projects/spatial-ai/venvs/yolo_ros/bin/activate
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=3
colcon build --symlink-install --packages-select mirte_sorting
source install/setup.bash

# Important for the correct python invoroment
sed -i '1s|.*|#!/home/linus/projects/spatial-ai/venvs/yolo_ros/bin/python|' \
  install/mirte_sorting/lib/mirte_sorting/perception_node
```

### Setup
```
cd ~/projects/spatial-ai/ws

source /opt/ros/humble/setup.bash
source ~/projects/spatial-ai/venvs/yolo_ros/bin/activate
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID=3
source install/setup.bash

```
### Terminal 1: Run the node
```
ros2 run mirte_sorting perception_node --ros-args -p model_path:=/home/linus/projects/spatial-ai/ws/src/spaic-object_sorting/mirte_detectio/mirte_detectio/best.pt
```
### Terminal 2: Command 'AT_STATION'
```
ros2 topic pub /nav/status std_msgs/msg/String "data: 'AT_SOURCE'" --once
```



### Run Clara
```
ros2 launch bolt_screw_nav2_demo nav2_odom_only.launch.py \
  use_mock_robot:=false \
  odom_topic:=/mirte_base_controller/odom \
  cmd_vel_out:=/mirte_base_controller/cmd_vel
  
```

```
  ros2 launch bolt_screw_nav2_demo sort_mission.launch.py
```

---
### Run Navigation
```
ros2 launch localisation_pkg localisation_launch.py map:=/home/linus/projects/spatial-ai/localisation_ws/maps/your_map.yaml
```


