"""
navigation.launch.py
────────────────────
STEP 2: Use this launch file on demo day with a pre-built map.

Localisation stack (RTAB → AMCL → EKF fallback):
  - robot_localization EKF      — fuses /odom + /imu → /odometry/filtered
  - Nav2 AMCL                   — 2-D laser localisation from saved map
  - RTAB-Map (optional)         — visual localisation from .db database
  - hybrid_localiser            — fuses the above → /robot_pose
  - localisation_input_node     — sends initial pose to AMCL at startup
  - localisation_output_node    — logs /robot_pose for debugging

Navigation stack:
  - Nav2 bringup (map_server + AMCL + planner + controller + bt_navigator)
  - Custom BT: behavior_trees/nav_to_pose_simple.xml  (4-retry recovery)
  - Velocity relay: /cmd_vel_smoothed → /mirte_base_controller/cmd_vel_unstamped

Perception / arm:
  - AprilTag detection + camera sync
  - Semantic map node
  - YOLO perception node
  - Pickup / dropoff arm nodes
  - Mission planner

Usage:
  ros2 launch mirte_sorting navigation.launch.py map:=/home/mirte/sorting_map.yaml

  With RTAB-Map visual localisation:
  ros2 launch mirte_sorting navigation.launch.py \\
    map:=/home/mirte/sorting_map.yaml use_rtabmap:=true

  With AprilTag auto-start:
  ros2 launch mirte_sorting navigation.launch.py \\
    map:=/home/mirte/sorting_map.yaml \\
    use_apriltag_init:=true home_tag_id:=0 \\
    home_tag_map_x:=0.0 home_tag_map_y:=0.0 home_tag_map_yaw:=0.0
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mirte_sorting")

    nav2_bringup_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])

    nav2_params    = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    ekf_params     = PathJoinSubstitution([pkg, "config", "ekf.yaml"])
    rtabmap_params = PathJoinSubstitution([pkg, "config", "rtabmap.yaml"])
    station_params = PathJoinSubstitution([pkg, "config", "station_locations.yaml"])
    apriltag_cfg   = PathJoinSubstitution([pkg, "config", "apriltag.yaml"])
    semantic_cfg   = PathJoinSubstitution([pkg, "config", "semantic_map_config.yaml"])
    rviz_cfg       = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"])

    use_sim_time        = LaunchConfiguration("use_sim_time")
    use_rviz            = LaunchConfiguration("rviz")
    model_path          = LaunchConfiguration("model_path")
    use_odom_relay      = LaunchConfiguration("use_odom_relay")
    publish_compat_tfs  = LaunchConfiguration("publish_compat_tfs")
    use_rtabmap         = LaunchConfiguration("use_rtabmap")
    use_apriltag_init   = LaunchConfiguration("use_apriltag_init")

    return LaunchDescription([

        # ── Pre-launch cleanup ─────────────────────────────────────────────
        # Kill any stale SLAM Toolbox process — if mapping.launch.py was left
        # running it publishes map→odom and conflicts with AMCL here.
        ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -f async_slam_toolbox_node 2>/dev/null; '
                 'pkill -f slam_toolbox 2>/dev/null; '
                 'sleep 0.5; '
                 'rm -f /dev/shm/fastrtps_port*; '
                 'true'],
            output='screen',
            name='pre_launch_cleanup',
        ),

        # ── Arguments ──────────────────────────────────────────────────────
        DeclareLaunchArgument("map",
            description="Full path to saved map YAML. "
                        "Build first with: ros2 launch mirte_sorting mapping.launch.py"),
        DeclareLaunchArgument("use_sim_time",       default_value="false"),
        DeclareLaunchArgument("autostart",          default_value="true"),
        DeclareLaunchArgument("params_file",        default_value=nav2_params),
        DeclareLaunchArgument("rviz",               default_value="false"),
        DeclareLaunchArgument("model_path",         default_value="",
            description="Path to YOLO .pt model file (leave empty for auto-detect)"),
        DeclareLaunchArgument("use_odom_relay",     default_value="true"),
        DeclareLaunchArgument("publish_compat_tfs", default_value="true"),
        DeclareLaunchArgument("use_rtabmap",        default_value="false",
            description="Enable RTAB-Map visual localisation (needs /home/mirte/rtabmap.db)"),
        DeclareLaunchArgument("use_apriltag_init",  default_value="false",
            description="Auto-detect start pose from a home AprilTag"),
        DeclareLaunchArgument("home_tag_id",        default_value="0",
            description="AprilTag ID at start position (use_apriltag_init only)"),
        DeclareLaunchArgument("home_tag_map_x",     default_value="0.0"),
        DeclareLaunchArgument("home_tag_map_y",     default_value="0.0"),
        DeclareLaunchArgument("home_tag_map_yaw",   default_value="0.0"),

        # ── EKF: fuses /odom + /imu → /odometry/filtered ──────────────────
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        ),

        # ── Nav2 stack with saved map ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch),
            launch_arguments={
                "slam":            "False",
                "map":             LaunchConfiguration("map"),
                "use_sim_time":    use_sim_time,
                "autostart":       LaunchConfiguration("autostart"),
                "params_file":     LaunchConfiguration("params_file"),
                "use_composition": "False",
            }.items(),
        ),

        # ── Velocity relay: Nav2 → MIRTE base controller ───────────────────
        # Nav2 velocity_smoother publishes /cmd_vel_smoothed.
        # MIRTE base controller subscribes to its own topic.
        Node(
            package="topic_tools",
            executable="relay",
            name="cmd_vel_relay",
            arguments=["/cmd_vel_smoothed",
                       "/mirte_base_controller/cmd_vel_unstamped"],
            output="screen",
        ),

        # ── Odom relay ─────────────────────────────────────────────────────
        Node(
            package="topic_tools",
            executable="relay",
            name="odom_relay",
            arguments=["/mirte_base_controller/odom", "/odom"],
            output="screen",
            condition=IfCondition(use_odom_relay),
        ),

        # ── Compatibility TFs ──────────────────────────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            output="screen",
            condition=IfCondition(publish_compat_tfs),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_frame"],
            output="screen",
            condition=IfCondition(publish_compat_tfs),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.0", "0.0", "0.18", "0", "0", "0", "base_footprint", "laser"],
            output="screen",
            condition=IfCondition(publish_compat_tfs),
        ),

        # ── RTAB-Map visual localisation (optional) ────────────────────────
        Node(
            condition=IfCondition(use_rtabmap),
            package="rtabmap_ros",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[rtabmap_params],
            remappings=[
                ("rgb/image",       "/camera/color/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("depth/image",     "/camera/depth/image_raw"),
                ("odom",            "/odometry/filtered"),
            ],
        ),

        # ── Camera info sync → AprilTag ────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="camera_info_sync_node",
            name="camera_info_sync_node",
            output="screen",
            parameters=[{
                "image_topic":              "/camera/color/image_raw",
                "camera_info_topic":        "/camera/color/camera_info",
                "synced_image_topic":       "/apriltag/image_rect",
                "synced_camera_info_topic": "/apriltag/camera_info",
                "use_image_frame_id":       True,
            }],
        ),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[apriltag_cfg],
            remappings=[
                ("image_rect",  "/apriltag/image_rect"),
                ("camera_info", "/apriltag/camera_info"),
            ],
        ),

        # ── Semantic map ───────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="semantic_map_node",
            name="semantic_map_node",
            output="screen",
            parameters=[semantic_cfg],
        ),

        # ── Hybrid localiser → /robot_pose ─────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="hybrid_localiser",
            name="hybrid_localiser",
            output="screen",
            parameters=[{"use_rtabmap": use_rtabmap}],
        ),

        # ── Initial pose (AMCL seed) ───────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="localisation_input_node",
            name="localisation_input_node",
            output="screen",
            parameters=[{
                "use_apriltag_init": use_apriltag_init,
                "home_tag_id":       LaunchConfiguration("home_tag_id"),
                "home_tag_map_x":    LaunchConfiguration("home_tag_map_x"),
                "home_tag_map_y":    LaunchConfiguration("home_tag_map_y"),
                "home_tag_map_yaw":  LaunchConfiguration("home_tag_map_yaw"),
            }],
        ),

        # ── Localisation output (logs /robot_pose) ─────────────────────────
        Node(
            package="mirte_sorting",
            executable="localisation_output_node",
            name="localisation_output_node",
            output="screen",
        ),

        # ── YOLO perception ────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[{"model_path": model_path}],
        ),

        # ── Pickup arm ─────────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="pickup_node",
            name="arm_node_pickup",
            output="screen",
            parameters=[{"model_path": model_path}],
        ),

        # ── Dropoff arm ────────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="dropoff_node",
            name="arm_node_dropoff",
            output="screen",
        ),

        # ── Mission planner ────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="mission_planner_node",
            name="mission_planner_node",
            output="screen",
            parameters=[station_params],
        ),

        # ── RViz2 (optional) ───────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_cfg],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
