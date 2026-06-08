"""
navigation.launch.py
────────────────────
STEP 2: Use this launch file on demo day with a pre-built map.

Starts:
  - Nav2 bringup (map_server + AMCL + planner + controller + bt_navigator)
  - Odom relay and TF compatibily nodes
  - AprilTag detection + camera sync
  - Hybrid localiser
  - Localisation input (sends initial pose to AMCL)
  - Perception node (YOLO shape detection)
  - Pickup arm node
  - Dropoff arm node
  - Mission planner (full autonomous sorting state machine)
  - RViz2 (optional)

BEFORE RUNNING:
  1. Edit config/station_locations.yaml with real map coordinates
  2. Place the YOLO model file (best.pt) in the package directory
     OR pass the full path via model_path argument

Usage:
  ros2 launch mirte_sorting navigation.launch.py map:=/home/mirte/sorting_map.yaml

  With a non-default YOLO model:
  ros2 launch mirte_sorting navigation.launch.py \\
    map:=/home/mirte/sorting_map.yaml \\
    model_path:=/home/mirte/best.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mirte_sorting")

    nav2_bringup_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])

    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    station_params = PathJoinSubstitution([pkg, "config", "station_locations.yaml"])
    apriltag_config = PathJoinSubstitution([pkg, "config", "apriltag.yaml"])
    semantic_config = PathJoinSubstitution([pkg, "config", "semantic_map_config.yaml"])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "rviz", "nav2_default_view.rviz"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz     = LaunchConfiguration("rviz")
    model_path   = LaunchConfiguration("model_path")
    use_odom_relay     = LaunchConfiguration("use_odom_relay")
    publish_compat_tfs = LaunchConfiguration("publish_compat_tfs")

    return LaunchDescription([

        # ── Arguments ──────────────────────────────────────────────────────
        DeclareLaunchArgument("map",
            description="Full path to saved map YAML. "
                        "Build first with: ros2 launch mirte_sorting mapping.launch.py"),
        DeclareLaunchArgument("use_sim_time",    default_value="false"),
        DeclareLaunchArgument("autostart",       default_value="true"),
        DeclareLaunchArgument("params_file",     default_value=nav2_params),
        DeclareLaunchArgument("rviz",            default_value="false"),
        DeclareLaunchArgument("model_path",      default_value="",
            description="Path to YOLO .pt model file (leave empty for auto-detect)"),
        DeclareLaunchArgument("use_odom_relay",     default_value="true"),
        DeclareLaunchArgument("publish_compat_tfs", default_value="true"),

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

        # ── Odom relay ─────────────────────────────────────────────────────
        Node(
            package="topic_tools",
            executable="relay",
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

        # ── Camera info sync ───────────────────────────────────────────────
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

        # ── AprilTag ───────────────────────────────────────────────────────
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[apriltag_config],
            remappings=[
                ("image_rect",  "/apriltag/image_rect"),
                ("camera_info", "/apriltag/camera_info"),
            ],
        ),

        # ── Semantic map (passive — records tag positions) ─────────────────
        Node(
            package="mirte_sorting",
            executable="semantic_map_node",
            name="semantic_map_node",
            output="screen",
            parameters=[semantic_config],
        ),

        # ── Hybrid localiser → /robot_pose ─────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="hybrid_localiser",
            name="hybrid_localiser",
            output="screen",
        ),

        # ── Initial pose publisher (sends pose to AMCL after map arrives) ──
        Node(
            package="mirte_sorting",
            executable="localisation_input_node",
            name="localisation_input_node",
            output="screen",
        ),

        # ── YOLO perception node ───────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[{"model_path": model_path}],
        ),

        # ── Pickup arm node ────────────────────────────────────────────────
        Node(
            package="mirte_sorting",
            executable="pickup_node",
            name="arm_node_pickup",
            output="screen",
            parameters=[{"model_path": model_path}],
        ),

        # ── Dropoff arm node ───────────────────────────────────────────────
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

        # ── RViz2 ─────────────────────────────────────────────────────────
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
