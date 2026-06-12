"""
navigation.launch.py
────────────────────
STEP 2: Autonomous sorting mission.

Brings up:
  - map_server + AMCL                      → localise against the saved map
  - localisation helpers                   → input/hybrid/output + apriltag corrector
  - Nav2 stack                             → controller/planner/behaviour/bt/…
  - robot_localization EKF                 → /odom + /imu → /odometry/filtered
  - odom relay + compat static TFs
  - perception + pickup/dropoff nodes
  - mission_planner_node                   → the 19-state FSM

Usage:
  ros2 launch mirte_sorting navigation.launch.py \\
    map:=/home/mirte/sorting_map.yaml \\
    semantic_map:=/home/mirte/semantic_map.yaml \\
    model_path:=/home/mirte/best.pt
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mirte_sorting")

    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    ekf_params  = PathJoinSubstitution([pkg, "config", "ekf.yaml"])
    stations    = PathJoinSubstitution([pkg, "config", "station_locations.yaml"])
    bt_xml      = PathJoinSubstitution([pkg, "behavior_trees", "nav_to_pose_simple.xml"])

    map_yaml      = LaunchConfiguration("map")
    semantic_map  = LaunchConfiguration("semantic_map")
    model_path    = LaunchConfiguration("model_path")
    cmd_vel_out   = LaunchConfiguration("cmd_vel_out")
    use_rviz      = LaunchConfiguration("rviz")

    lifecycle_nav = [
        "controller_server", "smoother_server", "planner_server",
        "behavior_server", "bt_navigator", "waypoint_follower", "velocity_smoother",
    ]

    nav2_stack = GroupAction([
        SetRemap(src="/cmd_vel_smoothed", dst=cmd_vel_out),
        Node(package="nav2_controller", executable="controller_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_smoother", executable="smoother_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_planner", executable="planner_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator",
             output="screen",
             parameters=[nav2_params,
                         {"default_nav_to_pose_bt_xml": bt_xml,
                          "default_nav_through_poses_bt_xml": bt_xml}]),
        Node(package="nav2_waypoint_follower", executable="waypoint_follower",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_velocity_smoother", executable="velocity_smoother",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": lifecycle_nav}]),
    ])

    localization = GroupAction([
        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen",
             parameters=[nav2_params, {"yaml_filename": map_yaml}]),
        Node(package="nav2_amcl", executable="amcl",
             name="amcl", output="screen", parameters=[nav2_params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["map_server", "amcl"]}]),

        # ── Integrated localisation helpers (formerly localisation_launch.py) ──
        Node(package="mirte_sorting", executable="localisation_input_node",
             name="localisation_input_node", output="screen"),
        Node(package="mirte_sorting", executable="hybrid_localiser",
             name="hybrid_localiser", output="screen",
             parameters=[{"use_rtabmap": False}]),
        Node(package="mirte_sorting", executable="localisation_output_node",
             name="localisation_output_node", output="screen"),
        Node(package="mirte_sorting", executable="apriltag_corrector",
             name="apriltag_corrector", output="screen",
             parameters=[{
                 "semantic_map_path": semantic_map,
                 "tag_frame_prefix": "tag36h11:",
                 "camera_frame": "camera_optical_frame",
                 "base_frame": "base_footprint",
                 "correction_cooldown": 3.0,
                 "max_detection_distance": 1.5,
             }]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Absolute path to the saved map .yaml"),
        DeclareLaunchArgument("semantic_map", default_value="",
                              description="Absolute path to semantic_map.yaml (apriltag corrector)"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="Path to the YOLO best.pt"),
        DeclareLaunchArgument("cmd_vel_out",
                              default_value="/mirte_base_controller/cmd_vel_unstamped"),
        DeclareLaunchArgument("rviz", default_value="true"),

        Node(package="robot_localization", executable="ekf_node",
             name="ekf_filter_node", output="screen", parameters=[ekf_params]),
        Node(package="topic_tools", executable="relay", name="odom_relay",
             arguments=["/mirte_base_controller/odom", "/odom"], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
             output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["0.0", "0.0", "0.18", "0", "0", "0", "base_footprint", "laser"],
             output="screen"),

        localization,
        nav2_stack,

        Node(package="mirte_sorting", executable="perception_node",
             name="perception_node", output="screen",
             parameters=[{"model_path": model_path}]),
        Node(package="mirte_sorting", executable="pickup_node",
             name="pickup_node", output="screen"),
        Node(package="mirte_sorting", executable="dropoff_node",
             name="dropoff_node", output="screen"),

        Node(condition=IfCondition(use_rviz), package="rviz2", executable="rviz2",
             name="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "Rviz_Settings_Mapping.rviz"])]),

        # Mission planner — delayed 20 s so Nav2 + AMCL are active first.
        TimerAction(period=20.0, actions=[
            Node(package="mirte_sorting", executable="mission_planner_node",
                 name="mission_planner_node", output="screen",
                 parameters=[stations]),
        ]),
    ])
