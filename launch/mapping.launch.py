"""
mapping.launch.py
─────────────────
STEP 1: Use this launch file to build the arena map before demo day.

Starts:
  - SLAM Toolbox (online async) → builds /map in real time
  - AprilTag detection → identifies station/bin tags
  - CameraInfoSync → ensures AprilTag receives correct camera info
  - SemanticMapNode → records tag positions; call save_semantic_map service when done
  - Odom relay: /mirte_base_controller/odom → /odom
  - Static TFs: base_link ↔ base_footprint and base_link ↔ base_frame
  - Hybrid localiser: fuses AMCL + RTAB-Map → /robot_pose

Drive the robot around the arena, then:
  ros2 service call /save_semantic_map std_srvs/srv/Trigger
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/mirte/sorting_map'}}"

Usage:
  ros2 launch mirte_sorting mapping.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mirte_sorting")

    slam_params = PathJoinSubstitution([pkg, "config", "slam_params.yaml"])
    apriltag_config = PathJoinSubstitution([pkg, "config", "apriltag.yaml"])
    semantic_config = PathJoinSubstitution([pkg, "config", "semantic_map_config.yaml"])

    use_odom_relay     = LaunchConfiguration("use_odom_relay")
    publish_compat_tfs = LaunchConfiguration("publish_compat_tfs")
    use_sim_time       = LaunchConfiguration("use_sim_time")

    return LaunchDescription([

        DeclareLaunchArgument("use_odom_relay",
            default_value="true",
            description="Relay /mirte_base_controller/odom → /odom"),
        DeclareLaunchArgument("publish_compat_tfs",
            default_value="true",
            description="Publish base_link ↔ base_footprint static TF"),
        DeclareLaunchArgument("use_sim_time",
            default_value="false"),

        # ── SLAM Toolbox ──────────────────────────────────────────────────
        # Builds the occupancy grid map from /scan + /odom
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),

        # ── Odom relay ────────────────────────────────────────────────────
        # MIRTE publishes odometry on a long topic; relay to standard /odom
        Node(
            package="topic_tools",
            executable="relay",
            arguments=["/mirte_base_controller/odom", "/odom"],
            output="screen",
            condition=IfCondition(use_odom_relay),
        ),

        # ── Compatibility static TFs ──────────────────────────────────────
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

        # ── Camera info sync → AprilTag requires matched timestamps ────────
        Node(
            package="mirte_sorting",
            executable="camera_info_sync_node",
            name="camera_info_sync_node",
            output="screen",
            parameters=[{
                "image_topic":             "/camera/color/image_raw",
                "camera_info_topic":       "/camera/color/camera_info",
                "synced_image_topic":      "/apriltag/image_rect",
                "synced_camera_info_topic": "/apriltag/camera_info",
                "use_image_frame_id":      True,
            }],
        ),

        # ── AprilTag detection ────────────────────────────────────────────
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[apriltag_config],
            remappings=[
                ("image_rect",   "/apriltag/image_rect"),
                ("camera_info",  "/apriltag/camera_info"),
            ],
        ),

        # ── Semantic map (averages AprilTag TF positions) ─────────────────
        Node(
            package="mirte_sorting",
            executable="semantic_map_node",
            name="semantic_map_node",
            output="screen",
            parameters=[semantic_config],
        ),

        # ── Hybrid localiser: fuses AMCL + RTAB-Map → /robot_pose ─────────
        Node(
            package="mirte_sorting",
            executable="hybrid_localiser",
            name="hybrid_localiser",
            output="screen",
        ),

        # ── Localisation input: sends initial pose to AMCL when map ready ─
        Node(
            package="mirte_sorting",
            executable="localisation_input_node",
            name="localisation_input_node",
            output="screen",
        ),

    ])
