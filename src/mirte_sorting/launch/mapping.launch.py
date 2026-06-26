"""
mapping.launch.py
─────────────────
STEP 1: Build the arena map before demo day.

Starts:
  - robot_localization EKF      — fuses /odom + /imu → /odometry/filtered
  - SLAM Toolbox (online async) — builds the 2-D occupancy grid map (/map)
  - RTAB-Map SLAM (optional)    — builds visual database (see rtabmap_db arg)
    Pass use_rtabmap_mapping:=true to enable alongside SLAM Toolbox.
    RTAB-Map runs with publish_tf:=false so it does not conflict with SLAM Toolbox.
  - AprilTag + CameraInfoSync   — detect station/bin tags during mapping
  - SemanticMapNode             — records tag positions in map frame
  - Odom relay                  — /mirte_base_controller/odom → /odom
  - Compatibility static TFs

When done driving:
  ros2 service call /save_semantic_map std_srvs/srv/Trigger
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\
    "{name: {data: '<map_save_path>'}}"
  (map_save_path defaults to <pkg_share>/maps/sorting_map)

Usage:
  ros2 launch mirte_sorting mapping.launch.py

  Also build RTAB-Map visual database:
  ros2 launch mirte_sorting mapping.launch.py use_rtabmap_mapping:=true

  Override save locations:
  ros2 launch mirte_sorting mapping.launch.py \\
    rtabmap_db:=/custom/path/rtabmap.db \\
    map_save_path:=/custom/path/sorting_map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("mirte_sorting")

    slam_params    = PathJoinSubstitution([pkg, "config", "slam_params.yaml"])
    ekf_params     = PathJoinSubstitution([pkg, "config", "ekf.yaml"])
    rtabmap_params = PathJoinSubstitution([pkg, "config", "rtabmap.yaml"])
    apriltag_cfg   = PathJoinSubstitution([pkg, "config", "apriltag.yaml"])
    semantic_cfg   = PathJoinSubstitution([pkg, "config", "semantic_map_config.yaml"])

    maps_dir = PathJoinSubstitution([pkg, "maps"])

    use_odom_relay       = LaunchConfiguration("use_odom_relay")
    publish_compat_tfs   = LaunchConfiguration("publish_compat_tfs")
    use_sim_time         = LaunchConfiguration("use_sim_time")
    use_rtabmap_mapping  = LaunchConfiguration("use_rtabmap_mapping")
    rtabmap_db           = LaunchConfiguration("rtabmap_db")
    map_save_path        = LaunchConfiguration("map_save_path")

    return LaunchDescription([

        # ── Pre-launch cleanup ─────────────────────────────────────────────
        # Kill any stale nav2 / AMCL processes — if navigation.launch.py was
        # left running it publishes map→odom via AMCL and conflicts with SLAM.
        ExecuteProcess(
            cmd=['bash', '-c',
                 'pkill -f amcl 2>/dev/null; '
                 'pkill -f map_server 2>/dev/null; '
                 'sleep 0.5; '
                 'rm -f /dev/shm/fastrtps_port*; '
                 'true'],
            output='screen',
            name='pre_launch_cleanup',
        ),

        DeclareLaunchArgument("use_odom_relay",
            default_value="true",
            description="Relay /mirte_base_controller/odom → /odom"),
        DeclareLaunchArgument("publish_compat_tfs",
            default_value="true",
            description="Publish base_link ↔ base_footprint static TF"),
        DeclareLaunchArgument("use_sim_time",
            default_value="false"),
        DeclareLaunchArgument("use_rtabmap_mapping",
            default_value="false",
            description="Also run RTAB-Map SLAM to build the visual database"),
        DeclareLaunchArgument("rtabmap_db",
            default_value=PathJoinSubstitution([maps_dir, "rtabmap.db"]),
            description="Path to write the RTAB-Map visual database"),
        DeclareLaunchArgument("map_save_path",
            default_value=PathJoinSubstitution([maps_dir, "sorting_map"]),
            description="Save prefix for SLAM Toolbox map (no extension)"),

        # ── EKF: fuses /odom + /imu → /odometry/filtered ──────────────────
        # SLAM Toolbox benefits from the smoothed odometry for scan matching.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        ),

        # ── SLAM Toolbox: 2-D occupancy grid ──────────────────────────────
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),

        # ── RTAB-Map SLAM (optional): visual database ──────────────────────
        # publish_tf:false — SLAM Toolbox owns map→odom; RTAB-Map only saves
        # the visual database used later in localisation mode.
        Node(
            condition=IfCondition(use_rtabmap_mapping),
            package="rtabmap_ros",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[
                rtabmap_params,
                {"Mem/IncrementalMemory": "true",
                 "Mem/InitWMWithAllNodes": "false",
                 "publish_tf": False,
                 "database_path": rtabmap_db},
            ],
            remappings=[
                ("rgb/image",       "/camera/color/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("depth/image",     "/camera/depth/image_raw"),
                ("odom",            "/odometry/filtered"),
            ],
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

        # ── Camera info sync → AprilTag ────────────────────────────────────
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
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            output="screen",
            parameters=[apriltag_cfg],
            remappings=[
                ("image_rect",   "/apriltag/image_rect"),
                ("camera_info",  "/apriltag/camera_info"),
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
    ])
