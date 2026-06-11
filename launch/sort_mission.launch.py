"""

Main launch file which starts the full Nav2 navigation stack (controller, planner, behaviour server,
BT navigator, velocity smoother and lifecycle manager), RViz for visualisation,
and the sort_mission_planner node that executes the sorting mission.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('bolt_screw_nav2_demo')
    params_file = LaunchConfiguration('params_file')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    use_mock_robot = LaunchConfiguration('use_mock_robot')
    stations = PathJoinSubstitution([pkg_share, 'config', 'stations.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'nav2_demo.rviz'])

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    nav2_nodes = GroupAction([
        SetRemap(src='/cmd_vel_smoothed', dst=cmd_vel_out),

        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen', parameters=[params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', output='screen', parameters=[params_file]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': lifecycle_nodes}],
        ),
    ])

    # Mock robot node, only started when use_mock_robot:=true (laptop testing).
    mock_robot = Node(
        condition=IfCondition(use_mock_robot),
        package='bolt_screw_nav2_demo',
        executable='mock_mirte_base',
        name='mock_mirte_base',
        output='screen',
        parameters=[{
            'cmd_vel_topic': cmd_vel_out,
            'odom_topic': '/mirte_base_controller/odom',
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    # Mission planner is delayed by 30 seconds to give Nav2 time to fully activate
    # before the first navigation goal is sent.
    mission_planner = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='bolt_screw_nav2_demo',
                executable='sort_mission_planner',
                name='sort_mission_planner',
                output='screen',
                parameters=[stations],
            )
        ]
    )

    return LaunchDescription([
        # Full path to the Nav2 YAML parameter file.
        # Defaults to nav2_odom_only.yaml (odometry-only, no SLAM/AMCL)
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_odom_only.yaml']),
            description='Full path to Nav2 parameter file.'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/mirte_base_controller/cmd_vel',
            description='Velocity topic for the MIRTE robot.'),
        DeclareLaunchArgument(
            'use_mock_robot',
            default_value='false',
            description='Set to true for laptop demo without real MIRTE hardware.'),
        mock_robot,
        nav2_nodes,
        rviz,
        mission_planner,
    ])
