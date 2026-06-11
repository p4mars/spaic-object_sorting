"""
Launch file for the Nav2 navigation stack (without mission planner).

Starts all Nav2 servers (controller, planner, behaviour, bt_navigator,
smoother, waypoint follower and velocity smoother) managed by a lifecycle
manager. Remaps /cmd_vel_smoothed to the MIRTE base controller topic.

"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('bolt_screw_nav2_demo') # Used to resolve paths to config files and parameter files
    params_file = LaunchConfiguration('params_file')
    use_mock_robot = LaunchConfiguration('use_mock_robot')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out')

    # All Nav2 servers managed by the lifecycle manager
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # Group all Nav2 nodes so the cmd_vel_smoothed remap applies to all of them
    nav2_nodes = GroupAction([
        # Nav2 publishes normal /cmd_vel. Velocity smoother subscribes to /cmd_vel
        # and publishes /cmd_vel_smoothed. Remap /cmd_vel_smoothed to the Mirte topic.
        SetRemap(src='/cmd_vel_smoothed', dst=cmd_vel_out),

        Node(package='nav2_controller', executable='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_smoother', executable='smoother_server', output='screen', parameters=[params_file]),
        Node(package='nav2_planner', executable='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen', parameters=[params_file]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen', parameters=[params_file]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', output='screen', parameters=[params_file]),
        
        # Lifecycle manager,  automatically configures and activates all Nav2 nodes above
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False}, {'autostart': True}, {'node_names': lifecycle_nodes}],
        ),
    ])

    # Mock robot node, only started when use_mock_robot:=true (testing on computer)
    mock = Node(
        condition=IfCondition(use_mock_robot),
        package='bolt_screw_nav2_demo',
        executable='mock_mirte_base',
        name='mock_mirte_base',
        output='screen',
        parameters=[{'cmd_vel_topic': cmd_vel_out}],
    )

    return LaunchDescription([
        # Full path to the Nav2 YAML parameter file
        # Defaults to nav2_odom_only.yaml
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_odom_only.yaml']),
            description='Full path to Nav2 parameter file.'),
        DeclareLaunchArgument(
    'use_mock_robot',
    default_value='true',
    description='Set to true for laptop testing without real MIRTE hardware. Set to false when running on the robot.'),
DeclareLaunchArgument(
    'cmd_vel_out',
    default_value='/mirte_base_controller/cmd_vel',  
    description='Velocity topic the MIRTE base controller subscribes to. Change to /cmd_vel_smoothed for laptop testing.'),
mock,
        mock,
        nav2_nodes,
    ])
