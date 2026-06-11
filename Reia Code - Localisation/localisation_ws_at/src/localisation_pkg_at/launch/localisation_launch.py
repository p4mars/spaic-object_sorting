import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('localisation_pkg')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'test_map.yaml'),
        description=' when file launched use: /home/mirte/localisation_ws_at/maps/sorting_map.yaml '
                    # path to map file currently set to test map change based on where linus saves the map '
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='true for Gazebo n false for real robot'
    )

    use_rtabmap_arg = DeclareLaunchArgument(
        'use_rtabmap',
        default_value='false',
        description='true when ya wanna RTAB-Map alongside AMCL'
    )

    semantic_map_arg = DeclareLaunchArgument(
        'semantic_map',
        default_value='',
        description='currently saved in: /home/mirte/localisation_ws_at/maps/semantic_map.yaml'
                    ' but may be bound to change so when launched double check'
    )

    # map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time':  LaunchConfiguration('use_sim_time')
        }]
    )

    # amcl
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            os.path.join(pkg_dir, 'config', 'amcl_config.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # ekf
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf_config.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # lifecycle manager
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localisation',
        parameters=[{
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # input node —-> sets initial pose
    localisation_input_node = Node(
        package='localisation_pkg',
        executable='localisation_input_node',
        name='localisation_input_node',
        parameters=[{'use_sim_time': False}]
    )

    # output node —-> reads position
    localisation_output_node = Node(
        package='localisation_pkg',
        executable='localisation_output_node',
        name='localisation_output_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # hybrid localiser —-> fuses AMCL + RTAB + odometry and provides position
    hybrid_node = Node(
        package='localisation_pkg',
        executable='hybrid_localiser',
        name='hybrid_localiser',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # THE NODE THAT ALWAYS FUSES --> GOT THIS FROM THE LASER BRIGHTSPACE IMG TAKEN IN MECH
    # static TF: base_link → laser
    # AMCL needs this to project laser scans into the map frame
    # x=0.0 y=0.0 z=0.1 = lidar is 10 cm above base_link centre, no rotation
    # Adjust x/y if the lidar is offset forward/sideways on Mirte.
    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0.0', '0.0', '0.1', '0.0', '0.0', '0.0',
                   'base_link', 'laser']
    )

    # apriltag corrector - re-seeds AMCL when a known tag is spotted to account for drift
    # tag_frame_prefix must match the apriltag_ros node publishes:
    #   'tag36h11:' - frames named tag36h11:0, tag36h11:3 …
    #   'tag_'      - frames named tag_0, tag_3 …
    apriltag_corrector_node = Node(
        package='localisation_pkg',
        executable='apriltag_corrector_at',
        name='apriltag_corrector_at',
        parameters=[{
            'semantic_map_path':       LaunchConfiguration('semantic_map'),
            'tag_frame_prefix':        'tag36h11:',
            'camera_frame':            'camera_optical_frame',
            'base_frame':              'base_link',
            'correction_cooldown':     3.0,
            'max_detection_distance':  1.5,
            'use_sim_time':            LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        use_rtabmap_arg,
        semantic_map_arg,
        laser_tf_node,
        map_server_node,
        amcl_node,
        ekf_node,
        lifecycle_node,
        localisation_input_node,
        localisation_output_node,
        hybrid_node,
        apriltag_corrector_node,
    ])

# note to self: executables are the programs to be run