import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # pretty much uses everything from the rtabmap config file

    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':          False,
            'frame_id':              'base_link',
            'odom_frame_id':         'odom',
            'subscribe_depth':       True,
            'subscribe_scan':        True,
            'approx_sync':           True,
            'queue_size':            10,

            # sensor topics
            'depth_topic':           '/camera/depth/image_raw',
            'rgb_topic':             '/camera/color/image_raw',
            'camera_info_topic':     '/camera/color/camera_info',
            'scan_topic':            '/scan',

            # memory
            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes':'false',

            # loop closure
            'Rtabmap/DetectionRate': '1',
            'Vis/MinInliers':        '10',

            # CPU optimisation for Mirte ARM64
            'RGBD/NeighborLinkRefining': 'false',
            'RGBD/ProximityBySpace':     'false',
            'Grid/FromDepth':            'true',
            'Grid/RangeMax':             '4.0',
        }],
        remappings=[
            ('rgb/image',            '/camera/color/image_raw'),
            ('rgb/camera_info',      '/camera/color/camera_info'),
            ('depth/image',          '/camera/depth/image_raw'),
            ('scan',                 '/scan'),
            ('odom',                 '/odometry/filtered'),
        ]
    )

    rtabmap_viz_node = Node(
        package='rtabmap_ros',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id':     'base_link',
        }]
    )

    return LaunchDescription([
        rtabmap_node,
        # rtabmap_viz_node,  # to be used only when display is available
    ])