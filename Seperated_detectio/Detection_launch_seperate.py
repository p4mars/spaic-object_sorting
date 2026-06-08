from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Detection node
        Node(
            package="mirte_group2",
            executable="Detection_Linux_v2",
            name="perception_node",
            output="screen"
        ),

        # Pickup node
        Node(
            package="mirte_group2",
            executable="Detection_pickup_v2",
            name="arm_node_pickup",
            output="screen"
        ),

        # Dropoff node
        Node(
            package="mirte_group2",
            executable="Detection_dropoff",
            name="arm_node_dropoff",
            output="screen"
        ),
    ])
