from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── 1. Perception node ────────────────────────────────────────────
        Node(
            package="mirte_detectio",
            executable="Detection_Linux_v2",
            name="perception_node",
            output="screen",
        ),

        # ── 2. Motor control node (navigate to block) ─────────────────────
        Node(
            package="mirte_detectio",
            executable="Detection_motor_control",
            name="motor_control_node",
            output="screen",
        ),

        # ── 3. Pickup arm node ────────────────────────────────────────────
        Node(
            package="mirte_detectio",
            executable="Detection_pickup_v2",
            name="arm_node_pickup",
            output="screen",
        ),

        # ── 4. Drop-off motor node (drive to drop zone) ───────────────────
        Node(
            package="mirte_detectio",
            executable="Detection_dropoff_motor",
            name="dropoff_motor_node",
            output="screen",
        ),

        # ── 5. Drop-off arm node ──────────────────────────────────────────
        Node(
            package="mirte_detectio",
            executable="Detection_dropoff",
            name="arm_node_dropoff",
            output="screen",
        ),
    ])
