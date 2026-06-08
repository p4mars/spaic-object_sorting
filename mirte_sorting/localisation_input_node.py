#!/usr/bin/env python3
"""
localisation_input_node.py
──────────────────────────
Sends the initial pose estimate to AMCL once the map is available.
Waits for the /map topic (TRANSIENT_LOCAL) before publishing /initialpose
so AMCL can start localising from (0, 0, 0) by default.
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


class LocalisationInputNode(Node):
    def __init__(self):
        super().__init__("localisation_input_node")

        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self._pose_sent = False

        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self.get_logger().info("Waiting for /map to send initial pose...")

    def _map_cb(self, _):
        if self._pose_sent:
            return
        self._pose_sent = True
        time.sleep(2.0)  # Give AMCL time to fully initialise after map arrives
        self._send_pose()
        self.get_logger().info("Initial pose sent.")

    def _send_pose(self):
        x = self.get_parameter("initial_x").value
        y = self.get_parameter("initial_y").value
        yaw = self.get_parameter("initial_yaw").value

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        import math
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.5
        msg.pose.covariance[7] = 0.5
        msg.pose.covariance[35] = 0.1
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
