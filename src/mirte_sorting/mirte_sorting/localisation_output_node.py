#!/usr/bin/env python3
"""
localisation_output_node.py
───────────────────────────
Subscribes to /robot_pose (the fused output from hybrid_localiser) and logs
the robot's position and heading. Reflects whichever source is active:
AMCL, RTAB-Map fusion, or odometry fallback.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class LocalisationOutputNode(Node):
    def __init__(self):
        super().__init__("localisation_output_node")

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.create_subscription(
            PoseWithCovarianceStamped, "/robot_pose", self._pose_cb, 10)
        self.get_logger().info("LocalisationOutputNode started.")

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        self.get_logger().debug(
            f"pose → x={self.current_x:.3f}  y={self.current_y:.3f}  "
            f"yaw={math.degrees(self.current_yaw):.1f}°")

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def get_position(self):
        return (self.current_x, self.current_y, self.current_yaw)


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
