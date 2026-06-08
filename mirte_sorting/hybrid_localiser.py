#!/usr/bin/env python3
"""
hybrid_localiser.py
───────────────────
Fuses AMCL and RTAB-Map pose estimates into a single /robot_pose output.
Falls back to raw odometry if neither map-based source is available.

Publishes:
  /robot_pose  (PoseWithCovarianceStamped, frame=map)

Subscribes:
  /amcl_pose                    — AMCL particle filter estimate
  /rtabmap/localization_pose    — RTAB-Map visual localisation (optional)
  /odometry/filtered            — EKF-fused odometry fallback
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HybridLocaliser(Node):
    def __init__(self):
        super().__init__("hybrid_localiser")

        self._amcl_pose = None
        self._rtab_pose = None
        self._odom_pose = None

        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/rtabmap/localization_pose", self._rtab_cb, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._odom_cb, 10)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10)

        self.create_timer(0.1, self._publish_pose)
        self.get_logger().info("HybridLocaliser started.")

    def _amcl_cb(self, msg): self._amcl_pose = msg
    def _rtab_cb(self, msg): self._rtab_pose = msg
    def _odom_cb(self, msg): self._odom_pose = msg

    @staticmethod
    def _uncertainty(pose_msg: PoseWithCovarianceStamped) -> float:
        return math.sqrt(abs(pose_msg.pose.covariance[0]))

    def _publish_pose(self):
        available = []
        for pose, name in ((self._amcl_pose, "AMCL"), (self._rtab_pose, "RTAB")):
            if pose is not None:
                u = self._uncertainty(pose)
                w = max(0.1, 1.0 - u * 2.0)
                available.append((pose, w, name))

        if not available:
            if self._odom_pose is not None:
                self.get_logger().warn_once("No map sources — using raw odometry.")
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = "map"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose = self._odom_pose.pose.pose
                self._pub.publish(msg)
            return

        total_w = sum(w for _, w, _ in available)
        x = sum(p.pose.pose.position.x * w / total_w for p, w, _ in available)
        y = sum(p.pose.pose.position.y * w / total_w for p, w, _ in available)
        best_pose = max(available, key=lambda t: t[1])[0]

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = best_pose.pose.pose.orientation
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HybridLocaliser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
