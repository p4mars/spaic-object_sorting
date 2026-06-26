#!/usr/bin/env python3
"""
hybrid_localiser.py
───────────────────
Fuses pose estimates into a single /robot_pose output using this fallback chain:

  1. RTAB-Map  (visual, highest accuracy — only when use_rtabmap:=true)
  2. AMCL      (2-D laser particle filter — requires a loaded map)
  3. EKF odom  (/odometry/filtered — no map needed, drifts over time)

When multiple map-based sources are fresh the output is a weighted average
(lower covariance = higher weight). If none are fresh the node falls back to
raw EKF odometry and logs a one-time warning.

Publishes:
  /robot_pose          PoseWithCovarianceStamped  frame=map
  /localisation/source String  — "RTAB+AMCL" | "RTAB" | "AMCL" | "ODOM" | "NONE"

Subscribes:
  /amcl_pose                 — AMCL particle filter
  /rtabmap/localization_pose — RTAB-Map visual (only when use_rtabmap:=true)
  /odometry/filtered         — robot_localization EKF fallback

Parameters:
  use_rtabmap  (bool, default false)
"""

import math

import rclpy
import rclpy.time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

_POSE_MAX_AGE_S = 2.0

# AMCL publishes /amcl_pose latched (TRANSIENT_LOCAL + RELIABLE).
# A VOLATILE subscriber receives nothing — match the durability.
_AMCL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class HybridLocaliser(Node):
    def __init__(self):
        super().__init__("hybrid_localiser")

        self.declare_parameter("use_rtabmap", False)
        use_rtabmap = self.get_parameter("use_rtabmap").value

        self._amcl_pose = None
        self._rtab_pose = None
        self._odom_pose = None
        self._last_source = ""

        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, _AMCL_QOS)

        if use_rtabmap:
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/rtabmap/localization_pose", self._rtab_cb, 10)
            self.get_logger().info("HybridLocaliser: RTAB-Map fusion enabled.")
        else:
            self.get_logger().info("HybridLocaliser: AMCL only (RTAB-Map disabled).")

        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10)

        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, "/robot_pose", 10)
        self._src_pub = self.create_publisher(String, "/localisation/source", 10)

        self.create_timer(0.1, self._publish_pose)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _amcl_cb(self, msg): self._amcl_pose = msg
    def _rtab_cb(self, msg): self._rtab_pose = msg
    def _odom_cb(self, msg): self._odom_pose = msg

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _uncertainty(pose_msg: PoseWithCovarianceStamped) -> float:
        return math.sqrt(
            abs(pose_msg.pose.covariance[0]) + abs(pose_msg.pose.covariance[7]))

    def _is_fresh(self, pose_msg: PoseWithCovarianceStamped) -> bool:
        age = (
            self.get_clock().now()
            - rclpy.time.Time.from_msg(pose_msg.header.stamp)
        ).nanoseconds / 1e9
        return age < _POSE_MAX_AGE_S

    def _weighted_fuse(self, available):
        """Weighted average of (pose, weight, name) tuples. Returns x, y, best_pose."""
        total_w = sum(w for _, w, _ in available)
        x = sum(p.pose.pose.position.x * w / total_w for p, w, _ in available)
        y = sum(p.pose.pose.position.y * w / total_w for p, w, _ in available)
        best_pose = max(available, key=lambda t: t[1])[0]
        return x, y, best_pose

    # ── Main publish loop ──────────────────────────────────────────────────────

    def _publish_pose(self):
        # Collect fresh map-based sources
        available = []
        for pose, name in ((self._rtab_pose, "RTAB"), (self._amcl_pose, "AMCL")):
            if pose is not None and self._is_fresh(pose):
                u = self._uncertainty(pose)
                w = max(0.1, 1.0 - u * 2.0)
                available.append((pose, w, name))

        if available:
            x, y, best_pose = self._weighted_fuse(available)
            source = "+".join(n for _, _, n in available)

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.orientation = best_pose.pose.pose.orientation
            msg.pose.covariance = best_pose.pose.covariance
            self._pub.publish(msg)
            self._publish_source(source)
            return

        # Fallback: EKF odometry (no map)
        if self._odom_pose is not None:
            self.get_logger().warn_once(
                "HybridLocaliser: no map sources — falling back to EKF odometry.")
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = self._odom_pose.pose.pose
            msg.pose.covariance = self._odom_pose.pose.covariance
            self._pub.publish(msg)
            self._publish_source("ODOM")
            return

        self._publish_source("NONE")

    def _publish_source(self, source: str):
        if source != self._last_source:
            self.get_logger().info(f"HybridLocaliser source → {source}")
            self._last_source = source
        self._src_pub.publish(String(data=source))


def main(args=None):
    rclpy.init(args=args)
    node = HybridLocaliser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
