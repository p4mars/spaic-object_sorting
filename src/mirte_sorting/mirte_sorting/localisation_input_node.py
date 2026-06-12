#!/usr/bin/env python3
"""
localisation_input_node.py
──────────────────────────
Sends the initial pose to AMCL once the map is available.

Two modes (controlled by use_apriltag_init parameter):

  AprilTag mode (use_apriltag_init: true)  — AUTO START POSITION
  ─────────────────────────────────────────────────────────────────
  Looks up a known "home" AprilTag in the TF tree after the map arrives.
  apriltag_ros publishes  camera_frame → tag_{home_tag_id}  when it detects
  the tag.  This node chains that with  base_footprint → camera_frame  (from
  the robot URDF) to compute  base_footprint → tag_{id},  then uses the tag's
  known map pose to derive the robot's map pose.

  Maths (2-D):
    robot_yaw  = tag_map_yaw  − tag_yaw_in_robot_frame
    robot_x    = tag_map_x   − cos(robot_yaw)·bx + sin(robot_yaw)·by
    robot_y    = tag_map_y   − sin(robot_yaw)·bx − cos(robot_yaw)·by
  where (bx, by, tag_yaw_in_robot_frame) = base_footprint → tag TF.

  Falls back to the configured (initial_x, initial_y, initial_yaw) if no
  tag is seen within apriltag_timeout_s seconds.

  YAML config:
    use_apriltag_init: true
    home_tag_id: 0           # AprilTag ID placed at the start position
    home_tag_map_x: 0.0      # tag's known position in the map frame
    home_tag_map_y: 0.0
    home_tag_map_yaw: 0.0
    apriltag_timeout_s: 5.0

  Fallback / fixed mode (use_apriltag_init: false, default)
  ──────────────────────────────────────────────────────────
  Sends the fixed (initial_x, initial_y, initial_yaw) pose straight to AMCL
  after a 2-second delay once the map is loaded.
"""

import math
import time

import rclpy
import rclpy.time
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformException


class LocalisationInputNode(Node):
    def __init__(self):
        super().__init__("localisation_input_node")

        self.declare_parameter("initial_x",           0.0)
        self.declare_parameter("initial_y",           0.0)
        self.declare_parameter("initial_yaw",         0.0)
        self.declare_parameter("use_apriltag_init",   False)
        self.declare_parameter("home_tag_id",         0)
        self.declare_parameter("home_tag_map_x",      0.0)
        self.declare_parameter("home_tag_map_y",      0.0)
        self.declare_parameter("home_tag_map_yaw",    0.0)
        self.declare_parameter("apriltag_timeout_s",  5.0)

        self._pose_sent = False
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)
        self.get_logger().info("LocalisationInputNode: waiting for /map...")

    # ── Map callback ───────────────────────────────────────────────────────────

    def _map_cb(self, _):
        if self._pose_sent:
            return
        self._pose_sent = True
        self._init_timer = self.create_timer(2.0, self._delayed_send)

    def _delayed_send(self):
        self._init_timer.cancel()

        use_tag = self.get_parameter("use_apriltag_init").value
        if use_tag:
            tag_id      = self.get_parameter("home_tag_id").value
            tag_map_x   = self.get_parameter("home_tag_map_x").value
            tag_map_y   = self.get_parameter("home_tag_map_y").value
            tag_map_yaw = self.get_parameter("home_tag_map_yaw").value
            timeout     = self.get_parameter("apriltag_timeout_s").value

            result = self._pose_from_tag(tag_id, tag_map_x, tag_map_y, tag_map_yaw, timeout)
            if result is not None:
                rx, ry, ryaw = result
                self.get_logger().info(
                    f"AprilTag init: tag_{tag_id} → "
                    f"x={rx:.3f} y={ry:.3f} yaw={math.degrees(ryaw):.1f}°")
                self._send_pose(rx, ry, ryaw)
                return
            self.get_logger().warn(
                f"AprilTag (id={tag_id}) not detected within {timeout:.0f}s "
                "— falling back to configured pose.")

        # Fixed / fallback pose
        x   = self.get_parameter("initial_x").value
        y   = self.get_parameter("initial_y").value
        yaw = self.get_parameter("initial_yaw").value
        self.get_logger().info(
            f"Initial pose: x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°")
        self._send_pose(x, y, yaw)

    # ── AprilTag pose computation ───────────────────────────────────────────────

    def _pose_from_tag(self, tag_id, tag_map_x, tag_map_y, tag_map_yaw, timeout_s):
        """
        Returns (robot_x, robot_y, robot_yaw) in the map frame, or None on timeout.

        Needs apriltag_ros to publish the TF: <camera_frame> → tag_<tag_id>
        AND the robot URDF to have: base_footprint → <camera_frame> in TF.
        tf2 chains these automatically → base_footprint → tag_<tag_id>.
        """
        tag_frame = f"tag_{tag_id}"
        deadline  = time.time() + timeout_s

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self._tf_buf.lookup_transform(
                    "base_footprint", tag_frame, rclpy.time.Time())
            except TransformException:
                continue

            # Tag position and heading in robot (base_footprint) frame
            bx   = t.transform.translation.x
            by   = t.transform.translation.y
            q    = t.transform.rotation
            byaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

            # Robot pose in map frame (see module docstring for derivation)
            robot_yaw = tag_map_yaw - byaw
            robot_x   = tag_map_x - (math.cos(robot_yaw) * bx - math.sin(robot_yaw) * by)
            robot_y   = tag_map_y - (math.sin(robot_yaw) * bx + math.cos(robot_yaw) * by)
            return robot_x, robot_y, robot_yaw

        return None

    # ── Pose publisher ─────────────────────────────────────────────────────────

    def _send_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0]  = 0.5
        msg.pose.covariance[7]  = 0.5
        msg.pose.covariance[35] = 0.1
        self._pub.publish(msg)
        self.get_logger().info("Initial pose published to /initialpose.")


def main(args=None):
    rclpy.init(args=args)
    node = LocalisationInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
