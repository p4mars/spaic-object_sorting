#!/usr/bin/env python3
"""
nav2_client.py
──────────────
Thin safety wrapper around nav2_simple_commander's BasicNavigator, plus the TF
helpers the mission planner needs.

Three safety layers on top of Nav2's own behaviour tree:
  1. Pre-flight  — refuse/warm-up if lidar or odom are stale before a goal.
  2. In-flight   — enforce a hard timeout while a goal is running.
  3. Recovery    — cancel the task and report failure so the planner's state
                   machine can run its own recovery branch.

The planner constructs this with an existing BasicNavigator (itself an rclpy
Node), so we attach our TF listener and sensor subscriptions to that node.
"""

import time

import rclpy
import rclpy.time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class Nav2Client:
    def __init__(self, navigator: BasicNavigator,
                 scan_topic: str = "/scan",
                 odom_topic: str = "/odom",
                 nav_timeout_sec: float = 90.0,
                 sensor_max_age_sec: float = 2.0):
        self._nav = navigator
        self._nav_timeout = nav_timeout_sec
        self._sensor_max_age = sensor_max_age_sec

        self._last_scan_t = 0.0
        self._last_odom_t = 0.0

        # TF + sensor freshness monitors live on the navigator's own node.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._nav)
        self._nav.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)
        self._nav.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

    # ── sensor freshness ──────────────────────────────────────────────────────
    def _scan_cb(self, _msg):
        self._last_scan_t = time.time()

    def _odom_cb(self, _msg):
        self._last_odom_t = time.time()

    def spin_sensors(self, dt: float = 0.1):
        rclpy.spin_once(self._nav, timeout_sec=dt)

    def warm_up(self, secs: float = 3.0):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            self.spin_sensors(0.1)

    def sensors_fresh(self) -> bool:
        now = time.time()
        return ((now - self._last_scan_t) < self._sensor_max_age and
                (now - self._last_odom_t) < self._sensor_max_age)

    # ── TF helper ────────────────────────────────────────────────────────────
    def get_current_pose_from_tf(self, target_frame: str = "map",
                                 source_frame: str = "base_footprint",
                                 fallback_from_odom: bool = False):
        candidates = [(target_frame, source_frame)]
        if fallback_from_odom:
            candidates += [(target_frame, "base_link"),
                           ("odom", "base_footprint"),
                           ("odom", "base_link")]
        for tgt, src in candidates:
            try:
                tf = self._tf_buffer.lookup_transform(
                    tgt, src, rclpy.time.Time(), timeout=Duration(seconds=1.0))
            except Exception:
                continue
            p = PoseStamped()
            p.header.frame_id = tgt
            p.header.stamp = self._nav.get_clock().now().to_msg()
            p.pose.position.x = tf.transform.translation.x
            p.pose.position.y = tf.transform.translation.y
            p.pose.position.z = tf.transform.translation.z
            p.pose.orientation = tf.transform.rotation
            return p
        return None

    # ── navigation with monitoring ────────────────────────────────────────────
    def go_to(self, pose: PoseStamped, label: str = "goal") -> bool:
        # Layer 1 — pre-flight.
        if not self.sensors_fresh():
            self._nav.get_logger().warn(
                f"[nav2_client] sensors stale before '{label}' — warming up.")
            self.warm_up(2.0)

        self._nav.get_logger().info(f"[nav2_client] → {label}")
        self._nav.goToPose(pose)

        # Layer 2 — in-flight timeout monitoring.
        start = time.time()
        while not self._nav.isTaskComplete():
            self.spin_sensors(0.1)
            if time.time() - start > self._nav_timeout:
                self._nav.get_logger().error(
                    f"[nav2_client] '{label}' exceeded {self._nav_timeout:.0f}s "
                    f"— cancelling.")
                self._nav.cancelTask()      # Layer 3 — bail out, let FSM recover.
                return False

        result = self._nav.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        self._nav.get_logger().info(
            f"[nav2_client] '{label}' result: {result} ({'OK' if ok else 'FAIL'})")
        return ok
