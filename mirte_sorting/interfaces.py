"""
interfaces.py
─────────────
All inter-node ROS 2 topic names in one place.
Import these constants instead of using raw strings anywhere.

═══════════════════════════════════════════════════════════
 INTER-NODE CONTRACT
═══════════════════════════════════════════════════════════

Mission planner PUBLISHES
─────────────────────────
  /nav/status   std_msgs/String
    "GOING_TO_SOURCE"      — driving to source station
    "AT_SOURCE"            — at source, arm should pick
    "GOING_TO_DESTINATION" — driving toward bin cluster
    "AT_DESTINATION"       — at correct bin, arm should drop
    "RETURNING_HOME"       — all sorted, driving to start
    "DONE"                 — mission complete
    "ABORTED"              — mission aborted

Mission planner SUBSCRIBES
──────────────────────────
  /object_class   String  — shape name: "heart"|"triangle"|"hexagon"|"l_shape"
  /source_empty   Bool    — True when no more objects at source
  /pick_complete  Bool    — arm: pick succeeded
  /pick_failed    Bool    — arm: pick failed
  /drop_complete  Bool    — arm: drop succeeded
  /drop_failed    Bool    — arm: drop failed

Perception node PUBLISHES
─────────────────────────
  /arm_grasp_pose     PoseStamped — exact object position for arm
  /object_class       String      — YOLO class name
  /source_empty       Bool        — True when search exhausted
  /detected_object/pos PoseStamped — nav waypoint (informational)

Arm nodes SUBSCRIBE / PUBLISH
──────────────────────────────
  Pickup: subscribes /nav/status + /arm_grasp_pose
          publishes  /pick_complete, /pick_failed, /object_class
  Dropoff: subscribes /nav/status
           publishes  /drop_complete, /drop_failed
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# ── Topic names ────────────────────────────────────────────────────────────────

NAV_STATUS_TOPIC   = "/nav/status"
OBJ_CLASS_TOPIC    = "/object_class"
SOURCE_EMPTY_TOPIC = "/source_empty"
PICK_DONE_TOPIC    = "/pick_complete"
PICK_FAILED_TOPIC  = "/pick_failed"
DROP_DONE_TOPIC    = "/drop_complete"
DROP_FAILED_TOPIC  = "/drop_failed"

# ── Status string values ───────────────────────────────────────────────────────

STATUS_GOING_SOURCE      = "GOING_TO_SOURCE"
STATUS_AT_SOURCE         = "AT_SOURCE"
STATUS_GOING_DESTINATION = "GOING_TO_DESTINATION"
STATUS_AT_DESTINATION    = "AT_DESTINATION"
STATUS_RETURNING_HOME    = "RETURNING_HOME"
STATUS_DONE              = "DONE"
STATUS_ABORTED           = "ABORTED"


# ── TeamBridge node ────────────────────────────────────────────────────────────

class TeamBridge(Node):
    """
    Bridges communication between the mission planner and arm/vision nodes.

    Publishes  /nav/status  (RELIABLE)
    Subscribes to all arm/vision feedback topics (BEST_EFFORT)
    """

    def __init__(self):
        super().__init__("sorting_team_bridge")

        _sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._status_pub = self.create_publisher(String, NAV_STATUS_TOPIC, 10)

        self.object_class: str  = ""
        self.source_empty: bool = False
        self.pick_done:    bool = False
        self.pick_failed:  bool = False
        self.drop_done:    bool = False
        self.drop_failed:  bool = False

        self.create_subscription(String, OBJ_CLASS_TOPIC,    self._class_cb,     _sub_qos)
        self.create_subscription(Bool,   SOURCE_EMPTY_TOPIC, self._empty_cb,     _sub_qos)
        self.create_subscription(Bool,   PICK_DONE_TOPIC,    self._pick_done_cb, _sub_qos)
        self.create_subscription(Bool,   PICK_FAILED_TOPIC,  self._pick_fail_cb, _sub_qos)
        self.create_subscription(Bool,   DROP_DONE_TOPIC,    self._drop_done_cb, _sub_qos)
        self.create_subscription(Bool,   DROP_FAILED_TOPIC,  self._drop_fail_cb, _sub_qos)

        self.get_logger().info("TeamBridge ready.")

    # Callbacks ----------------------------------------------------------------

    def _class_cb(self, m: String):
        self.object_class = m.data.strip().lower()
        self.get_logger().info(f"[VISION] class='{self.object_class}'")

    def _empty_cb(self, m: Bool):
        if m.data:
            self.source_empty = True
            self.get_logger().info("[VISION] Source EMPTY")

    def _pick_done_cb(self, m: Bool):
        if m.data:
            self.pick_done = True
            self.get_logger().info("[ARM] Pick COMPLETE")

    def _pick_fail_cb(self, m: Bool):
        if m.data:
            self.pick_failed = True
            self.get_logger().warn("[ARM] Pick FAILED")

    def _drop_done_cb(self, m: Bool):
        if m.data:
            self.drop_done = True
            self.get_logger().info("[ARM] Drop COMPLETE")

    def _drop_fail_cb(self, m: Bool):
        if m.data:
            self.drop_failed = True
            self.get_logger().warn("[ARM] Drop FAILED")

    # Public API ---------------------------------------------------------------

    def publish_status(self, status: str):
        self._status_pub.publish(String(data=status))
        self.get_logger().info(f"[NAV STATUS] → {status}")

    def reset_for_new_cycle(self):
        self.object_class = ""
        self.pick_done    = False
        self.pick_failed  = False
        self.drop_done    = False
        self.drop_failed  = False

    def _tick(self, dt: float = 0.1):
        rclpy.spin_once(self, timeout_sec=dt)

    def wait_for_object_class(self, timeout_sec: float) -> str:
        self.object_class = ""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self._tick(0.1)
            if self.object_class:
                return self.object_class
        self.get_logger().warn(
            f"Timeout ({timeout_sec:.0f}s) waiting for {OBJ_CLASS_TOPIC}.")
        return ""

    def wait_for_pick(self, timeout_sec: float) -> bool:
        self.pick_done   = False
        self.pick_failed = False
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self._tick(0.1)
            if self.pick_done:
                return True
            if self.pick_failed:
                return False
        self.get_logger().warn(
            f"Timeout ({timeout_sec:.0f}s) waiting for pick result.")
        return False

    def wait_for_drop(self, timeout_sec: float) -> bool:
        self.drop_done   = False
        self.drop_failed = False
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self._tick(0.1)
            if self.drop_done:
                return True
            if self.drop_failed:
                return False
        self.get_logger().warn(
            f"Timeout ({timeout_sec:.0f}s) waiting for drop result.")
        return False
