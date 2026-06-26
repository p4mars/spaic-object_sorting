#!/usr/bin/env python3
"""
mission_planner_node.py
───────────────────────
Central mission planner for the MIRTE sorting robot.

State machine flow:
  INIT → SAVE_START_POSE → WAIT_FOR_NAV2
  → NAVIGATE_TO_SOURCE → DETECT_OBJECT → PICK_OBJECT
  → NAVIGATE_TO_DESTINATION → DETECT_BIN → DROP_OBJECT
  → CHECK_REMAINING → (loop or RETURN_TO_START → TASK_COMPLETE)

Inter-node interface (see interfaces.py):
  Publishes  /nav/status        (String) — arm/perception nodes react to this
  Subscribes /pick_complete     (Bool)
             /drop_complete     (Bool)
             /object_class      (String)
             /source_empty      (Bool)
"""

import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mirte_sorting.interfaces import (
    TeamBridge,
    STATUS_AT_DESTINATION,
    STATUS_AT_SOURCE,
    STATUS_DONE,
    STATUS_ABORTED,
    STATUS_GOING_SOURCE,
    STATUS_GOING_DESTINATION,
    STATUS_RETURNING_HOME,
)
from mirte_sorting.nav2_client import Nav2Client
from mirte_sorting.station_manager import StationManager
from mirte_sorting.task_state_machine import MissionState, StateMachine

# ── Timing parameters ──────────────────────────────────────────────────────────
ARM_TIMEOUT_SEC    = 30.0
CLASS_TIMEOUT_SEC  = 15.0
MAP_TIMEOUT_SEC    = 60.0
MAX_NAV_RETRIES    = 2
MAX_PICK_RETRIES   = 1
MAX_DROP_RETRIES   = 1
MAX_DETECT_RETRIES = 3


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        self.get_logger().info("MissionPlannerNode initialising...")

        self._sm       = StateMachine()
        self._nav2     = BasicNavigator()
        self._nav      = Nav2Client(self._nav2)
        self._stations = StationManager(self)
        self._bridge   = TeamBridge()

        self._sorted_count    = 0
        self._current_shape   = ""
        self._last_failed_leg = ""

        # Subscribe to /map with TRANSIENT_LOCAL so we receive it even if
        # it was published before we started.
        _map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._map_received = False
        self.create_subscription(OccupancyGrid, "/map", self._map_cb, _map_qos)

        self.get_logger().info("MissionPlannerNode ready.")

    def _map_cb(self, _):
        if not self._map_received:
            self._map_received = True
            self.get_logger().info("[MAP] Occupancy grid received.")

    def _info(self, m):  self.get_logger().info(m)
    def _warn(self, m):  self.get_logger().warn(m)
    def _error(self, m): self.get_logger().error(m)

    def _spin_all(self, dt: float = 0.1):
        rclpy.spin_once(self._bridge, timeout_sec=dt)
        self._nav.spin_sensors(dt)
        rclpy.spin_once(self, timeout_sec=0.01)

    # ── State handlers ─────────────────────────────────────────────────────────

    def _handle_init(self):
        self._info("── INIT ──")
        self._sm.transition(MissionState.SAVE_START_POSE)

    def _handle_save_start_pose(self):
        self._info("── SAVE_START_POSE ──")
        self._nav.warm_up(3.0)
        pose = self._nav.get_current_pose_from_tf(
            target_frame="map", source_frame="base_footprint",
            fallback_from_odom=True)
        if pose:
            self._stations.save_start_pose(pose)
        else:
            self._warn("Could not read start pose — will use (0,0,0) on return.")
        self._stations.log_all()
        self._sm.transition(MissionState.WAIT_FOR_NAV2)

    def _handle_wait_for_nav2(self):
        self._info("── WAIT_FOR_NAV2 ──")
        deadline = time.time() + MAP_TIMEOUT_SEC
        while not self._map_received and time.time() < deadline:
            self._spin_all(0.2)
        if not self._map_received:
            self._error(
                "No /map within timeout! "
                "Start SLAM: ros2 launch mirte_sorting mapping.launch.py")
            self._sm.force(MissionState.ABORT)
            return
        self._info("Map received. Waiting for Nav2 lifecycle nodes...")
        self._nav2.waitUntilNav2Active()
        self._info("Nav2 active — mission starting!")
        self._sm.transition(MissionState.NAVIGATE_TO_SOURCE)

    def _handle_navigate_to_source(self):
        self._info(f"── NAVIGATE_TO_SOURCE (sorted: {self._sorted_count}) ──")
        self._bridge.reset_for_new_cycle()
        self._bridge.publish_status(STATUS_GOING_SOURCE)
        ok = self._nav.go_to(self._stations.source_station(), "Source Station")
        if ok:
            self._sm.transition(MissionState.DETECT_OBJECT)
        else:
            self._last_failed_leg = "source"
            self._sm.transition(MissionState.NAVIGATION_FAILED)

    def _handle_detect_object(self):
        self._info("── DETECT_OBJECT ──")
        self._bridge.publish_status(STATUS_AT_SOURCE)
        self._info("At source. Waiting for arm to pick...")

        pick_ok = self._bridge.wait_for_pick(ARM_TIMEOUT_SEC)
        if not pick_ok:
            self._warn(f"Pick timed out or failed after {ARM_TIMEOUT_SEC:.0f}s.")
            self._sm.transition(MissionState.PICK_FAILED)
            return

        self._info("Pick confirmed. Waiting for object class...")
        shape = self._bridge.wait_for_object_class(CLASS_TIMEOUT_SEC)
        if not shape:
            self._warn(
                f"No class received within {CLASS_TIMEOUT_SEC:.0f}s. "
                f"Publish shape name on /object_class.")
            self._sm.transition(MissionState.OBJECT_NOT_FOUND)
            return

        self._current_shape = shape
        self._info(f"Shape: '{self._current_shape}'")
        self._sm.transition(MissionState.NAVIGATE_TO_DESTINATION)

    def _handle_navigate_to_destination(self):
        self._info("── NAVIGATE_TO_DESTINATION ──")
        self._bridge.publish_status(STATUS_GOING_DESTINATION)
        ok = self._nav.go_to(
            self._stations.destination_station(), "Destination (approach)")
        if ok:
            self._sm.transition(MissionState.DETECT_BIN)
        else:
            self._last_failed_leg = "destination"
            self._sm.transition(MissionState.NAVIGATION_FAILED)

    def _handle_detect_bin(self):
        self._info(f"── DETECT_BIN (carrying '{self._current_shape}') ──")
        bin_pose = self._stations.bin_pose(self._current_shape)
        ok = self._nav.go_to(bin_pose, f"Bin [{self._current_shape}]")
        if ok:
            self._bridge.publish_status(STATUS_AT_DESTINATION)
            self._sm.transition(MissionState.DROP_OBJECT)
        else:
            self._last_failed_leg = "bin"
            self._sm.transition(MissionState.BIN_NOT_FOUND)

    def _handle_drop_object(self):
        self._info(f"── DROP_OBJECT (shape '{self._current_shape}') ──")
        self._info("At bin. Waiting for arm to drop...")
        ok = self._bridge.wait_for_drop(ARM_TIMEOUT_SEC)
        if ok:
            self._sorted_count += 1
            self._info(f"Drop succeeded! Total sorted: {self._sorted_count}")
            self._current_shape = ""
            self._sm.transition(MissionState.CHECK_REMAINING)
        else:
            self._sm.transition(MissionState.DROP_FAILED)

    def _handle_check_remaining(self):
        self._info("── CHECK_REMAINING ──")
        self._spin_all(0.3)
        if self._bridge.source_empty:
            self._info(f"All shapes sorted! Total: {self._sorted_count}")
            self._sm.transition(MissionState.RETURN_TO_START)
        else:
            self._info("More shapes remain — going back to source.")
            self._sm.transition(MissionState.NAVIGATE_TO_SOURCE)

    def _handle_return_to_start(self):
        self._info("── RETURN_TO_START ──")
        self._bridge.publish_status(STATUS_RETURNING_HOME)
        ok = self._nav.go_to(self._stations.start_pose(), "Start Position")
        if ok:
            self._sm.transition(MissionState.TASK_COMPLETE)
        else:
            self._warn("Could not reach start exactly — mission still complete.")
            self._sm.force(MissionState.TASK_COMPLETE)

    def _handle_task_complete(self):
        self._info("╔══════════════════════════════════╗")
        self._info(f"║  MISSION COMPLETE                ║")
        self._info(f"║  {self._sorted_count} shape(s) sorted            ║")
        self._info("╚══════════════════════════════════╝")
        self._bridge.publish_status(STATUS_DONE)

    # ── Failure handlers ───────────────────────────────────────────────────────

    def _handle_navigation_failed(self):
        self._warn(f"── NAVIGATION_FAILED (leg: '{self._last_failed_leg}') ──")
        retries = self._sm.increment_retries()
        if retries <= MAX_NAV_RETRIES:
            self._warn(f"Retry {retries}/{MAX_NAV_RETRIES} in 2s...")
            time.sleep(2.0)
            self._sm.force(MissionState.RECOVERY)
        else:
            self._error(f"Navigation failed {MAX_NAV_RETRIES + 1} times — aborting.")
            self._sm.force(MissionState.ABORT)

    def _handle_object_not_found(self):
        self._warn("── OBJECT_NOT_FOUND ──")
        retries = self._sm.increment_retries()
        if retries <= MAX_DETECT_RETRIES:
            self._warn(f"Detection retry {retries}/{MAX_DETECT_RETRIES}...")
            time.sleep(1.5)
            self._sm.force(MissionState.NAVIGATE_TO_SOURCE)
        else:
            self._error("Detection failed too many times — aborting.")
            self._sm.force(MissionState.ABORT)

    def _handle_pick_failed(self):
        self._warn("── PICK_FAILED ──")
        retries = self._sm.increment_retries()
        if retries <= MAX_PICK_RETRIES:
            self._warn(f"Pick retry {retries}/{MAX_PICK_RETRIES}...")
            self._sm.force(MissionState.DETECT_OBJECT)
        else:
            self._error("Pick failed too many times — aborting.")
            self._sm.force(MissionState.ABORT)

    def _handle_bin_not_found(self):
        self._warn(f"── BIN_NOT_FOUND (shape '{self._current_shape}') ──")
        retries = self._sm.increment_retries()
        if retries <= 1:
            self._warn("Retrying bin approach...")
            self._sm.force(MissionState.NAVIGATE_TO_DESTINATION)
        else:
            self._error("Cannot reach bin — aborting.")
            self._sm.force(MissionState.ABORT)

    def _handle_drop_failed(self):
        self._warn("── DROP_FAILED ──")
        retries = self._sm.increment_retries()
        if retries <= MAX_DROP_RETRIES:
            self._warn(f"Drop retry {retries}/{MAX_DROP_RETRIES}...")
            self._bridge.publish_status(STATUS_AT_DESTINATION)
            self._sm.force(MissionState.DROP_OBJECT)
        else:
            self._error("Drop failed too many times — aborting.")
            self._sm.force(MissionState.ABORT)

    def _handle_recovery(self):
        self._warn(f"── RECOVERY (last leg: '{self._last_failed_leg}') ──")
        time.sleep(2.0)
        if self._last_failed_leg == "source":
            self._sm.force(MissionState.NAVIGATE_TO_SOURCE)
        elif self._last_failed_leg in ("destination", "bin"):
            self._sm.force(MissionState.NAVIGATE_TO_DESTINATION)
        else:
            self._sm.force(MissionState.NAVIGATE_TO_SOURCE)

    def _handle_abort(self):
        self._error("── ABORT ── Mission aborted.")
        self._bridge.publish_status(STATUS_ABORTED)
        self._info("Attempting emergency return to start...")
        self._nav.go_to(self._stations.start_pose(), "Start (abort)")

    # ── Dispatch table + main loop ─────────────────────────────────────────────

    _HANDLERS = {
        MissionState.INIT:                    "_handle_init",
        MissionState.SAVE_START_POSE:         "_handle_save_start_pose",
        MissionState.WAIT_FOR_NAV2:           "_handle_wait_for_nav2",
        MissionState.NAVIGATE_TO_SOURCE:      "_handle_navigate_to_source",
        MissionState.DETECT_OBJECT:           "_handle_detect_object",
        MissionState.PICK_OBJECT:             "_handle_detect_object",
        MissionState.NAVIGATE_TO_DESTINATION: "_handle_navigate_to_destination",
        MissionState.DETECT_BIN:              "_handle_detect_bin",
        MissionState.DROP_OBJECT:             "_handle_drop_object",
        MissionState.CHECK_REMAINING:         "_handle_check_remaining",
        MissionState.RETURN_TO_START:         "_handle_return_to_start",
        MissionState.TASK_COMPLETE:           "_handle_task_complete",
        MissionState.NAVIGATION_FAILED:       "_handle_navigation_failed",
        MissionState.OBJECT_NOT_FOUND:        "_handle_object_not_found",
        MissionState.PICK_FAILED:             "_handle_pick_failed",
        MissionState.BIN_NOT_FOUND:           "_handle_bin_not_found",
        MissionState.DROP_FAILED:             "_handle_drop_failed",
        MissionState.RECOVERY:                "_handle_recovery",
        MissionState.ABORT:                   "_handle_abort",
    }

    def run(self):
        self._info("╔══════════════════════════════════════════════╗")
        self._info("║  MIRTE Sorting Robot — Autonomous Mission    ║")
        self._info("║  heart | triangle | hexagon | l_shape        ║")
        self._info("╚══════════════════════════════════════════════╝")
        while rclpy.ok() and not self._sm.is_done():
            self._spin_all(0.05)
            handler = self._HANDLERS.get(self._sm.state)
            if handler:
                getattr(self, handler)()
            else:
                self._error(f"No handler for state: {self._sm.state.name}")
                break
        self._info(f"Mission ended. Final state: {self._sm.state.name}")


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted (Ctrl+C).")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
