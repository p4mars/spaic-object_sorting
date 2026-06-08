"""
station_manager.py
──────────────────
Loads all map-frame station/bin poses from station_locations.yaml ROS 2
parameters and provides them as PoseStamped objects ready for Nav2.

Usage:
    stations = StationManager(node)
    pose = stations.source_station()
    pose = stations.destination_station()
    pose = stations.bin_pose("heart")
    stations.save_start_pose(pose)
    pose = stations.start_pose()

Bin colours (from task image):
    heart    → yellow bin
    triangle → red bin
    hexagon  → green bin
    l_shape  → blue bin
    default  → fallback if class unknown
"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

SHAPE_CLASSES = ["heart", "triangle", "hexagon", "l_shape", "default"]


def _yaw_to_quat(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _build_pose(x: float, y: float, yaw: float, frame: str, clock) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = clock.now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    qz, qw = _yaw_to_quat(yaw)
    p.pose.orientation.x = 0.0
    p.pose.orientation.y = 0.0
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


class StationManager:
    def __init__(self, node: Node):
        self._node  = node
        self._clock = node.get_clock()
        self._log   = node.get_logger()

        node.declare_parameter("map_frame", "map")
        node.declare_parameter("source_station.x",   1.0)
        node.declare_parameter("source_station.y",   0.5)
        node.declare_parameter("source_station.yaw", 0.0)
        node.declare_parameter("destination_station.x",   3.0)
        node.declare_parameter("destination_station.y",   0.5)
        node.declare_parameter("destination_station.yaw", 1.57)
        node.declare_parameter("start_pose.x",   0.0)
        node.declare_parameter("start_pose.y",   0.0)
        node.declare_parameter("start_pose.yaw", 0.0)

        for cls in SHAPE_CLASSES:
            node.declare_parameter(f"bins.{cls}.x",   3.0)
            node.declare_parameter(f"bins.{cls}.y",   0.0)
            node.declare_parameter(f"bins.{cls}.yaw", 1.57)

        self._saved_start: Optional[PoseStamped] = None
        self._log.info("StationManager ready. Edit station_locations.yaml for real coordinates.")

    @property
    def _frame(self) -> str:
        return self._node.get_parameter("map_frame").value

    def source_station(self) -> PoseStamped:
        return _build_pose(
            self._node.get_parameter("source_station.x").value,
            self._node.get_parameter("source_station.y").value,
            self._node.get_parameter("source_station.yaw").value,
            self._frame, self._clock)

    def destination_station(self) -> PoseStamped:
        return _build_pose(
            self._node.get_parameter("destination_station.x").value,
            self._node.get_parameter("destination_station.y").value,
            self._node.get_parameter("destination_station.yaw").value,
            self._frame, self._clock)

    def bin_pose(self, shape_class: str) -> PoseStamped:
        key = shape_class if shape_class in SHAPE_CLASSES else "default"
        if key != shape_class:
            self._log.warn(
                f"Unknown class '{shape_class}' — using 'default' bin. "
                f"Known: {SHAPE_CLASSES}")
        return _build_pose(
            self._node.get_parameter(f"bins.{key}.x").value,
            self._node.get_parameter(f"bins.{key}.y").value,
            self._node.get_parameter(f"bins.{key}.yaw").value,
            self._frame, self._clock)

    def save_start_pose(self, pose: PoseStamped):
        self._saved_start = pose
        self._log.info(
            f"Start pose saved: x={pose.pose.position.x:.3f}  "
            f"y={pose.pose.position.y:.3f}")

    def start_pose(self) -> PoseStamped:
        if self._saved_start is not None:
            self._saved_start.header.stamp = self._clock.now().to_msg()
            return self._saved_start
        self._log.warn("No saved start pose — using YAML default (0,0,0).")
        return _build_pose(
            self._node.get_parameter("start_pose.x").value,
            self._node.get_parameter("start_pose.y").value,
            self._node.get_parameter("start_pose.yaw").value,
            self._frame, self._clock)

    def log_all(self):
        src = self.source_station()
        dst = self.destination_station()
        lines = [
            "══ Station poses ══",
            f"  source:      x={src.pose.position.x:.2f}  y={src.pose.position.y:.2f}",
            f"  destination: x={dst.pose.position.x:.2f}  y={dst.pose.position.y:.2f}",
            f"  start saved: {self._saved_start is not None}",
            "  bins:",
        ]
        for cls in SHAPE_CLASSES:
            bp = self.bin_pose(cls)
            lines.append(f"    {cls:10s}  x={bp.pose.position.x:.2f}  y={bp.pose.position.y:.2f}")
        lines.append("══════════════════")
        self._log.info("\n".join(lines))
