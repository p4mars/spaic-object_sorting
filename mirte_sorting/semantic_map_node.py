#!/usr/bin/env python3
"""
semantic_map_node.py
────────────────────
Builds a semantic map by watching AprilTag TF frames and averaging their
positions over a sliding window. Saves to YAML on the 'save_semantic_map'
service call.

The saved YAML lists stations and drop-boxes with their map-frame poses,
so the mission planner can load real coordinates after the mapping session.

Tag ID → name mapping is configured via the 'tag_names' parameter.
  Default: 1:station_1, 2:station_2, 10:dropbox_1 … 13:dropbox_4

Publishes: nothing (data is only saved to disk on service call)
Services:
  save_semantic_map  (std_srvs/Trigger) — writes maps/semantic_map.yaml
"""

import math
import os
from collections import defaultdict, deque

import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("save_path", "src/spaic-object_sorting/maps/semantic_map.yaml")
        self.declare_parameter("sample_window", 15)
        self.declare_parameter("time_interval", 0.5)
        self.declare_parameter("tag_names", [
            "1:station_1", "2:station_2",
            "10:dropbox_1", "11:dropbox_2", "12:dropbox_3", "13:dropbox_4",
        ])

        self._map_frame = self.get_parameter("map_frame").value
        self._save_path = self.get_parameter("save_path").value
        self._window = int(self.get_parameter("sample_window").value)
        self._interval = self.get_parameter("time_interval").value

        self._tag_to_name: dict[int, str] = {}
        for item in self.get_parameter("tag_names").value:
            tid, name = item.split(":", 1)
            self._tag_to_name[int(tid)] = name

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._samples: dict = defaultdict(lambda: deque(maxlen=self._window))

        self.create_timer(self._interval, self._collect)
        self.create_service(Trigger, "save_semantic_map", self._handle_save)
        self.get_logger().info(
            f"SemanticMapNode ready. Tags: {list(self._tag_to_name.values())}")

    def _collect(self):
        for tag_id, label in self._tag_to_name.items():
            child_frame = f"{label}_tag"
            try:
                tf = self._tf_buffer.lookup_transform(
                    self._map_frame, child_frame, rclpy.time.Time())
            except TransformException:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            self._samples[label].append({
                "tag_id": tag_id,
                "frame": child_frame,
                "x": float(t.x),
                "y": float(t.y),
                "z": float(t.z),          
                "qx": float(q.x),         
                "qy": float(q.y),
                "qz": float(q.z),
                "qw": float(q.w),
                "yaw": float(_yaw_from_quaternion(q)),
            })


    def _handle_save(self, _, response):
        payload = {"stations": {}, "drop_boxes": {}}
        for label, values in self._samples.items():
            if not values:
                continue
            n = len(values)
            x = sum(v["x"] for v in values) / n
            y = sum(v["y"] for v in values) / n
            yaw = sum(v["yaw"] for v in values) / n
            entry = {
                "tag_id": values[-1]["tag_id"],
                "frame": values[-1]["frame"],
                "pose": {"x": x, "y": y, "yaw": yaw},
                "samples": n,
            }
            if label.startswith("station"):
                payload["stations"][label] = entry
            else:
                payload["drop_boxes"][label] = entry

        os.makedirs(os.path.dirname(os.path.abspath(self._save_path)), exist_ok=True)
        with open(self._save_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, sort_keys=True)

        response.success = True
        response.message = f"Semantic map saved to {self._save_path}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
