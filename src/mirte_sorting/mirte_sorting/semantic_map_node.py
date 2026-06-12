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
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quaternion(q) -> float:
    """Extract planar yaw (rotation around Z) from a quaternion.
    This uses the standard quaternion->Euler formula and returns radians
    in [-pi, pi], which is what we store in the semantic map.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SemanticMapNode(Node):
    def __init__(self):
        super().__init__("semantic_map_node")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("save_path", "")
        self.declare_parameter("sample_window", 10)
        self.declare_parameter("time_interval", 0.2)
        self.declare_parameter("tag_names", [
            "1:station_1", "2:station_2",
            "10:dropbox_1", "11:dropbox_2", "12:dropbox_3", "13:dropbox_4",
        ])

        self._map_frame = self.get_parameter("map_frame").value
        save_path = self.get_parameter("save_path").value
        if not save_path:
            save_path = os.path.join(
                get_package_share_directory("mirte_sorting"), "maps", "semantic_map.yaml")
        self._save_path = save_path
        self._window = int(self.get_parameter("sample_window").value)
        self._interval = self.get_parameter("time_interval").value

        self._tag_to_name: dict[int, str] = {}
        for item in self.get_parameter("tag_names").value:
            tid, name = item.split(":", 1)
            self._tag_to_name[int(tid)] = name

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._samples: dict = defaultdict(lambda: deque(maxlen=self._window))

        # Create a empty dict to trach the tags that have been published
        self._published_tags: dict[str, dict] = {}
        # Publisher for the visualized markers
        self._marker_pub = self.create_publisher(MarkerArray, "semantic_map_markers", 10)

        # Start the periodic collection of tag poses
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

            # Check if the frame have enough samples and have not been published yet
            samples = self._samples[label]
            if label not in self._published_tags and len(samples) >= self._window:
                n = len(samples)
                avg_x   = sum(s["x"]  for s in samples) / n
                avg_y   = sum(s["y"]  for s in samples) / n
                avg_z   = sum(s["z"]  for s in samples) / n
                avg_yaw = sum(s["yaw"] for s in samples) / n
                avg_qx  = sum(s["qx"] for s in samples) / n
                avg_qy  = sum(s["qy"] for s in samples) / n
                avg_qz  = sum(s["qz"] for s in samples) / n
                avg_qw  = sum(s["qw"] for s in samples) / n
                norm = math.sqrt(avg_qx**2 + avg_qy**2 + avg_qz**2 + avg_qw**2)
                avg_qx /= norm
                avg_qy /= norm
                avg_qz /= norm
                avg_qw /= norm
                # Project the tag's local Z axis onto the map XY plane to get a 2D arrow yaw
                zx = 2.0 * (avg_qx * avg_qz + avg_qy * avg_qw)
                zy = 2.0 * (avg_qy * avg_qz - avg_qx * avg_qw)
                z_yaw = math.atan2(zy, zx)
                self._publish_marker(tag_id, label, avg_x, avg_y, avg_z, z_yaw)
                self._published_tags[label] = {
                    "tag_id": tag_id,
                    "frame": child_frame,
                    "pose": {"x": avg_x, "y": avg_y, "yaw": avg_yaw},
                    "samples": n,
                }
                self.get_logger().info(f"Tag '{label}' locked in from {n} samples.")


    def _publish_marker(self, tag_id, label, x, y, z, yaw):
        """Publish a visualization marker for the given tag pose.
        The markers are published in the map frame,
        rotation is shown with arrows and only in the planar yaw angle, 
        and the label is shown as text above the tag."""

        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        # Add an arrow marker to show the orientation of the tag
        arrow = Marker()
        arrow.header.frame_id = self._map_frame
        arrow.header.stamp = stamp
        arrow.ns = "semantic_map"
        arrow.id = tag_id
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = z
        arrow.pose.orientation.x = 0.0
        arrow.pose.orientation.y = 0.0
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x = 0.3   # shaft length
        arrow.scale.y = 0.05  # shaft diameter
        arrow.scale.z = 0.08  # head diameter
        arrow.color.r = 0.0
        arrow.color.g = 0.8
        arrow.color.b = 0.2
        arrow.color.a = 0.9
        arrow.lifetime.sec = 0
        arrow.lifetime.nanosec = 0
        array.markers.append(arrow)

        # Add a text marker above the cube to show the label
        text = Marker()
        text.header.frame_id = self._map_frame
        text.header.stamp = stamp
        text.ns = "semantic_map_labels"
        text.id = tag_id
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = z + 0.15
        text.pose.orientation.w = 1.0
        text.scale.z = 0.08
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a = 1.0
        text.text = label
        text.lifetime.sec = 0
        text.lifetime.nanosec = 0
        array.markers.append(text)

        self.get_logger().info(
            f"publish marker. Tag: {label}, x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, yaw: {math.degrees(yaw):.1f}°")
        self._marker_pub.publish(array)

    def _handle_save(self, _, response):
        payload = {"stations": {}, "drop_boxes": {}}
        for label, entry in self._published_tags.items():
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
