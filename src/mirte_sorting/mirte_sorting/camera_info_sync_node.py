#!/usr/bin/env python3
"""
camera_info_sync_node.py
────────────────────────
Synchronises camera_info timestamps with image timestamps so AprilTag
detection receives consistently stamped data.

Subscribes:
  /camera/color/image_raw       — raw RGB image
  /camera/color/camera_info     — camera calibration

Publishes:
  /apriltag/image_rect          — image with timestamp matched to camera_info
  /apriltag/camera_info         — camera_info with timestamp matched to image
"""

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class CameraInfoSyncNode(Node):
    def __init__(self):
        super().__init__("camera_info_sync_node")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("synced_image_topic", "/apriltag/image_rect")
        self.declare_parameter("synced_camera_info_topic", "/apriltag/camera_info")
        self.declare_parameter("use_image_frame_id", True)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        synced_image_topic = self.get_parameter("synced_image_topic").value
        synced_camera_info_topic = self.get_parameter("synced_camera_info_topic").value
        self._use_image_frame_id = self.get_parameter("use_image_frame_id").value

        # Cache the latest camera_info; on each image we stamp the cached
        # info to match — not exact-time sync, but sufficient for a static camera.
        self._latest_info = None

        # RELIABLE: don't drop messages (vs BEST_EFFORT which can skip frames)
        # KEEP_LAST/depth=10: only buffer the 10 most recent messages
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(CameraInfo, camera_info_topic, self._info_cb, qos)
        self.create_subscription(Image, image_topic, self._image_cb, qos)

        self._image_pub = self.create_publisher(Image, synced_image_topic, qos)
        self._info_pub = self.create_publisher(CameraInfo, synced_camera_info_topic, qos)

        self.get_logger().info(
            f"CameraInfoSyncNode: {image_topic} + {camera_info_topic} "
            f"→ {synced_image_topic}")

    def _info_cb(self, msg: CameraInfo):
        # Just cache the latest info; images drive the pipeline
        self._latest_info = msg

    def _image_cb(self, image_msg: Image):
        if self._latest_info is None:
            self.get_logger().warn_throttle(
                self.get_clock(), 5000,
                "No camera_info received yet — cannot publish synced topics.")
            return

        synced_image = copy.deepcopy(image_msg)
        # deepcopy so we don't mutate the cached _latest_info when we re-stamp it
        synced_info = copy.deepcopy(self._latest_info)
        synced_info.header.stamp = synced_image.header.stamp
        if self._use_image_frame_id:
            # If True, overwrite camera_info's frame_id with the image's frame_id.
            # Set to False only if your camera driver publishes them under different
            # frame IDs intentionally.
            synced_info.header.frame_id = synced_image.header.frame_id

        self._image_pub.publish(synced_image)
        self._info_pub.publish(synced_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
