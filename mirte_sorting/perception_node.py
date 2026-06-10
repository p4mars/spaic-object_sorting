#!/usr/bin/env python3
"""
perception_node.py
──────────────────
YOLO-based object detection node for the MIRTE sorting robot.

Activates when the mission planner publishes "AT_SOURCE" on /nav/status.
Detects objects in the camera image, locates them in 3D using the depth
camera, transforms the position to the map frame, and publishes:
  /arm_grasp_pose     (PoseStamped) — exact grasp point for the arm node
  /object_class       (String)      — YOLO class name for the mission planner
  /source_empty       (Bool)        — True when no more objects are found

Subscribes:
  /nav/status                       — activates/deactivates on AT_SOURCE
  /camera/color/image_raw           — RGB camera frames
  /camera/depth/image_raw           — aligned depth frames
  /robot_pose  (PoseWithCovarianceStamped) — fused robot position
"""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener

from mirte_sorting import detection_functions as DF
from mirte_sorting.interfaces import NAV_STATUS_TOPIC, STATUS_AT_SOURCE

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("n_frames", 10)
        self.declare_parameter("block_stop_dist", 0.20)

        model_path = self.get_parameter("model_path").value
        self.conf_thres = self.get_parameter("conf_threshold").value
        self.N_FRAMES = self.get_parameter("n_frames").value
        self.block_stop = self.get_parameter("block_stop_dist").value

        # Load YOLO model
        self._model = None
        if _YOLO_AVAILABLE:
            resolved = Path(model_path) if model_path else Path(__file__).parent / "best.pt"
            if resolved.exists():
                self._model = YOLO(str(resolved))
                self.get_logger().info(f"YOLO model loaded: {resolved}")
            else:
                self.get_logger().error(
                    f"YOLO model not found at {resolved}. "
                    "Set the 'model_path' parameter to the correct .pt file path.")
        else:
            self.get_logger().error("ultralytics not installed — detection disabled.")

        # ── State ─────────────────────────────────────────────────────────
        self.active = False
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.frame_buffer = []
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Search / sweep state
        self.search_angle = 0
        self.search_direction = -1
        self.search_limit = 40
        self.search_step = 21
        self.flipdir = 0

        # ── Camera intrinsics (approximate from FOV) ───────────────────────
        self.image_size = [640, 480]
        self.depth_fov = [58.4, 45.5]
        self.color_fov = [66.1, 40.2]

        W, H = self.image_size
        self.fx_d = (W / 2.0) / np.tan(np.deg2rad(self.depth_fov[0] / 2.0))
        self.fy_d = (H / 2.0) / np.tan(np.deg2rad(self.depth_fov[1] / 2.0))
        self.cx_d = W / 2.0
        self.cy_d = H / 2.0

        self.fx_color = (W / 2.0) / np.tan(np.deg2rad(self.color_fov[0] / 2.0))
        self.fy_color = (H / 2.0) / np.tan(np.deg2rad(self.color_fov[1] / 2.0))
        self.cx_color = W / 2.0
        self.cy_color = H / 2.0

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(String, NAV_STATUS_TOPIC, self._status_cb, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self._color_cb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self._depth_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/robot_pose", self._pose_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/mirte_base_controller/cmd_vel", 10)
        self.arm_grasp_pub = self.create_publisher(PoseStamped, "/arm_grasp_pose", 10)
        self.target_pos_pub = self.create_publisher(PoseStamped, "/detected_object/pos", 10)
        self.object_class_pub = self.create_publisher(String, "/object_class", 10)
        self.source_empty_pub = self.create_publisher(Bool, "/source_empty", 10)

        self.create_timer(0.1, self._loop)
        self.get_logger().info("PerceptionNode ready — waiting for AT_SOURCE.")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        if msg.data == STATUS_AT_SOURCE:
            if not self.active:
                self.active = True
                self._reset_search()
                self.frame_buffer = []
                self.get_logger().info("Detection ACTIVATED (AT_SOURCE received).")
        else:
            if self.active:
                self.active = False
                self.get_logger().info(f"Detection DEACTIVATED ({msg.data}).")

    def _color_cb(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _reset_search(self):
        self.search_angle = 0
        self.search_direction = -1
        self.flipdir = 0

    def _color_to_depth_pixel(self, u_c, v_c):
        x_c = (u_c - self.cx_color) / self.fx_color
        y_c = (v_c - self.cy_color) / self.fy_color
        u_d = int(self.fx_d * x_c + self.cx_d)
        v_d = int(self.fy_d * y_c + self.cy_d)
        return u_d, v_d

    def _yaw_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def _next_search_angle(self):
        self.search_angle += self.search_direction * self.search_step
        if abs(self.search_angle) > self.search_limit:
            self.search_direction *= -1
            self.flipdir += 0
            self.search_angle = self.search_direction * self.search_step

        if self.flipdir >= 2:
            self.get_logger().info("Search exhausted — publishing source_empty.")
            self.source_empty_pub.publish(Bool(data=True))
            self.flipdir = 0
            self.active = False
            return None
        return self.search_angle

    def _rotate_robot(self, angle_deg):
        ANGULAR_SPEED = math.radians(10)
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / ANGULAR_SPEED
        twist = Twist()
        twist.angular.z = (1.0 if angle_rad > 0 else -1.0) * ANGULAR_SPEED
        t0 = time.time()
        while time.time() - t0 < duration:
            self.cmd_pub.publish(twist)
        self.cmd_pub.publish(Twist())

    # ── Main detection loop ───────────────────────────────────────────────

    def _loop(self):
        if not self.active or self._model is None:
            return
        if self.color is None or self.depth is None:
            return

        self.frame_buffer.append((self.color.copy(), self.depth.copy()))
        if len(self.frame_buffer) < self.N_FRAMES:
            return

        detections = []
        H, W = self.image_size[1], self.image_size[0]

        for color, depth in self.frame_buffer:
            results = self._model(color)
            for result in results:
                boxes = result.boxes
                for i in range(len(boxes)):
                    cx = int(boxes.xywh[i][0].item())
                    cy = int(boxes.xywh[i][1].item())
                    half_w = max(1, int(boxes.xywh[i][2].item() / 2))

                    # # Sample depth across horizontal slice of bounding box
                    # z_list = []
                    # for px in range(max(0, cx - half_w), min(W, cx + half_w)):
                    #     if 0 < cy < H:
                    #         d_val = depth[cy, px]
                    #         if d_val > 0:
                    #             z_list.append((d_val, px))
                    radius = 20
                    z_list = []

                    for py in range(max(0, cy - radius), min(H, cy + radius)):
                        for px in range(max(0, cx - radius), min(W, cx + radius)):
                            d_val = depth[py, px]
                            if d_val > 0:   # ignore invalid depth
                                z_list.append((d_val, px, py))

                    if not z_list:
                        continue

                    z_list.sort()

                    z_mm, best_px = z_list[len(z_list) // 2]
                    z = float(z_mm) / 1000.0

                    x = (best_px - self.cx_d) * (z / self.fx_d)
                    y = (cy - self.cy_d) * (z / self.fy_d)

                    class_id = int(boxes.cls[i].item())
                    class_name = result.names[class_id]
                    conf = float(boxes.conf[i].item())

                    if conf < self.conf_thres or z <= 0:
                        continue

                    detections.append({
                        "pos": [x, y, z],
                        "class": class_name,
                        "conf": conf,
                        "pixel": (cx, cy),
                    })

        self.frame_buffer = []
        self.get_logger().info(f"Detection buffer size: {len(self.frame_buffer)}")

        clusters = DF.cluster_detections(detections, radius=10)
        fused = DF.fuse_clusters(clusters)
        target = DF.pick_best(fused)

        if target is None:
            angle = self._next_search_angle()
            if angle is not None:
                self.get_logger().info(f"No detections — sweeping {angle}°")
                self._rotate_robot(angle)
            return

        # Detection found — reset search state
        self._reset_search()

        x, y, z = target["pos"]
        class_name = target["class"]

        # Transform from camera depth frame to map frame
        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_depth_optical_frame"
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = z

        try:
            point_map = self.tf_buffer.transform(
                point_cam, "map",
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f"TF camera→map failed: {e}")
            return

        bx, by, bz = point_map.point.x, point_map.point.y, point_map.point.z

        rx, ry = self.current_x, self.current_y
        dx, dy = bx - rx, by - ry
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist < 1e-6:
            return

        # Navigation waypoint: stop block_stop metres before the object
        ux, uy = dx / dist, dy / dist
        wx = bx - self.block_stop * ux
        wy = by - self.block_stop * uy
        yaw = math.atan2(by - ry, bx - rx)

        # Publish navigation goal
        nav_pose = PoseStamped()
        nav_pose.header.stamp = self.get_clock().now().to_msg()
        nav_pose.header.frame_id = "map"
        nav_pose.pose.position.x = wx
        nav_pose.pose.position.y = wy
        nav_pose.pose.position.z = 0.0
        nav_pose.pose.orientation = self._yaw_to_quaternion(yaw)
        self.target_pos_pub.publish(nav_pose)

        # Publish grasp pose (exact object location)
        grasp = PoseStamped()
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.header.frame_id = "map"
        grasp.pose.position.x = bx
        grasp.pose.position.y = by
        grasp.pose.position.z = bz
        grasp.pose.orientation = self._yaw_to_quaternion(yaw)
        self.arm_grasp_pub.publish(grasp)

        # Publish object class for mission planner
        self.object_class_pub.publish(String(data=class_name))

        self.get_logger().info(
            f"[DETECT] class={class_name}  "
            f"map=({bx:.3f},{by:.3f},{bz:.3f})  "
            f"waypoint=({wx:.3f},{wy:.3f})")

        # Deactivate until next AT_SOURCE
        self.active = False


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
