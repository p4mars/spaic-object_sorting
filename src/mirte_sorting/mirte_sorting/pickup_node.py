#!/usr/bin/env python3
"""
pickup_node.py
──────────────
Arm control node for picking up objects.

Activates when the mission planner publishes "AT_SOURCE" on /nav/status
AND a grasp pose has been received from the perception node.

After a successful pick:
  - publishes /pick_complete (Bool True) → mission planner proceeds
  - refines object class from close-up camera view and publishes /object_class

Subscribes:
  /nav/status         — activates on "AT_SOURCE"
  /arm_grasp_pose     — grasp target from perception node
  /camera/color/image_raw

Publishes:
  /pick_complete      (Bool)   — True on success
  /pick_failed        (Bool)   — True on failure
  /object_class       (String) — refined class name after close-up view
"""

import math
from pathlib import Path
import time

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from mirte_sorting.interfaces import (
    NAV_STATUS_TOPIC,
    STATUS_AT_SOURCE,
    PICK_DONE_TOPIC,
    PICK_FAILED_TOPIC,
    OBJ_CLASS_TOPIC,
)

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False

try:
    from pymoveit2 import MoveIt2, GripperInterface
    _MOVEIT_AVAILABLE = True
except ImportError:
    _MOVEIT_AVAILABLE = False


class PickupNode(Node):
    def __init__(self):
        super().__init__("arm_node_pickup")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf_threshold", 0.5)

        model_path = self.get_parameter("model_path").value
        self.conf_thres = self.get_parameter("conf_threshold").value

        # ── State ─────────────────────────────────────────────────────────
        self.active = False
        self.grasp_pose: PoseStamped | None = None
        self.current_class: str = ""
        self.color = None
        self.bridge = CvBridge()
        self.image_size = [640, 480]

        # ── YOLO model (for close-up class refinement) ─────────────────────
        self._model = None
        if _YOLO_AVAILABLE:
            resolved = Path(model_path) if model_path else Path(__file__).parent / "best.pt"
            if resolved.exists():
                self._model = YOLO(str(resolved))

        # ── MoveIt2 arm interface ──────────────────────────────────────────
        if _MOVEIT_AVAILABLE:
            # self.arm = MoveIt2(
            #     node=self,
            #     joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"],
            #     base_link_name="base_link",
            #     end_effector_name="wrist",
            #     group_name="arm"
            # )

            # self.gripper = GripperInterface(
            #     node=self,
            #     gripper_joint_names=["gripper_joint"],
            #     open_gripper_joint_positions=[0.04],
            #     closed_gripper_joint_positions=[0.0],
            #     gripper_group_name="gripper"
            # )
            self.arm = MoveIt2(
                node=self,
                joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"],
                base_link_name="base_link",
                end_effector_name="wrist",
                group_name="mirte_arm",
                use_move_group_action=True,
            )

            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["gripper_joint"],
                open_gripper_joint_positions=[0.2102],
                closed_gripper_joint_positions=[-0.2],
                gripper_command_action_name="/mirte_master_gripper_controller/gripper_cmd",
            )


        else:
            self.get_logger().warn(
                "pymoveit2 not available — arm motion will be skipped. "
                "Install: pip install pymoveit2")
            self.arm = None
            self.gripper = None

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(String, NAV_STATUS_TOPIC, self._status_cb, 10)
        self.create_subscription(PoseStamped, "/arm_grasp_pose", self._grasp_pose_cb, 10)
        self.create_subscription(String, OBJ_CLASS_TOPIC, self._class_cb, 10)
        self.create_subscription(Image, "/camera/color/image_raw", self._color_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.pick_done_pub = self.create_publisher(Bool, PICK_DONE_TOPIC, 10)
        self.pick_fail_pub = self.create_publisher(Bool, PICK_FAILED_TOPIC, 10)
        self.class_pub = self.create_publisher(String, OBJ_CLASS_TOPIC, 10)

        self.get_logger().info("PickupNode ready — waiting for AT_SOURCE.")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        if msg.data == STATUS_AT_SOURCE:
            if self.grasp_pose is not None and not self.active:
                self.active = True
                self.get_logger().info("Pickup ACTIVATED — executing grasp.")
                self._execute_grasp()
            elif self.grasp_pose is None:
                self.get_logger().warn(
                    "AT_SOURCE received but no grasp pose yet — "
                    "waiting for perception node.")

    def _grasp_pose_cb(self, msg: PoseStamped):
        self.grasp_pose = msg
        self.get_logger().info(
            f"Grasp pose stored: "
            f"x={msg.pose.position.x:.3f} y={msg.pose.position.y:.3f}")

    def _class_cb(self, msg: String):
        self.current_class = msg.data.strip().lower()

    def _color_cb(self, msg: Image):
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    # ── Grasp execution ───────────────────────────────────────────────────

    def _execute_grasp(self):
        if self.arm is None:
            self.get_logger().warn("No arm interface — simulating pick.")
            self._publish_pick_done()
            return

        # Transform grasp pose from map to base_link
        try:
            grasp_base = self.tf_buffer.transform(
                self.grasp_pose,
                "base_link",
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().error(f"TF map→base_link failed: {e}")
            self.pick_fail_pub.publish(Bool(data=True))
            self.active = False
            return

        bx = grasp_base.pose.position.x
        by = grasp_base.pose.position.y
        bz = grasp_base.pose.position.z

        if self.color is None:
            self.get_logger().warn("No camera frame received yet, skipping YOLO re-check")

        else:
            # YOLO re-check to confirm class closest to center
            results = model(self.color)
            detections = []

            for result in results:
                boxes = result.boxes
                for i in range(len(boxes)):

                    cx = int(boxes.xywh[i][0].item())
                    cy = int(boxes.xywh[i][1].item())

                    class_id = int(boxes.cls[i].item())
                    class_name = result.names[class_id]
                    conf = float(boxes.conf[i].item())

                    if conf <= self.conf_thres:
                        continue

                    detections.append({
                        "class": class_name,
                        "pixel": (cx, cy)
                    })

            if len(detections) > 0:
                class_candidate = []
                for det in detections:
                    px, py = det["pixel"]
                    # distance from image center
                    d_x = abs(px - self.image_size[0] / 2)
                    class_candidate.append((d_x, det["class"]))

                class_candidate.sort()
                new_class = class_candidate[0][1]

                if new_class != self.class_name:
                    self.target_pub_class.publish(String(data=new_class))
                    self.class_name = new_class

        # Pre-grasp: 10 cm behind the object
        self.get_logger().info("Waiting for MoveIt to be ready...")
        time.sleep(5.0)
        self.get_logger().info("Attempting pre-grasp planning...")

        # Move to home first
        self.get_logger().info("Moving to home position...")
        traj = self.arm.plan(
            joint_positions=[0.0, 0.0, 0.0, 0.0],
            joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"]
        )
        if traj is not None:
            self.get_logger().info("Home plan succeeded, executing...")
            self.arm.execute(traj)
            time.sleep(2.0)
        else:
            self.get_logger().warn("Home planning failed")
            return

        dist = math.sqrt(bx ** 2 + by ** 2)
        if dist > 1e-6:
            ux, uy = bx / dist, by / dist
        else:
            ux, uy = 1.0, 0.0

        pre = PoseStamped()
        pre.header.frame_id = "base_link"
        pre.pose.position.x = bx - 0.10 * ux
        pre.pose.position.y = by - 0.10 * uy
        pre.pose.position.z = bz
        pre.pose.orientation = grasp_base.pose.orientation

        try:
            # Move to pre-grasp
            traj = self.arm.plan(pose=pre)
            if traj is not None:
                self.arm.execute(traj)
            else:
                self.get_logger().warn("Pre-grasp planning failed")
                return

            # Move to exact grasp position
            traj = self.arm.plan(pose=grasp_base.pose)
            if traj is not None:
                self.arm.execute(traj)
            else:
                self.get_logger().warn("grasp planning failed")
                return

            # Close gripper
            self.gripper.close()

            # Lift 15 cm
            lift = PoseStamped()
            lift.header.frame_id = "base_link"
            lift.pose.position.x = bx
            lift.pose.position.y = by
            lift.pose.position.z = bz + 0.15
            lift.pose.orientation = grasp_base.pose.orientation

            traj = self.arm.plan(pose=lift.pose)
            if traj is not None:
                self.arm.execute(traj)
            else:
                self.get_logger().warn("lift planning failed")
                return

            self.get_logger().info("Pick successful.")
            self._publish_pick_done()

        except Exception as e:
            self.get_logger().error(f"Arm execution failed: {e}")
            self.pick_fail_pub.publish(Bool(data=True))

        self.grasp_pose = None
        self.active = False

    def _refine_class_from_camera(self):
        """Run YOLO on current frame to refine object class if possible."""
        if self._model is None or self.color is None:
            return
        try:
            results = self._model(self.color)
            candidates = []
            W = self.image_size[0]
            for result in results:
                for i in range(len(result.boxes)):
                    conf = float(result.boxes.conf[i].item())
                    if conf < self.conf_thres:
                        continue
                    cx = int(result.boxes.xywh[i][0].item())
                    d_center = abs(cx - W // 2)
                    class_name = result.names[int(result.boxes.cls[i].item())]
                    candidates.append((d_center, class_name))
            if candidates:
                candidates.sort()
                refined = candidates[0][1]
                if refined != self.current_class:
                    self.get_logger().info(
                        f"Class refined from '{self.current_class}' to '{refined}'")
                    self.current_class = refined
        except Exception as e:
            self.get_logger().warn(f"Class refinement failed: {e}")

    def _publish_pick_done(self):
        # Publish final class before signalling pick complete
        if self.current_class:
            self.class_pub.publish(String(data=self.current_class))
        self.pick_done_pub.publish(Bool(data=True))
        self.get_logger().info(f"Pick complete — class='{self.current_class}'")


def main(args=None):
    rclpy.init(args=args)
    node = PickupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
