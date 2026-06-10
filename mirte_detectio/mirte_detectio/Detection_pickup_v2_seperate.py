import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface

from tf2_ros import Buffer, TransformListener
import rclpy.duration

import numpy as np
from ultralytics import YOLO
from pathlib import Path
import time
import threading

# load YOLO model
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "best.pt"
model = YOLO(str(MODEL_PATH))


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node_pickup")

        # activation check
        self.active = False
        self.grasp_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # MoveIt2 interfaces
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

        # subscribe to grasp pose (already in base_link)
        self.create_subscription(PoseStamped, "/arm_grasp_pose", self.grasp_pose_cb, 10)

        # subscribe to camera for detections
        self.create_subscription(Image, "/camera/color/image_raw", self.color_cb, 10)

        # subscribe to class
        self.create_subscription(String, "/detected_object/class", self.class_cb, 10)

        # subscribe to activation command
        self.create_subscription(String, "/activate_arm/pick", self.activate_cb, 10)

        # publisher to signal success
        self.done_pub = self.create_publisher(String, "/block_picked_up", 10)

        # publisher to update object class
        self.target_pub_class = self.create_publisher(String, "/detected_object/class", 10)

        # publisher to restart detection after pickup
        self.detect_pub = self.create_publisher(String, "/activate_detection", 10)

        # image size
        self.image_size = [640, 480]

        self.bridge = CvBridge()

        # confidence threshold for YOLO re-check
        self.conf_thres = 0.5

        self.get_logger().info("Pickup node ready")

        self.color = None
        self.class_name = "unknown"


    def color_cb(self, msg):
        # convert ROS2 image to cv2
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def class_cb(self, msg):
        # retrieve object class
        self.class_name = msg.data

    def grasp_pose_cb(self, msg):
        # Try transforming to base_link, fall back to using as-is
        try:
            # try map -> base_link first
            if msg.header.frame_id != "base_link":
                grasp_base = self.tf_buffer.transform(
                    msg,
                    "base_link",
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.grasp_pose = grasp_base
            else:
                self.grasp_pose = msg
            self.get_logger().info("Stored grasp pose")
        except Exception as e:
            self.get_logger().warn(f"TF failed, using pose as-is: {e}")
            # use the pose directly without transform
            self.grasp_pose = msg

    def activate_cb(self, msg):
        if msg.data == "start":
            self.active = True
            if self.grasp_pose is None:
                self.get_logger().warn("Activation received but no grasp pose yet")
                return
            # Run in separate thread so the ROS2 executor keeps spinning
            thread = threading.Thread(target=self.execute_grasp)
            thread.daemon = True
            thread.start()

        elif msg.data == "stop":
            self.active = False
            self.grasp_pose = None
            self.get_logger().info("pickup - Arm deactivated")


    def execute_grasp(self):
        # grasp pose is already in base_link
        gp = self.grasp_pose
        bx = gp.pose.position.x
        by = gp.pose.position.y
        bz = gp.pose.position.z

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

        # PRE-GRASP POSE (10 cm behind block)
        dist = math.sqrt(bx ** 2 + by ** 2)
        ux = bx / dist
        uy = by / dist

        pre = PoseStamped()
        pre.header.frame_id = "base_link"
        pre.header.stamp = self.get_clock().now().to_msg()
        pre.pose.position.x = bx - 0.10 * ux
        pre.pose.position.y = by - 0.10 * uy
        pre.pose.position.z = bz
        pre.pose.orientation = gp.pose.orientation

        # Move to pre-grasp
        traj = self.arm.plan(pose=pre)
        if traj is not None:
            self.arm.execute(traj)
        else:
            self.get_logger().warn("Pre-grasp planning failed")
            return

        # Move to grasp
        gp.header.stamp = self.get_clock().now().to_msg()
        traj = self.arm.plan(pose=gp)
        if traj is not None:
            self.arm.execute(traj)
        else:
            self.get_logger().warn("Grasp planning failed")
            return

        # Close gripper
        self.gripper.close()

        # LIFT BLOCK 15 cm
        lift = PoseStamped()
        lift.header.frame_id = "base_link"
        lift.header.stamp = self.get_clock().now().to_msg()
        lift.pose.position.x = bx
        lift.pose.position.y = by
        lift.pose.position.z = bz + 0.15
        lift.pose.orientation = gp.pose.orientation

        traj = self.arm.plan(pose=lift)
        self.get_logger().info(f"Planning result: {traj}")

        if traj is not None:
            self.get_logger().info("Executing pre-grasp...")
            self.arm.execute(traj)
        else:
            self.get_logger().warn("Lift planning failed")
            return

        # Publish success
        self.done_pub.publish(String(data="picked_up"))
        self.get_logger().info("Block picked up.")

        # Reset state
        self.grasp_pose = None
        self.active = False

        # Restart detection for next block
        restart = String()
        restart.data = "start"
        self.detect_pub.publish(restart)
        self.get_logger().info("[PICKUP] Detection restarted")


def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
