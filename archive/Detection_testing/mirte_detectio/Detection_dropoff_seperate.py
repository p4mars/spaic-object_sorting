import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

from pymoveit2 import MoveIt2
from pymoveit2 import GripperInterface
import time
import threading


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node_dropoff")

        # activation check
        self.active = False

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

        # subscribe to activation command
        self.create_subscription(String, "/activate_arm/drop", self.activate_cb, 10)

        # publisher to signal success
        self.done_pub = self.create_publisher(String, "/block_dropped_off", 10)

    def activate_cb(self, msg):
        if msg.data == "start":
            self.active = True
            thread = threading.Thread(target=self.execute_drop)
            thread.daemon = True
            thread.start()
        elif msg.data == "stop":
            self.active = False
            self.get_logger().info("dropoff - Arm deactivated")

    def execute_drop(self):
        self.get_logger().info("Waiting for MoveIt to be ready...")
        time.sleep(5.0)
        self.get_logger().info("Attempting pre-drop planning...")
        # Move forward to drop-off position
        drop = PoseStamped()
        drop.header.frame_id = "base_link"
        drop.header.stamp = self.get_clock().now().to_msg()
        drop.pose.position.x = 0.25  # 25 cm forward
        drop.pose.position.y = 0.0
        drop.pose.position.z = 0.05  # adjust height if needed
        drop.pose.orientation.w = 1.0

        traj = self.arm.plan(pose=drop)
        self.get_logger().info(f"Planning result: {traj}")
        if traj is not None:
            self.get_logger().info("Executing pre-drop...")
            self.arm.execute(traj)
        else:
            self.get_logger().warn("Drop planning failed")
            return

        # Open gripper to release block
        self.gripper.open()

        # Retract arm back to neutral
        retract = PoseStamped()
        retract.header.frame_id = "base_link"
        retract.header.stamp = self.get_clock().now().to_msg()
        retract.pose.position.x = 0.10
        retract.pose.position.y = 0.0
        retract.pose.position.z = 0.10
        retract.pose.orientation.w = 1.0

        traj = self.arm.plan(pose=retract)
        if traj is not None:
            self.arm.execute(traj)
        else:
            self.get_logger().warn("Retract planning failed")
            return

        # Publish success
        self.done_pub.publish(String(data="dropped_off"))
        self.get_logger().info("Block dropped off.")

        # Reset state
        self.active = False
        time.sleep(2.0)


def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
