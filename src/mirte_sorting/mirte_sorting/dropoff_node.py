#!/usr/bin/env python3
"""
dropoff_node.py
───────────────
Arm control node for dropping off objects into the correct bin.

Activates when the mission planner publishes "AT_DESTINATION" on /nav/status.
The robot has already navigated to the correct bin before this is triggered.

Publishes:
  /drop_complete  (Bool True) — mission planner can proceed
  /drop_failed    (Bool True) — mission planner retries or aborts
"""

import rclpy
import time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String

from mirte_sorting.interfaces import (
    NAV_STATUS_TOPIC,
    STATUS_AT_DESTINATION,
    DROP_DONE_TOPIC,
    DROP_FAILED_TOPIC,
)

try:
    from pymoveit2 import MoveIt2, GripperInterface
    _MOVEIT_AVAILABLE = True
except ImportError:
    _MOVEIT_AVAILABLE = False


class DropoffNode(Node):
    def __init__(self):
        super().__init__("arm_node_dropoff")

        self.active = False

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


        else:
            self.get_logger().warn(
                "pymoveit2 not available — arm motion will be skipped. "
                "Install: pip install pymoveit2")
            self.arm = None
            self.gripper = None

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(String, NAV_STATUS_TOPIC, self._status_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.drop_done_pub = self.create_publisher(Bool, DROP_DONE_TOPIC, 10)
        self.drop_fail_pub = self.create_publisher(Bool, DROP_FAILED_TOPIC, 10)

        self.get_logger().info("DropoffNode ready — waiting for AT_DESTINATION.")

    # ── Callback ──────────────────────────────────────────────────────────

    def _status_cb(self, msg: String):
        if msg.data == STATUS_AT_DESTINATION and not self.active:
            self.active = True
            self.get_logger().info("Dropoff ACTIVATED — executing drop.")
            self._execute_drop()

    # ── Drop execution ─────────────────────────────────────────────────────

    def _execute_drop(self):
        # if self.arm is None:
        #     self.get_logger().warn("No arm interface — simulating drop.")
        #     self.drop_done_pub.publish(Bool(data=True))
        #     self.active = False
        #     return

        # try:
        #     # Extend arm to drop-off position above the bin
        #     # drop = PoseStamped()
        #     # drop.header.frame_id = "base_link"
        #     # drop.pose.position.x = 0.25
        #     # drop.pose.position.y = 0.0
        #     # drop.pose.position.z = 0.10
        #     # drop.pose.orientation.w = 1.0

        #     # self.arm.set_pose_goal(drop.pose)
        #     # self.arm.execute()

        #     # # Release object
        #     # self.gripper.open()

        #     # # Retract to neutral pose
        #     # retract = PoseStamped()
        #     # retract.header.frame_id = "base_link"
        #     # retract.pose.position.x = 0.10
        #     # retract.pose.position.y = 0.0
        #     # retract.pose.position.z = 0.10
        #     # retract.pose.orientation.w = 1.0

        #     # self.arm.set_pose_goal(retract.pose)
        #     # self.arm.execute()

        #     self.arm.move_to_pose(
        #         position=(0.25, 0.0, 0.10),
        #         quat_xyzw=(0.0,0.0,0.0,0.1),
        #         frame_id="base_link",
        #     )
        #     self.arm.wait_until_executed()
        #     self.gripper.open()

        #     self.arm.move_to_pose(
        #         position=(0.1, 0.0, 0.10),
        #         quat_xyzw=(0.0, 0.0, 0.0, 0.1),
        #         frame_id="base_link",
        #     )
        #     self.arm.wait_until_executed()


        #     self.drop_done_pub.publish(Bool(data=True))
        #     self.get_logger().info("Drop successful.")

        # except Exception as e:
        #     self.get_logger().error(f"Arm drop execution failed: {e}")
        #     self.drop_fail_pub.publish(Bool(data=True))

        # self.active = False

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



def main(args=None):
    rclpy.init(args=args)
    node = DropoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
