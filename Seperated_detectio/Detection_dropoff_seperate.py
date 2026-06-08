import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

from moveit2 import MoveIt2
from moveit2 import GripperInterface


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node_dropoff")

        # activation check
        self.active = False

        # MoveIt2 interfaces
        self.arm = MoveIt2(
            node=self,
            joint_names=["shoulder_pan", "shoulder_lift", "elbow", "wrist"],
            base_link="base_link",
            end_effector_link="gripper_link"
        )

        self.gripper = GripperInterface(self)

        # subscribe to activation command
        self.create_subscription(String, "/activate_arm/drop", self.activate_cb, 10)

        # publisher to signal success
        self.done_pub = self.create_publisher(String, "/block_dropped_off", 10)

    def activate_cb(self, msg):
        if msg.data == "start":
            self.active = True
            self.execute_drop()

        elif msg.data == "stop":
            self.active = False
            self.grasp_pose = None
            self.get_logger().info("dropoff - Arm deactivated")

    def execute_drop(self):
        # Move forward to drop-off position
        drop = PoseStamped()
        drop.header.frame_id = "base_link"
        drop.pose.position.x = 0.25   # 25 cm forward
        drop.pose.position.y = 0.0
        drop.pose.position.z = 0.05   # adjust height if needed
        drop.pose.orientation.w = 1.0

        self.arm.set_pose_goal(drop.pose)
        self.arm.execute()

        # Open gripper to release block
        self.gripper.open()

        # Retract arm back to neutral
        retract = PoseStamped()
        retract.header.frame_id = "base_link"
        retract.pose.position.x = 0.10
        retract.pose.position.y = 0.0
        retract.pose.position.z = 0.10
        retract.pose.orientation.w = 1.0

        self.arm.set_pose_goal(retract.pose)
        self.arm.execute()

        # Publish success
        self.done_pub.publish(String(data="dropped_off"))
        self.get_logger().info("Block dropped off.")

        # Reset state
        self.active = False


def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
