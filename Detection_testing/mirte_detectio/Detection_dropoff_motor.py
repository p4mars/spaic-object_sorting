import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import math


class DropoffMotorNode(Node):
    def __init__(self):
        super().__init__("dropoff_motor_node")

        # subscribe to dropoff command
        self.create_subscription(String, "/activate_dropoff_move", self.cb, 10)

        # publisher for robot movement
        self.cmd_pub = self.create_publisher(Twist, "/mirte_base_controller/cmd_vel", 10)

        # publisher to activate dropoff arm
        self.drop_pub = self.create_publisher(String, "/activate_arm/drop", 10)

        self.create_subscription(String, "/block_dropped_off", self.dropped_cb, 10)
        self.detect_pub = self.create_publisher(String, "/activate_detection", 10)

        self.get_logger().info("Dropoff motor node ready")

    def dropped_cb(self, msg):
        if msg.data == "dropped_off":
            out = String()
            out.data = "start"
            self.detect_pub.publish(out)
            self.get_logger().info("[DROPOFF] Detection restarted")

    def cb(self, msg):
        if msg.data != "start":
            return

        self.get_logger().info("[DROPOFF] Moving to drop zone")

        # Rotate robot 90 degrees
        twist = Twist()
        twist.angular.z = 0.4   # angular rate
        t_end = time.time() + (math.pi / (0.4*2))

        while time.time() < t_end:
            self.cmd_pub.publish(twist)

        self.stop()

        # Drive forward 40 cm
        twist = Twist()
        twist.linear.x = 0.15   # forward speed
        t_end = time.time() + (0.40 / 0.15)

        while time.time() < t_end:
            self.cmd_pub.publish(twist)

        self.stop()

        # Trigger dropoff arm
        msg = String()
        msg.data = "start"
        self.drop_pub.publish(msg)

        self.get_logger().info("[DROPOFF] Arm dropoff triggered")


    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        time.sleep(0.1)


def main():
    rclpy.init()
    node = DropoffMotorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
