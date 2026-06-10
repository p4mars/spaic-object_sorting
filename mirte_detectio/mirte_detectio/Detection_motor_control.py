import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import math
import time


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        # Subscribe to block target (base_link frame, relative to robot)
        self.create_subscription(
            PoseStamped,
            "/detected_object/pos",
            self.target_cb,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/mirte_base_controller/cmd_vel", 10)
        self.pickup_pub = self.create_publisher(String, "/activate_arm/pick", 10)
        self.dropoff_pub = self.create_publisher(String, "/activate_dropoff_move", 10)
        self.detect_pub = self.create_publisher(String, "/activate_detection", 10)

        # Subscribe to pickup done
        self.create_subscription(String, "/block_picked_up", self.pickup_done_cb, 10)

        # Control parameters — tune these on the real robot
        self.rot_speed   = 0.3    # rad/s
        self.drive_speed = 0.12   # m/s
        self.angle_tol   = math.radians(3)
        self.dist_tol    = 0.02   # metres

        # Prevent concurrent executions
        self.busy = False

        self.get_logger().info("ODOM-based motor control node ready")

    # ------------------------------------------------------------------ #
    def pickup_done_cb(self, msg):
        if msg.data == "picked_up":
            self.get_logger().info("[MOTOR] Block picked up — triggering drop-off move")
            out = String()
            out.data = "start"
            self.dropoff_pub.publish(out)

    # ------------------------------------------------------------------ #
    def target_cb(self, msg):
        if self.busy:
            self.get_logger().warn("[MOTOR] Already executing — ignoring new target")
            return

        self.busy = True

        tx = msg.pose.position.x
        ty = msg.pose.position.y

        yaw   = math.atan2(ty, tx)
        dist  = math.sqrt(tx**2 + ty**2)

        self.get_logger().info(
            f"[MOTOR] Target x={tx:.3f} y={ty:.3f} "
            f"dist={dist:.3f}m yaw={math.degrees(yaw):.1f}°"
        )

        # 1. Rotate to face the block
        self.rotate_by(yaw)

        # 2. Drive forward
        self.drive_forward(dist)

        # 3. Full stop
        self.stop()

        # 4. Trigger pickup
        out = String()
        out.data = "start"
        self.pickup_pub.publish(out)
        self.get_logger().info("[MOTOR] Pickup triggered")

        self.busy = False

    # ------------------------------------------------------------------ #
    def rotate_by(self, angle_rad):
        """Open-loop rotation by angle_rad (positive = left/CCW)."""
        if abs(angle_rad) < self.angle_tol:
            return

        self.get_logger().info(f"[MOTOR] Rotating {math.degrees(angle_rad):.1f}°")

        duration  = abs(angle_rad) / self.rot_speed
        direction = 1.0 if angle_rad > 0 else -1.0

        twist = Twist()
        twist.angular.z = direction * self.rot_speed

        t_end = time.time() + duration
        while time.time() < t_end:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop()

    # ------------------------------------------------------------------ #
    def drive_forward(self, dist_m):
        """Open-loop forward drive for dist_m metres."""
        if dist_m < self.dist_tol:
            return

        self.get_logger().info(f"[MOTOR] Driving {dist_m:.3f} m")

        duration = dist_m / self.drive_speed

        twist = Twist()
        twist.linear.x = self.drive_speed

        t_end = time.time() + duration
        while time.time() < t_end:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop()

    # ------------------------------------------------------------------ #
    def stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.15)


# ------------------------------------------------------------------ #
def main():
    rclpy.init()
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()