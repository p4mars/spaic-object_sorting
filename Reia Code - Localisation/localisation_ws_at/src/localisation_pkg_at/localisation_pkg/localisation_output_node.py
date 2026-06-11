import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


_AMCL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE
)


class LocalisationOutputNode(Node):
    def __init__(self):
        super().__init__('localisation_output_node')

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        self.subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            _AMCL_QOS
        )

        self.get_logger().info('localisation output node started')

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        self.get_logger().info(
            f'position → x: {self.current_x:.3f} '
            f'y: {self.current_y:.3f} '
            f'yaw: {math.degrees(self.current_yaw):.1f}°'
        )

    def quaternion_to_yaw(self, orientation):
        siny = 2.0 * (orientation.w * orientation.z +
                      orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y +
                              orientation.z * orientation.z)
        return math.atan2(siny, cosy)

    def get_position(self):
        return (self.current_x, self.current_y, self.current_yaw)


def main():
    rclpy.init()
    node = LocalisationOutputNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
