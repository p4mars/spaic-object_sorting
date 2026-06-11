import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math



_AMCL_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE
)


class HybridLocaliser(Node):
    def __init__(self):
        super().__init__('hybrid_localiser')

        self.amcl_pose  = None
        self.rtab_pose  = None
        self.odom_pose  = None

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            _AMCL_QOS
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/rtabmap/localization_pose',
            self.rtab_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/robot_pose',
            10
        )

        self.create_timer(0.1, self.publish_pose)
        self.get_logger().info('hybrid localiser started')

    def amcl_callback(self, msg):
        self.amcl_pose = msg

    def rtab_callback(self, msg):
        self.rtab_pose = msg

    def odom_callback(self, msg):
        self.odom_pose = msg

    def get_uncertainty(self, pose_msg):
        return math.sqrt(abs(pose_msg.pose.covariance[0]))

    def publish_pose(self):
        available = []

        if self.amcl_pose is not None:
            uncertainty = self.get_uncertainty(self.amcl_pose)
            weight = max(0.1, 1.0 - uncertainty * 2)
            available.append((self.amcl_pose, weight, 'AMCL'))

        if self.rtab_pose is not None:
            uncertainty = self.get_uncertainty(self.rtab_pose)
            weight = max(0.1, 1.0 - uncertainty * 2)
            available.append((self.rtab_pose, weight, 'RTAB'))

        if not available:
            if self.odom_pose is not None:
                self.get_logger().warn(
                    'no map sources — using odometry only'
                )
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.pose.pose = self.odom_pose.pose.pose
                self.pub.publish(msg)
            return

        total_weight = sum(w for _, w, _ in available)

        x_fused = sum(
            p.pose.pose.position.x * w / total_weight
            for p, w, _ in available
        )
        y_fused = sum(
            p.pose.pose.position.y * w / total_weight
            for p, w, _ in available
        )

        best = max(available, key=lambda item: item[1])

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x_fused
        msg.pose.pose.position.y = y_fused
        msg.pose.pose.orientation = best[0].pose.pose.orientation

        self.pub.publish(msg)

        sources = [name for _, _, name in available]
        self.get_logger().info(
            f'fused -> x: {x_fused:.3f}  y: {y_fused:.3f}  '
            f'sources: {sources}'
        )


def main():
    rclpy.init()
    node = HybridLocaliser()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
