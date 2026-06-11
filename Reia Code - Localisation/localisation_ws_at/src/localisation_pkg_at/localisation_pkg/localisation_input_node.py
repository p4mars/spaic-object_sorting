import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import time


class LocalisationInputNode(Node):
    def __init__(self):
        super().__init__('localisation_input_node')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.pose_sent = False

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.get_logger().info('waiting for map...')

    def map_callback(self, msg):
        if not self.pose_sent:
            self.pose_sent = True
            time.sleep(2.0)
            self.send_pose()
            self.destroy_subscription(self.map_sub)

    def send_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x    = 0.0
        msg.pose.pose.position.y    = 0.0
        msg.pose.pose.position.z    = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.pose.covariance[0]  = 0.5
        msg.pose.covariance[7]  = 0.5
        msg.pose.covariance[35] = 0.1

        self.publisher.publish(msg)
        self.get_logger().info('initial pose sent — done')


def main():
    rclpy.init()
    node = LocalisationInputNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()