
"""Very small differential drive mock so Nav2 can be tested without Mirte hardware.

It subscribes to cmd_vel, integrates a fake odom pose, broadcasts odom->base_link,
and publishes an empty LaserScan. 

Only used when its launched on the computer not on the robot itself. 
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float):
    """Convert a yaw angle (radians) to a quaternion (x, y, z, w).
    
    For a 2D robot only the z and w are non-zero. This is 
    used for both the TF broadcast and the odometry.
    """
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class MockMirteBase(Node):
    def __init__(self):
        super().__init__('mock_mirte_base')
        # Parameters 
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_smoothed')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 20.0)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        rate = float(self.get_parameter('publish_rate').value)

        # Internal odometry state
        # Pose is integrated from velocity commands using Euler integration
        self.x = 0.0 # x position 
        self.y = 0.0 # y position
        self.yaw = 0.0 # orientation (radians)
        self.v = 0.0 # linear velocity 
        self.w = 0.0 # angular velocity 
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now() # Used to detect stale commands

        # ROS interfaces
        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Main update loop, runs at publish_rate
        self.timer = self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(
            f'Mock base running: subscribes {self.cmd_vel_topic}, publishes {self.odom_topic}, {self.scan_topic}, TF {self.odom_frame}->{self.base_frame}'
        )

    def cmd_cb(self, msg: Twist):
        """Store the latest velocity command from Nav2 """
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        """Main update loop: integrate odometry, 
        broadcast TF, publish odom and scan """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Safety stop if no cmd_vel have been recieved. 
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > 0.7:
            self.v = 0.0
            self.w = 0.0

        # Euler integration of the differential drive kinematics
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

        # Broadcast odom to base link TF so Nav2 can track the robots position
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # Publish odometry message so Nav2 controller and costmaps can use it
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        # Publish an empty LaserScan so Nav2 costmap layers that require a scan
        # topic do not block. All ranges are set to inf (no obstacles detected).
        # On the real robot this is replaced by lidar data.
        scan = LaserScan()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0 # 1 degree resolution (360 points)
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 3.0
        n = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * n # No obstacles
        self.scan_pub.publish(scan)


def main(args: Optional[list] = None):
    rclpy.init(args=args)
    node = MockMirteBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
