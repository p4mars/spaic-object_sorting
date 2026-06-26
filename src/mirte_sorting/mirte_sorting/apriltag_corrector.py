import math
import os

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
import tf2_ros
import yaml

from geometry_msgs.msg import PoseWithCovarianceStamped


class AprilTagCorrectorAt(Node):

    def __init__(self):
        super().__init__('apriltag_corrector_at')

        #  declaring parameters (basically loading from launch file)
        self.declare_parameter('semantic_map_path', '')
        self.declare_parameter('tag_frame_prefix', 'tag36h11:')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('correction_cooldown', 3.0)
        self.declare_parameter('max_detection_distance', 1.5)

        # loading semantic map the one that has all the april tag vals
        map_path = self.get_parameter('semantic_map_path').value
        self.tag_poses = self._load_semantic_map(map_path)
        self.tag_prefix = self.get_parameter('tag_frame_prefix').value

        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cooldown = self.get_parameter('correction_cooldown').value
        self.max_dist = self.get_parameter('max_detection_distance').value
        self._last_correction = 0.0

        # this basically lets the node look up transforms between coordinate frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # creating teh publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
            ),
        )

        # runs the fn _check_tags every 2 sec
        self.create_timer(0.2, self._check_tags)

        self.get_logger().info(
            f'apriltag corrector ready  |  '
            f'watching tag IDs: {sorted(self.tag_poses.keys())}  |  '
            f'prefix: "{self.tag_prefix}"'
        )

    # loads the semantic map
    def _load_semantic_map(self, path: str) -> dict:
        if not path or not os.path.exists(path):
            self.get_logger().warn(
                f'semantic_map_path not found: "{path}"  — '
                'pass the correct path as a launch argument'
            )
            return {}

        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        tags = {}

        def _extract(obj):
            if not isinstance(obj, dict):
                return
            if (
                'tag_id' in obj
                and 'pose' in obj
                and obj['tag_id'] is not None
                and isinstance(obj.get('pose'), dict)
                and obj['pose'].get('x') is not None
            ):
                tid = int(obj['tag_id'])
                p   = obj['pose']
                tags[tid] = (float(p['x']), float(p['y']), float(p.get('yaw', 0.0)))
            for v in obj.values():
                _extract(v)

        _extract(data)

        if tags:
            self.get_logger().info(f'loaded {len(tags)} tags: {sorted(tags.keys())}')
        else:
            self.get_logger().warn('semantic_map parsed but no tags found — check YAML structure')

        return tags

    # main timer

    def _check_tags(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_correction < self.cooldown:
            return

        for tag_id, (wx, wy, wyaw) in self.tag_poses.items():
            tag_frame = f'{self.tag_prefix}{tag_id}'
            result    = self._try_correct(tag_id, tag_frame, wx, wy, wyaw)
            if result is not None:
                rx, ry, ryaw, dist = result
                self._publish_correction(rx, ry, ryaw, dist)
                self._last_correction = now
                self.get_logger().info(
                    f'[tag {tag_id}] correction applied  '
                    f'x={rx:.3f}  y={ry:.3f}  '
                    f'yaw={math.degrees(ryaw):.1f}°  '
                    f'dist={dist:.2f} m'
                )
                break  # one correction per tick is enough

    def _try_correct(self, tag_id, tag_frame, wx, wy, wyaw):

        zero_time = rclpy.time.Time()
        short     = rclpy.duration.Duration(seconds=0.05)

        #
        try:
            cam_to_tag = self.tf_buffer.lookup_transform(
                self.camera_frame, tag_frame, zero_time, timeout=short
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None  # tag not currently visible

        t    = cam_to_tag.transform.translation
        dist = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        if dist > self.max_dist:
            return None


        try:
            base_to_cam = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, zero_time, timeout=short
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                f'No TF: {self.base_frame} → {self.camera_frame}.  '
                'Check robot URDF or set camera_frame parameter.',
                throttle_duration_sec=10.0,
            )
            return None

        try:
            rx, ry, ryaw = self._robot_world_pose(
                wx, wy, wyaw, cam_to_tag, base_to_cam
            )
        except Exception as e:
            self.get_logger().warn(f'pose computation failed: {e}')
            return None

        return rx, ry, ryaw, dist


    def _robot_world_pose(self, wx, wy, wyaw, cam_to_tag_tf, base_to_cam_tf):

        ct   = cam_to_tag_tf.transform
        tx   = ct.translation.x
        ty   = ct.translation.y
        tyaw = self._quat_to_yaw(ct.rotation)

        bc   = base_to_cam_tf.transform
        bx   = bc.translation.x
        by   = bc.translation.y
        byaw = self._quat_to_yaw(bc.rotation)

        # camera world pose = tag_world * inv(cam_to_tag)
        cx, cy, cyaw = self._se2_compose(wx, wy, wyaw,
                                         *self._se2_inverse(tx, ty, tyaw))

        # robot world pose = camera_world * inv(base_to_cam)
        rx, ry, ryaw = self._se2_compose(cx, cy, cyaw,
                                         *self._se2_inverse(bx, by, byaw))
        return rx, ry, ryaw

    @staticmethod
    def _se2_compose(x1, y1, t1, x2, y2, t2):
        x = x1 + math.cos(t1) * x2 - math.sin(t1) * y2
        y = y1 + math.sin(t1) * x2 + math.cos(t1) * y2
        return x, y, t1 + t2

    @staticmethod
    def _se2_inverse(x, y, t):
        return (
            -(math.cos(t) * x + math.sin(t) * y),
             (math.sin(t) * x - math.cos(t) * y),
            -t,
        )

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    # creating a publisher for the correction

    def _publish_correction(self, x: float, y: float, yaw: float, dist: float):

        xy_cov  = max(0.01, dist * 0.03) ** 2
        yaw_cov = max(0.02, dist * 0.05) ** 2

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = xy_cov   # x
        msg.pose.covariance[7] = xy_cov   # y
        msg.pose.covariance[35] = yaw_cov  # yaw
        self.pose_pub.publish(msg)


def main():
    rclpy.init()
    node = AprilTagCorrectorAt()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
