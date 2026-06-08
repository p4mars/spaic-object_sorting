import scripts.Detection.Detection_Functions as DF
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion 
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import rclpy.duration


import cv2
import numpy as np
from ultralytics import YOLO
import math
import time
from pathlib import Path

# load YOLO model
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "best.pt"
model = YOLO(str(MODEL_PATH))


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        # checks if the perception should be running
        self.active = False
        self.create_subscription(String, "/activate_detection", self.activate_cb, 10)

        # defines Cv bridge
        self.bridge = CvBridge()
        self.color = None
        self.depth = None

        # camera subscriptions
        self.create_subscription(Image, "/camera/color/image_raw", self.color_cb, 10)
        self.create_subscription(Image, "/camera/depth/image_raw", self.depth_cb, 10)

        # publisher for robot movement
        self.cmd_pub = self.create_publisher(Twist, "/mirte_base_controller/cmd_vel", 10)

        # arm grasp pose (now in base_link)
        self.arm_grasp_pub = self.create_publisher(PoseStamped, "/arm_grasp_pose", 10)

        # publisher for empty block field
        self.no_blocks_pub = self.create_publisher(String, "/blocks_exhausted", 10)

        # publisher for object class and LOCAL position
        self.target_pub = self.create_publisher(PoseStamped, "/detected_object/pos", 10)
        self.target_pub_class = self.create_publisher(String, "/detected_object/class",10)

        # transform buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # frame buffer
        self.frame_buffer = []
        self.N_FRAMES = 10

        # confidence threshold
        self.conf_thres = 0.5

        # stop before block distance
        self.block_stop = 0.2

        # image size
        self.image_size = [640, 480]  # [width, height]

        # fields of view
        self.depth_fov = [58.4, 45.5]   # [FOV_x, FOV_y] in degrees
        self.color_fov = [66.1, 40.2]

        # approximate intrinsics from FOV
        self.fx_d = (self.image_size[0] / 2.0) / np.tan(np.deg2rad(self.depth_fov[0] / 2.0))
        self.fy_d = (self.image_size[1] / 2.0) / np.tan(np.deg2rad(self.depth_fov[1] / 2.0))
        self.cx_d = self.image_size[0] / 2.0
        self.cy_d = self.image_size[1] / 2.0

        # timer loop that activates loop every n seconds
        self.timer = self.create_timer(0.1, self.loop)


    def activate_cb(self, msg):
        if msg.data == "start":
            self.active = True
            self.get_logger().info("Detection activated")
        elif msg.data == "stop":
            self.active = False
            self.get_logger().info("Detection deactivated")


    def color_cb(self, msg):
        # convert ROS2 image to cv2
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_cb(self, msg):
        # convert ROS2 depth image to cv2
        self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")


    def yaw_to_quaternion(self, yaw):
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
    
    def color_to_depth_pixel(self, u_c, v_c):
        # normalized color ray
        x_c = (u_c - self.cx_color) / self.fx_color
        y_c = (v_c - self.cy_color) / self.fy_color

        # project into depth pixel space
        u_d = int(self.fx_d * x_c + self.cx_d)
        v_d = int(self.fy_d * y_c + self.cy_d)

        return u_d, v_d



    def loop(self):
        # only runs when told not to be idle
        if not self.active:
            return
        
        # wait until both images are available
        if self.color is None or self.depth is None:
            return
        
        # Collect frames for multi-frame sampling
        self.frame_buffer.append((self.color.copy(), self.depth.copy()))

        # wait until enough frames are collected
        if len(self.frame_buffer) < self.N_FRAMES:
            return

        # Run YOLO on all collected frames
        detections = []

        for color, depth in self.frame_buffer:
            results = model(color)

            for result in results:
                boxes = result.boxes

                for i in range(len(boxes)):

                    # box width and height
                    cx = int(boxes.xywh[i][0].item())
                    cy = int(boxes.xywh[i][1].item())

                    # depth lookup (row = y, col = x)
                    z_list = []

                    for pixel in range(cx - boxes.xywh[i][2] / 2, cx + boxes.xywh[i][2] / 2):
                        z_list.append((depth[cy,pixel],pixel))
                    
                    z_list_sorted = sorted(z_list)
                    z, cx = z_list_sorted[int(len(z_list_sorted)/2)]

                    # convert mm → meters
                    z /= 1000.0

                    # relative position using depth intrinsics
                    x = (cx - self.cx_d) * (z / self.fx_d)
                    y = (cy - self.cy_d) * (z / self.fy_d)

                    # class + confidence
                    class_id = int(boxes.cls[i].item())
                    class_name = result.names[class_id]
                    conf = float(boxes.conf[i].item())

                    if conf <= self.conf_thres:
                        continue 

                    detections.append({
                        "pos": [x, y, z],
                        "class": class_name,
                        "conf": conf,
                        "pixel": (cx, cy)
                    })

        # clear buffer for next cycle
        self.frame_buffer = []

        # Cluster fuse detections across frames
        clusters = DF.cluster_detections(detections, radius=10)
        fused = DF.fuse_clusters(clusters)

        # Pick best target
        target = DF.pick_best(fused)

        if target is None:
            self.get_logger().info("No detections — staying idle")
            return

        # unpack target (already in camera frame)
        x, y, z = target["pos"]

        # Publish grasp pose for the arm (in base_link)
        grasp = PoseStamped()
        grasp.header.stamp = self.get_clock().now().to_msg()
        
        # Convert from camera frame into odom frame
        point_cam = PointStamped()
        point_cam.header.frame_id = "camera_depth_optical_frame"
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = z

        try:
            point_odom = self.tf_buffer.transform(
                point_cam,
                "odom",
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform camera to odom failed: {e}")
            return

        bx = point_odom.point.x
        by = point_odom.point.y
        bz = point_odom.point.z

        grasp = PoseStamped()
        grasp.header.stamp = self.get_clock().now().to_msg()
        grasp.header.frame_id = "odom"

        grasp.pose.position.x = bx
        grasp.pose.position.y = by
        grasp.pose.position.z = bz
        grasp.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.arm_grasp_pub.publish(grasp)
        self.get_logger().info(f"[BLOCK] base_link=({x:.3f}, {y:.3f}, {z:.3f})")

        # Publish navigation target for your new motor node
        nav = PoseStamped()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = "odom"

        dist = math.sqrt(bx*bx + by*by)
        ux = bx / dist
        uy = by / dist

        nav.pose.position.x = bx - self.block_stop * ux
        nav.pose.position.y = by - self.block_stop * uy
        nav.pose.position.z = 0.0

        yaw = math.atan2(by, bx)
        nav.pose.orientation = self.yaw_to_quaternion(yaw)

        self.target_pub.publish(nav)
        self.target_pub_class.publish(String(data=target["class"]))

        self.get_logger().info(
            f"[NAV] waypoint=({nav.pose.position.x:.3f}, {nav.pose.position.y:.3f}), yaw={yaw:.3f} rad"
        )
        
        # puts detection to sleep
        self.active = False
        self.get_logger().info("Detection completed — going idle")


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
