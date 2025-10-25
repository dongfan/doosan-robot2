# realsense_manager.py
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import pyrealsense2 as rs
import numpy as np

class RealSenseManager:
    def __init__(self, node):
        self.node = node
        self.bridge = CvBridge()

        self.latest_color = None
        self.latest_depth_mm = None
        self.intrinsics = None

        # ✅ 올바른 토픽 + 센서 QoS 사용
        self.node.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            qos_profile_sensor_data
        )
        self.node.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )
        self.node.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10
        )
        self.node.get_logger().info("📷 RealSense subscriptions set (color+aligned depth)")

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        # aligned depth: 16UC1 (mm)
        self.latest_depth_mm = self.bridge.imgmsg_to_cv2(msg, "16UC1")

    def info_callback(self, msg):
        if self.intrinsics is None:
            intr = rs.intrinsics()
            intr.width  = msg.width
            intr.height = msg.height
            intr.ppx = msg.k[2]
            intr.ppy = msg.k[5]
            intr.fx  = msg.k[0]
            intr.fy  = msg.k[4]
            intr.model = (
                rs.distortion.brown_conrady
                if msg.distortion_model in ['plumb_bob', 'rational_polynomial']
                else rs.distortion.none
            )
            intr.coeffs = list(msg.d)
            self.intrinsics = intr
            self.node.get_logger().info("✅ Camera intrinsics received")

    def get_latest_frames(self):
        return self.latest_color, self.latest_depth_mm
