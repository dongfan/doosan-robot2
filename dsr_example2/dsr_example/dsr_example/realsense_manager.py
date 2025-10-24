import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import cv2
import numpy as np

class RealSenseManager(Node):
    def __init__(self, node_name="realsense_manager"):
        super().__init__(node_name)
        self.bridge = CvBridge()

        self.latest_color = None
        self.latest_depth_mm = None
        self.intrinsics = None

        # --- 단일 구독 방식 (message_filters 제거) ---
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.info_callback,
            10
        )

        self.get_logger().info("📷 RealSense 카메라 단일 구독 모드로 초기화 완료")

    def color_callback(self, msg):
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 🔸 프레임 수신 로그
        # print("📸 Color frame received.")
        cv2.imshow("RealSense Camera", self.latest_color)
        cv2.waitKey(1)

    def depth_callback(self, msg):
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
            self.get_logger().info("카메라 Intrinsics 수신 완료 ✅")

    def get_latest_frames(self):
        return self.latest_color, self.latest_depth_mm
