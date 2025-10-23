import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CarDetector(Node):
    def __init__(self):
        super().__init__('car_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Bool, '/car_detected', 10)
        self.get_logger().info("🚗 CarDetector Node Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 빨간색 계열 toy car 감지 (예시)
        mask1 = cv2.inRange(hsv, (0, 120, 70), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (170, 120, 70), (180, 255, 255))
        mask = mask1 | mask2

        area = np.count_nonzero(mask)
        if area > 5000:
            self.pub.publish(Bool(data=True))
            self.get_logger().info("🟥 Toy car detected!")

def main(args=None):
    rclpy.init(args=args)
    node = CarDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
