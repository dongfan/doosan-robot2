import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from dsr_example.realsense_manager import RealSenseManager
from dsr_example.yolo_manager import YoloDetector
import cv2
import json

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('realsense_yolo_node')
        self.bridge = CvBridge()
        self.realsense = RealSenseManager()
        self.yolo = YoloDetector()

        # 객체 인식 결과 publish용
        self.pub = self.create_publisher(String, '/yolo/detections', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.get_logger().info("📸 RealSense + YOLO Node initialized")

    def timer_callback(self):
        color_frame, _ = self.realsense.get_latest_frames()
        if color_frame is None:
            return

        detections = self.yolo.detect(color_frame)
        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.pub.publish(msg)

        self.get_logger().info("📸 RealSense 실시간 화면 실행")
        cv2.imshow("YOLO Detection", color_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
