#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        # QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        self.publisher_ = self.create_publisher(Image, 'yolo_depth/image_raw', qos_profile)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()
        # 테스트 이미지 파일 경로 (실제 존재하는 파일 경로로 수정)
        self.image_path = "test3.png"
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error(f"Failed to load image from: {self.image_path}")
        else:
            # 640x640 크기로 리사이즈
            self.image = cv2.resize(self.image, (640, 640))
            self.get_logger().info(f"Loaded image from: {self.image_path}")

    def timer_callback(self):
        if self.image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)
            self.get_logger().info("Published test image.")

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
