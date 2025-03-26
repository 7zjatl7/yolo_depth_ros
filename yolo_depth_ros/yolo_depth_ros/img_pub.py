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
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.bridge = CvBridge()

        self.image_path = "cctv1.png"
        self.image = cv2.imread(self.image_path)
        if self.image is None:
            self.get_logger().error(f"Failed to load image from: {self.image_path}")
        else:
            # 640x640 크기로 리사이즈
            self.image = cv2.resize(self.image, (1920, 1080))
            self.get_logger().info(f"Loaded image from: {self.image_path}")

        # 동영상 파일 경로 (실제 존재하는 파일 경로로 수정)
        # self.video_path = "chungmu1_cctv.mp4"
        # self.cap = cv2.VideoCapture(self.video_path)
        # if not self.cap.isOpened():
        #     self.get_logger().error(f"동영상을 열 수 없습니다: {self.video_path}")
        # else:
        #     self.get_logger().info(f"동영상 열림: {self.video_path}")

        # 여러 바운딩 박스와 레이블 정보 정의
        # 각 항목은 ((xmin, ymin, xmax, ymax), "label : 객체명 (x, y, z)") 형식입니다.
        self.bounding_boxes = [
            ((755, 680, 780, 735), "label : wood (-53.93, 136.20, 1.04)"),
            ((1030, 775, 1070, 840), "label : can (-50.93, 133.20, 1.02)"),
            # 필요에 따라 추가
        ]
        # 각각의 바운딩 박스에 사용할 색상 (B, G, R)
        self.colors = [
            (0, 0, 0),   
            (0, 0, 255),   # 초록
            (255, 0, 0),   # 파랑
            (0, 255, 255)  # 노랑
        ]

    def timer_callback(self):
        # if self.cap.isOpened():
        #     ret, frame = self.cap.read()
        #     # 동영상 끝에 도달하면 처음으로 돌림
        #     if not ret:
        #         self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        #         ret, frame = self.cap.read()
        #         if not ret:
        #             self.get_logger().error("동영상에서 프레임을 읽지 못했습니다.")
        #             return

        # 프레임의 전체 크기
        img_h, img_w = self.image.shape[:2]
        margin = 10  # 오른쪽 위 여백

        # 바운딩 박스 그리기 (각각 다른 색상 사용)
        for idx, (bbox, _) in enumerate(self.bounding_boxes):
            xmin, ymin, xmax, ymax = bbox
            color = self.colors[idx % len(self.colors)]
            cv2.rectangle(self.image, (xmin, ymin), (xmax, ymax), color, 3)

        # 오른쪽 위에 전체 레이블을 모아서 출력 (각 레이블마다 해당 색상 적용)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.2
        text_thickness = 3

        for i, (_, label_text) in enumerate(self.bounding_boxes):
            (text_w, text_h), baseline = cv2.getTextSize(label_text, font, font_scale, text_thickness)
            text_x = img_w - text_w - margin
            line_height = text_h + 10
            text_y = margin + (i + 1) * line_height
            # 해당 인덱스의 색상 적용
            color = self.colors[i % len(self.colors)]
            cv2.putText(self.image, label_text, (text_x, text_y), font, font_scale, color, text_thickness, cv2.LINE_AA)

        # ROS Image 메시지로 변환 후 퍼블리싱
        msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info("동영상 프레임에 여러 바운딩 박스와 오른쪽 위 레이블(각각 다른 색상)을 오버레이 후 퍼블리싱 완료.")

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

# class TestImagePublisher(Node):
#     def __init__(self):
#         super().__init__('test_image_publisher')
#         # QoS 설정
#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.RELIABLE,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             durability=QoSDurabilityPolicy.VOLATILE,
#             depth=10
#         )
#         self.publisher_ = self.create_publisher(Image, 'yolo_depth/image_raw', qos_profile)
#         self.timer = self.create_timer(0.03, self.timer_callback)
#         self.bridge = CvBridge()
#         # 테스트 이미지 파일 경로 (실제 존재하는 파일 경로로 수정)
#         self.image_path = "cctv1.png"
#         self.image = cv2.imread(self.image_path)
#         if self.image is None:
#             self.get_logger().error(f"Failed to load image from: {self.image_path}")
#         else:
#             # 640x640 크기로 리사이즈
#             self.image = cv2.resize(self.image, (1920, 1080))
#             self.get_logger().info(f"Loaded image from: {self.image_path}")

#     def timer_callback(self):

#         # if self.cap.isOpened():
#             # ret, frame = self.cap.read()
#             # # 동영상 끝에 도달하면 처음으로 돌림
#             # if not ret:
#             #     self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
#             #     ret, frame = self.cap.read()
#             #     if not ret:
#             #         self.get_logger().error("동영상에서 프레임을 읽지 못했습니다.")
#             #         return
#             # msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")


#         # msg.header.stamp = self.get_clock().now().to_msg()
#         # self.publisher_.publish(msg)
#         # self.get_logger().info("동영상 프레임 퍼블리싱 완료.")

#         if self.image is not None:
#             msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
#             msg.header.stamp = self.get_clock().now().to_msg()
#             self.publisher_.publish(msg)
#             self.get_logger().info("Published test image.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = TestImagePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()