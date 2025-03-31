#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        # QoS 설정
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        # "points" 토픽에 문자열 메시지를 발행하는 Publisher 생성
        self._pub_points = self.create_publisher(String, "points", 10)
        # YoloDepthNode에서 발행하는 3D 좌표가 포함된 DetectionArray 메시지를 구독
        self._sub_detection = self.create_subscription(
            DetectionArray, '/yolo_depth/detections_3d', self.detection_callback, self.qos_profile,
        )

    def detection_callback(self, detections_msg: DetectionArray):
        self.get_logger().info("Published detections:\n")
        detection_lines = []
        for det in detections_msg.detections:
            # 각 detection의 이름은 class_name 필드에서 가져온다고 가정합니다.
            label = det.class_name if hasattr(det, 'class_name') else "unknown"
            # point_3d 필드에 x, y, z 값이 있다고 가정합니다.
            x = det.point_3d.x if hasattr(det, 'point_3d') else 0.0
            y = det.point_3d.y if hasattr(det, 'point_3d') else 0.0
            z = det.point_3d.z if hasattr(det, 'point_3d') else 0.0
            detection_lines.append(f"{label}: position = ({x}, {y}, {z})")
        # 리스트에 담긴 각 detection 정보를 줄바꿈 문자로 연결하여 하나의 문자열로 만듭니다.
        data_str = "\n".join(detection_lines)
        msg = String()
        msg.data = data_str
        self._pub_points.publish(msg)
        self.get_logger().info("Published detections:\n" + data_str)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
