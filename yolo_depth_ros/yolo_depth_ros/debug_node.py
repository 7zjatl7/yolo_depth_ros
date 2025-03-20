# Copyright (C) 2023 Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import cv2
import random
import numpy as np
import matplotlib
from typing import Tuple, List

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import message_filters
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from yolo_msgs.msg import BoundingBox2D
from yolo_msgs.msg import Detection
from yolo_msgs.msg import DetectionArray
from visualization_msgs.msg import Marker, MarkerArray

class DebugNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("debug_node")

        self._id_to_color = {}
        self.cv_bridge = CvBridge()
        
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT.value)
        self.declare_parameter("cmap_name", "Spectral_r")


    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.cmap_name = self.get_parameter("cmap_name").get_parameter_value().string_value
        reliability = self.get_parameter("image_reliability").get_parameter_value().integer_value

        self.image_qos_profile = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._dbg_rgb_pub = self.create_publisher(Image, "dbg_rgb_image", 10)
        self._dbg_depth_pub = self.create_publisher(Image, "dbg_depth_image", 10)

        self._marker_pub = self.create_publisher(MarkerArray, "detections_markers", 10)


        self.get_logger().info(f"[{self.get_name()}] Configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Activating...")

        self.image_sub = message_filters.Subscriber(
            self, Image, "image_raw", qos_profile=self.image_qos_profile
        )

        self.depth_sub = message_filters.Subscriber(
            self, Image, "depth_estimation", qos_profile=10
        )

        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "detections_3d", qos_profile=10
        )

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub, self.detections_sub),
            queue_size=10,
            slop=1.0
        )
        self._synchronizer.registerCallback(self.detections_cb)

        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")

        self.destroy_subscription(self.color_sub.sub)
        self.destroy_subscription(self.depth_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)

        del self._synchronizer

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")
        
        self.destroy_publisher(self._dbg_rgb_pub)
        self.destroy_publisher(self._dbg_depth_pub)

        self.destroy_publisher(self._marker_pub)

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")
        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS

    def detections_cb(self, img_msg: Image, depth_msg: Image, detection_msg: DetectionArray) -> None:

        rgb_img = self.cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        depth_img = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="bgr8")
        dbg_rgb_img = rgb_img.copy()
        dbg_depth_img = depth_img.copy()

        detection_texts: List[Tuple[str, Tuple[int, int, int]]] = []

        for i, detection in enumerate(detection_msg.detections):
            color = self.generate_color(i)

            bbox = detection.bbox
            cx = bbox.center.position.x
            cy = bbox.center.position.y
            w = bbox.size.x
            h = bbox.size.y

            min_pt = (int(cx - w / 2.0), int(cy - h / 2.0))
            max_pt = (int(cx + w / 2.0), int(cy + h / 2.0))

            cv2.rectangle(dbg_rgb_img, min_pt, max_pt, color, 2)
            cv2.rectangle(dbg_depth_img, min_pt, max_pt, color, 2)

            depth_val = getattr(detection, "depth", 0.0)
            px = getattr(detection.point_3d, "x", 0.0)
            py = getattr(detection.point_3d, "y", 0.0)
            pz = getattr(detection.point_3d, "z", 0.0)
            score = detection.score
            class_name = detection.class_name

            text_str = f"{class_name} ({score:.2f}): {depth_val:.2f}m ({px:.2f},{py:.2f},{pz:.2f})"
            detection_texts.append((text_str, color))

        line_height = 30
        x_right_margin = 10
        start_y = 30
        img_h, img_w = dbg_rgb_img.shape[:2]

        for i, (text_str, color) in enumerate(detection_texts):
            (text_w, text_h), baseline = cv2.getTextSize(
                text_str, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            x_pos = img_w - x_right_margin - text_w
            y_pos = start_y + i * line_height

            cv2.putText(
                dbg_rgb_img,
                text_str,
                (x_pos, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                dbg_depth_img,
                text_str,
                (x_pos, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
                cv2.LINE_AA,
            )

        self._dbg_rgb_pub.publish(self.cv_bridge.cv2_to_imgmsg(dbg_rgb_img, encoding="bgr8"))
        self._dbg_depth_pub.publish(self.cv_bridge.cv2_to_imgmsg(dbg_depth_img, encoding="bgr8"))

        marker_array = MarkerArray()
        for i, detection in enumerate(detection_msg.detections):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.SPHERE 
            marker.action = Marker.ADD

            marker.pose.position.x = detection.point_3d.x
            marker.pose.position.y = detection.point_3d.y
            marker.pose.position.z = detection.point_3d.z

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0

            marker_array.markers.append(marker)

        self._marker_pub.publish(marker_array)

    def generate_color(self, index: int) -> Tuple[int, int, int]:
        """
        index를 바탕으로 HSV 색을 생성한 뒤 BGR로 변환해 반환합니다.
        Hue 범위(0~179)를 일정 간격(index*30)으로 증가시키고,
        Saturation/Value는 고정된 값으로 지정합니다.
        """
        hue = (index * 30) % 180
        sat = 200
        val = 200
        color_hsv = np.uint8([[[hue, sat, val]]])  # shape=(1,1,3)
        color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0, 0]
        # (B, G, R) = color_bgr
        return (int(color_bgr[0]), int(color_bgr[1]), int(color_bgr[2]))


def main():
    rclpy.init()
    node = DebugNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()