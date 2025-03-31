import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn, LifecycleState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

import numpy as np
import message_filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolo_msgs.msg import DetectionArray, Detection



class YoloDepthNode(LifecycleNode):

    def __init__(self):
        super().__init__("yolo_depth_node")

        # Declare parameters
        self.declare_parameter("detections", "tracking")
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT.value)

        self.declare_parameter("fx", 1000.0)
        self.declare_parameter("fy", 1800.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 320.0)

        self.cv_bridge = CvBridge()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:

        ret = super().on_configure(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret

        self.get_logger().info("[yolo_depth_node] on_configure...")
        self.reliability = self.get_parameter("image_reliability").get_parameter_value().integer_value
        self.sub_detect_topic = self.get_parameter("detections").get_parameter_value().string_value
        self.fx = self.get_parameter("fx").get_parameter_value().double_value
        self.fy = self.get_parameter("fy").get_parameter_value().double_value
        self.cx = self.get_parameter("cx").get_parameter_value().double_value
        self.cy = self.get_parameter("cy").get_parameter_value().double_value

        self.sync_qos = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self._pub_fused = self.create_publisher(DetectionArray, "detections_3d", 10)


        # super().on_configure(state)
        self.get_logger().info("[yolo_depth_fusion_node] Configured.")
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("[yolo_depth_fusion_node] on_activate...")

        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, self.sub_detect_topic, qos_profile=self.sync_qos
        )
        self.depth_sub = message_filters.Subscriber(
            self, Image, "depth_estimation", qos_profile=self.sync_qos
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self.detections_sub, self.depth_sub],
            queue_size=10,
            slop=0.5
        )
        self._sync.registerCallback(self.fusion_cb)

        self.get_logger().info("[yolo_depth_fusion_node] Activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("[yolo_depth_fusion_node] on_deactivate...")

        self.destroy_subscription(self.detections_sub.sub)
        self.destroy_subscription(self.depth_sub.sub)

        del self._sync

        self.get_logger().info("[yolo_depth_fusion_node] Deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("[yolo_depth_fusion_node] on_cleanup...")

        self.destroy_publisher(self._pub_fused)

        self.get_logger().info("[yolo_depth_fusion_node] Cleaned up.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("[yolo_depth_fusion_node] on_shutdown...")
        super().on_cleanup(state)
        self.get_logger().info("[yolo_depth_fusion_node] Shutdown complete.")
        return TransitionCallbackReturn.SUCCESS

    def fusion_cb(self, detections_msg: DetectionArray, depth_msg: Image):
        cv_depth = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        fused_msg = DetectionArray()
        fused_msg.header.stamp = detections_msg.header.stamp
        fused_msg.header.frame_id = detections_msg.header.frame_id

        for det in detections_msg.detections:
            det_msg = Detection()
            det_msg.class_id = det.class_id
            det_msg.class_name = det.class_name
            det_msg.id = det.id
            det_msg.score = round(det.score,2)

            det_msg.bbox = det.bbox

            bbox = det.bbox
            cx = int(bbox.center.position.x)
            cy = int(bbox.center.position.y)            
            
            if 0 <= cy < cv_depth.shape[0] and 0 <= cx < cv_depth.shape[1]:
                raw_depth = cv_depth[cy, cx, 1] if cv_depth.ndim == 3 else cv_depth[cy, cx]
                if np.isnan(raw_depth) or np.isinf(raw_depth):
                    depth_val = 0.0
                else:
                    depth_val = round(float(raw_depth),2)
            else:
                depth_val = 0.0

            det_msg.depth = float(depth_val)

            X = (cx - self.cx) * depth_val / self.fx
            Y = (cy - self.cy) * depth_val / self.fy
            Z = depth_val

            det_msg.point_3d.x = round(X,2)
            det_msg.point_3d.y = round(Y,2)
            det_msg.point_3d.z = round(Z,2)

            fused_msg.detections.append(det_msg)
        self._pub_fused.publish(fused_msg)


def main():
    rclpy.init()
    node = YoloDepthNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
