import cv2
from typing import List, Dict
from cv_bridge import CvBridge
import numpy as np
import matplotlib
from PIL import Image as PILImage


import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState
from transformers import pipeline

from sensor_msgs.msg import Image
import torch

class DepthNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("depth_node")

        self.declare_parameter("model", "depth-anything/Depth-Anything-V2-Small-hf")
        self.declare_parameter("device", "cuda:0")

        self.declare_parameter("enable", True)

        self.declare_parameter("imgsz_height", 640)
        self.declare_parameter("imgsz_width", 640)
        
        self.declare_parameter("grayscale", False)
        self.declare_parameter("colormap", "Spectral")

        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT.value)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.enable = self.get_parameter("enable").get_parameter_value().bool_value
        self.reliability = self.get_parameter("image_reliability").get_parameter_value().integer_value
        self.grayscale = self.get_parameter("grayscale").get_parameter_value().bool_value
        self.colormap = self.get_parameter("colormap").get_parameter_value().string_value

        # detection pub
        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._pub = self.create_lifecycle_publisher(Image, "depth_estimation", 10)
        self.cv_bridge = CvBridge()

        super().on_configure(state)
        self.get_logger().info(f"[{self.get_name()}] Configured")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Activating...")

        self.depth_pipe = pipeline(
            task="depth-estimation",
            model=self.model,
            device=self.device
        )

        self._sub = self.create_subscription(
            Image, 'image_raw', self.image_cb, self.image_qos_profile
        )

        super().on_activate(state)
        self.get_logger().info(f"[{self.get_name()}] Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Deactivating...")

        del self.depth_pipe
        if "cuda" in self.device:
            self.get_logger().info("Clearing CUDA cache")
            torch.cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)
        self.get_logger().info(f"[{self.get_name()}] Deactivated")
        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Cleaning up...")

        self.destroy_publisher(self._pub)
        del self.image_qos_profile

        super().on_cleanup(state)
        self.get_logger().info(f"[{self.get_name()}] Cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Shutting down...")


        super().on_shutdown(state)
        self.get_logger().info(f"[{self.get_name()}] Shutted down")
        return TransitionCallbackReturn.SUCCESS
    
    def image_cb(self, msg: Image) -> None:
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

        result = self.depth_pipe(pil_image)
        depth = np.array(result["depth"])

        depth_norm = (depth - depth.min()) / (depth.max() - depth.min() + 1e-8) * 255.0
        depth_norm = depth_norm.astype(np.uint8)

        if self.grayscale:
            depth_img = self.cv_bridge.cv2_to_imgmsg(depth_norm, encoding="mono8")
           
        else:
            cmap = matplotlib.colormaps.get_cmap(self.colormap)
            depth_color = (cmap(depth_norm)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
            depth_img = self.cv_bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")

        depth_img.header = msg.header
        self._pub.publish(depth_img)

        del cv_image, pil_image, depth, depth_norm


def main():
    rclpy.init()
    node = DepthNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
