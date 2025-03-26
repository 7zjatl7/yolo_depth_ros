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
# from transformers import pipeline

from sensor_msgs.msg import Image
import torch

from .depth_anything_v2.dpt import DepthAnythingV2



class DepthNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("depth_node")

        self.declare_parameter("model", "vitb")
        self.declare_parameter("model_path", "depth_anything_vitb.pth")
        self.declare_parameter("device", "cuda:0")

        self.declare_parameter("enable", True)

        self.declare_parameter("imgsz_height", 640)
        self.declare_parameter("imgsz_width", 640)
        
        self.declare_parameter("max_depth", 50)

        self.declare_parameter("image_reliability", QoSReliabilityPolicy.BEST_EFFORT.value)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"[{self.get_name()}] Configuring...")

        self.model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        self.model = self.get_parameter("model").get_parameter_value().string_value
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.enable = self.get_parameter("enable").get_parameter_value().bool_value
        self.reliability = self.get_parameter("image_reliability").get_parameter_value().integer_value
        self.imgsz_height = self.get_parameter("imgsz_height").get_parameter_value().integer_value
        self.imgsz_width = self.get_parameter("imgsz_width").get_parameter_value().integer_value
        self.max_depth = self.get_parameter("max_depth").get_parameter_value().integer_value


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

        depth_anything = DepthAnythingV2(**{**self.model_configs[self.model], 'max_depth':self.max_depth})

        ckpt = torch.load(self.model_path, map_location='cpu')
        state_dict = ckpt['model'] if 'model' in ckpt else ckpt
        depth_anything.load_state_dict(state_dict)
        self.depth_anything = depth_anything.to(self.device).eval()

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

        depth = self.depth_anything.infer_image(cv_image, self.imgsz_height)

        # depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        # depth = depth.astype(np.uint8)

        depth_img = self.cv_bridge.cv2_to_imgmsg(depth, encoding="32FC1")


        depth_img.header = msg.header
        self._pub.publish(depth_img)

        del cv_image, depth


def main():
    rclpy.init()
    node = DepthNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
