#!/usr/bin/env python3
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():

    def run_yolo_depth(context: LaunchContext, use_tracking):

        use_tracking = eval(context.perform_substitution(use_tracking))

        det_model = LaunchConfiguration("det_model")
        det_model_cmd = DeclareLaunchArgument(
            "det_model", default_value="yolov8m.pt", description="YOLO detection model path"
        )

        dep_model = LaunchConfiguration("dep_model")
        dep_model_cmd = DeclareLaunchArgument(
            "dep_model", default_value="vitl", choices=["vits", "vitb", "vitl"], description="small, base, large depth model"
        )

        dep_model_path = LaunchConfiguration("dep_model_path")
        dep_model_path_cmd = DeclareLaunchArgument(
            "dep_model", default_value="depth-anything/Depth-Anything-V2-Small-hf", description="Depth estimation model path"
        )

        device = LaunchConfiguration("device")
        device_cmd = DeclareLaunchArgument(
            "device", default_value="cuda:0", description="Device to use (GPU/CPU)"
        )

        enable = LaunchConfiguration("enable")
        enable_cmd = DeclareLaunchArgument(
            "enable", default_value="True", description="Enable the nodes")

        threshold = LaunchConfiguration("threshold")
        threshold_cmd = DeclareLaunchArgument(
            "threshold", default_value="0.5", description="Minimum probability of a detection to be published")

        iou = LaunchConfiguration("iou")
        iou_cmd = DeclareLaunchArgument("iou", default_value="0.7", description="IoU threshold")

        input_image_topic = LaunchConfiguration("input_image_topic")
        input_image_topic_cmd = DeclareLaunchArgument(
            "input_image_topic", default_value="/camera/rgb/image_raw", description="Name of the input image topic",)

        imgsz_height = LaunchConfiguration("imgsz_height")
        imgsz_height_cmd = DeclareLaunchArgument(
            "imgsz_height", default_value="640", description="Input image height"
        )

        imgsz_width = LaunchConfiguration("imgsz_width")
        imgsz_width_cmd = DeclareLaunchArgument(
            "imgsz_width", default_value="640", description="Input image width"
        )

        half = LaunchConfiguration("half")
        half_cmd = DeclareLaunchArgument(
            "half", default_value="False", description="Whether to enable half-precision (FP16) inference speeding up model inference with minimal impact on accuracy")

        max_depth = LaunchConfiguration("max_depth")
        max_depth_cmd = DeclareLaunchArgument(
            "max_depth", default_value="50", description="Maximum number of depth allowed per image")

        max_det = LaunchConfiguration("max_det")
        max_det_cmd = DeclareLaunchArgument(
            "max_det", default_value="300", description="Maximum number of detections allowed per image")

        augment = LaunchConfiguration("augment")
        augment_cmd = DeclareLaunchArgument(
            "augment", default_value="False", description="Whether to enable test-time augmentation (TTA) for predictions improving detection robustness at the cost of speed")

        agnostic_nms = LaunchConfiguration("agnostic_nms")
        agnostic_nms_cmd = DeclareLaunchArgument(
            "agnostic_nms", default_value="False", description="Whether to enable class-agnostic Non-Maximum Suppression (NMS) merging overlapping boxes of different classes")

        image_reliability = LaunchConfiguration("image_reliability")
        image_reliability_cmd = DeclareLaunchArgument(
            "image_reliability", default_value="1", choices=["0", "1", "2"], description="Image QoS reliability (0=system default, 1=Reliable, 2=Best Effort)")

        fx = LaunchConfiguration("fx")
        fx_cmd = DeclareLaunchArgument(
            "fx", default_value="1000.0", description="Camera focal length in the x-axis (fx)"
        )

        fy = LaunchConfiguration("fy")
        fy_cmd = DeclareLaunchArgument(
            "fy", default_value="1800.0", description="Camera focal length in the y-axis (fy)"
        )

        cx = LaunchConfiguration("cx")
        cx_cmd = DeclareLaunchArgument(
            "cx", default_value="319.5", description="Camera principal point x-coordinate (cx)"
        )
        
        cy = LaunchConfiguration("cy")
        cy_cmd = DeclareLaunchArgument(
            "cy", default_value="319.5", description="Camera principal point y-coordinate (cy)"
        )

        namespace = LaunchConfiguration("namespace")
        namespace_cmd = DeclareLaunchArgument(
            "namespace", default_value="yolo_depth", description="Namespace for the nodes"
        )

        
        detections_topic = "detections"

        if use_tracking:
            detections_topic = "tracking"

        yolo_node_cmd = Node(
            package="yolo_depth_ros",
            executable="yolo_node",
            name="yolo_node",
            namespace=namespace,
            parameters=[
                {
                    "model": det_model,
                    "device": device,
                    "enable": enable,
                    "imgsz_height": imgsz_height,
                    "imgsz_width": imgsz_width,
                    "threshold": threshold,
                    "iou": iou,
                    "half": half,
                    "max_det": max_det,
                    "augment": augment,
                    "agnostic_nms": agnostic_nms,
                    "image_reliability": image_reliability
                }
            ],
            remappings=[("image_raw", input_image_topic)],
        )


        tracking_node_cmd = Node(
            package="yolo_depth_ros",
            executable="tracking_node",
            name="tracking_node",
            namespace=namespace,
            parameters=[
                {
                    "tracker": "bytetrack.yaml",
                    "image_reliability": image_reliability
                }
            ],
            remappings=[("image_raw", input_image_topic)],
            condition=IfCondition(PythonExpression([str(use_tracking)])),
        )


        depth_node_cmd = Node(
            package="yolo_depth_ros",
            executable="depth_node",
            name="depth_node",
            namespace=namespace,
            parameters=[
                {
                    "model": dep_model,
                    "model_path": dep_model_path,
                    "device": device,
                    "enable": enable,
                    "max_depth": max_depth,
                    "imgsz_height": imgsz_height,
                    "imgsz_width": imgsz_width,
                    "image_reliability": image_reliability,
                }
            ],
            remappings=[("image_raw", input_image_topic)],
        )

        fusion_node_cmd = Node(
            package="yolo_depth_ros",
            executable="yolo_depth_node",
            name="yolo_depth_node",
            namespace=namespace,
            parameters=[
                {
                    "image_reliability": image_reliability,
                    "fx": fx,
                    "fy": fy,
                    "cx": cx,
                    "cy": cy,
                }
            ],
            remappings=[("detections", detections_topic)],
        )

        debug_node_cmd = Node(
            package="yolo_depth_ros",
            executable="debug_node",
            name="debug_node",
            namespace=namespace,
            parameters=[
                {
                    "image_reliability": image_reliability,
                }
            ],
            remappings=[("image_raw", input_image_topic)]
        )

        return (
            det_model_cmd,
            dep_model_cmd,
            dep_model_path_cmd,
            device_cmd,
            enable_cmd,
            threshold_cmd,
            iou_cmd,
            input_image_topic_cmd,
            imgsz_height_cmd,
            imgsz_width_cmd,
            half_cmd,
            max_depth_cmd,
            max_det_cmd,
            augment_cmd,
            agnostic_nms_cmd,
            image_reliability_cmd,
            fx_cmd,
            fy_cmd,
            cx_cmd,
            cy_cmd,
            namespace_cmd,
            yolo_node_cmd,
            tracking_node_cmd,
            depth_node_cmd,
            fusion_node_cmd,
            debug_node_cmd,
        )


    use_tracking = LaunchConfiguration("use_tracking")
    use_tracking_cmd = DeclareLaunchArgument(
        "use_tracking",
        default_value="True",
        description="Whether to activate tracking",
    )

    return LaunchDescription(
        [
            use_tracking_cmd,
            OpaqueFunction(function=run_yolo_depth, args=[use_tracking]),
        ]
    )