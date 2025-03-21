import rclpy
from rclpy.node import Node
import cv2_img
from ultralytics import YOLO

from lasr_vision_interfaces.msg import Detection, Detection3D
from lasr_vision_interfaces.srv import YoloDetection  # , YoloDetection3D

import tf2_ros as tf

# import tf2_sensor_msgs  # noqa

# TODO address 3D Detection later on (check ROS1 implementation)

# global tf buffer
tf_buffer = tf.Buffer()  # cache_time=rospy.Duration(10) needed for 3D


def start_tf_buffer() -> None:
    tf.TransformListener(tf_buffer, AccessNode.get_node())


# model cache
loaded_models = {}


def load_model(dataset: str) -> None:
    """
    Load a model into cache
    """
    node = AccessNode.get_node()
    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        model = YOLO(dataset)
        node.get_logger().info(f"Loaded {dataset} model")

        loaded_models[dataset] = model

    return model


def detect(
    request: YoloDetection.Request(), debug_publisher: rclpy.node.Publisher
) -> YoloDetection.Response():
    """
    Run YOLO inference on given detection request
    """
    node = AccessNode.get_node()

    # decode the image
    node.get_logger().info("Decoding")
    img = cv2_img.msg_to_pillow_img(request.image_raw)

    # load model
    node.get_logger().info("Loading model")
    model = load_model(request.dataset)

    # run inference
    node.get_logger().info("Running inference")
    results = model(img, conf=request.confidence, iou=request.nms)
    result = results[0]
    node.get_logger().info("Inference complete")

    # construct response
    detected_objects = []
    object_count = result.boxes.cls.size(dim=0)
    has_segment_masks = result.masks is not None
    for i in range(0, object_count):
        detection = Detection()
        detection.name = result.names[int(result.boxes.cls[i])]
        detection.confidence = float(result.boxes.conf.cpu().numpy()[i])
        detection.xywh = result.boxes.xywh[i].cpu().numpy().astype(int).tolist()

        # copy segmented mask if available
        if has_segment_masks:
            detection.xyseg = result.masks.xy[i].flatten().astype(int).tolist()

        detected_objects.append(detection)

    # publish to debug topic
    debug_publisher.publish(cv2_img.cv2_img_to_msg(result.plot()))

    response = YoloDetection.Response()
    response.detected_objects = detected_objects
    return response


class AccessNode(Node):
    """
    Class to  create and access the node to avoid duplications
    """

    _node = None  # Static variable to hold the node instance

    @staticmethod
    def get_node():
        """Returns the singleton ROS 2 node instance, creating it if necessary."""
        if AccessNode._node is None:
            AccessNode._node = Node("yolo_access_node")
        return AccessNode._node

    @staticmethod
    def shutdown():
        """Shuts down the singleton node properly."""
        if AccessNode._node is not None:
            AccessNode._node.destroy_node()
            AccessNode._node = None
            AccessNode.shutdown()
