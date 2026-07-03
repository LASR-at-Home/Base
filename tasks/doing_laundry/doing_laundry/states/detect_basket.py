import time
import numpy as np
import rclpy
import yasmin
import yasmin_ros
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import OpenVocabDetect
from yasmin import Blackboard


class DetectBasket(yasmin.State):
    """
    Captures image from /head_front_camera/rgb/image_raw,
    then calls open_vocab/detect with basket queries.

    Outputs:
        blackboard["basket_detection"] — Detection with highest confidence
    Outcomes:
        found     — basket detected with confidence > 0.3
        not_found — nothing detected
    """

    QUERIES = ["laundry basket", "grey basket", "basket"]
    # CONFIDENCE_THRESHOLD = 0.3
    CONFIDENCE_THRESHOLD = 0.0
    IMAGE_TOPIC = "/head_front_camera/rgb/image_raw"

    def __init__(self):
        super().__init__(
            outcomes=["found", "not_found"],
        )
        self._client = None
        self._node = None

    def _setup(self):
        if self._node is None:
            self._node = yasmin_ros.logger_node
        if self._client is None:
            self._client = self._node.create_client(
                OpenVocabDetect, "/open_vocab/detect"
            )

    def _capture_image(self, timeout: float = 5.0):
        """Capture a single image from the camera topic."""
        captured = []

        def cb(msg):
            if not captured:
                captured.append(msg)

        sub = self._node.create_subscription(
            Image, self.IMAGE_TOPIC, cb, 1
        )

        start = time.time()
        while not captured and (time.time() - start) < timeout:
            time.sleep(0.1)

        self._node.destroy_subscription(sub)

        if not captured:
            yasmin.YASMIN_LOG_WARN(
                f"DetectBasket: no image received from {self.IMAGE_TOPIC}"
            )
            return None

        return captured[0]

    def execute(self, blackboard: Blackboard):
        self._setup()
        yasmin.YASMIN_LOG_INFO("DetectBasket: capturing image...")

        img_msg = self._capture_image()
        if img_msg is None:
            return "not_found"

        yasmin.YASMIN_LOG_INFO("DetectBasket: calling open_vocab/detect...")

        if not self._client.wait_for_service(timeout_sec=5.0):
            yasmin.YASMIN_LOG_WARN("DetectBasket: /open_vocab/detect not available")
            return "not_found"

        req = OpenVocabDetect.Request()
        req.image = img_msg
        req.queries = self.QUERIES
        # req.box_threshold = 0.3
        # req.text_threshold = 0.25

        req.box_threshold = 0.0
        req.text_threshold = 0.0

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=30.0)

        if future.result() is None:
            yasmin.YASMIN_LOG_WARN("DetectBasket: no response from service")
            return "not_found"

        detections = future.result().detections
        if not detections:
            yasmin.YASMIN_LOG_INFO("DetectBasket: no basket detected")
            return "not_found"

        best = max(detections, key=lambda d: d.confidence)
        yasmin.YASMIN_LOG_INFO(
            f"DetectBasket: found '{best.name}' confidence {best.confidence:.2f}"
        )

        if best.confidence < self.CONFIDENCE_THRESHOLD:
            return "not_found"

        blackboard["basket_detection"] = best
        return "found"