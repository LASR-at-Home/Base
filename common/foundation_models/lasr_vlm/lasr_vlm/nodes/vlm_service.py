#!/usr/bin/env python3
import os
import tempfile

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from lasr_vlm_interfaces.srv import VlmDescribePeople
from lasr_vlm.vlm_inference import (
    ModelConfig,
    VLMInference,
    visually_describe_people,
)


class VlmDescribePeopleService(Node):
    """
    ROS 2 service node that wraps VLM inference to visually describe people.
    Receives a ROS image, runs it through the VLM, and returns the attributes.
    """

    def __init__(self):
        super().__init__("vlm_describe_people_service")

        self.create_service(
            VlmDescribePeople,
            "/vlm/describe_people",
            self.describe_people_callback,
        )

        model_config = ModelConfig(model_name="moondream")
        self.vlm = VLMInference(model_config, new_model=False)
        self.get_logger().info("VLM Describe People service started")

    def _image_msg_to_bgr8(self, image_msg):
        """Convert a ROS Image message to an OpenCV BGR image without cv_bridge."""
        if image_msg.encoding not in ("bgr8", "rgb8", "mono8"):
            raise ValueError(f"Unsupported image encoding: {image_msg.encoding}")

        image = np.frombuffer(image_msg.data, dtype=np.uint8)

        if image_msg.encoding == "mono8":
            image = image.reshape((image_msg.height, image_msg.width))
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        image = image.reshape((image_msg.height, image_msg.width, 3))
        if image_msg.encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        return image

    def describe_people_callback(self, request, response):
        """
        Handle incoming service requests.
        Converts the ROS image to a file, runs VLM inference, and returns attributes.
        """
        self.get_logger().info("Received request to describe person")

        try:
            cv_image = self._image_msg_to_bgr8(request.image_raw)

            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as f:
                tmp_path = f.name
            cv2.imwrite(tmp_path, cv_image)

            result = visually_describe_people(
                input_image=tmp_path,
                inference=self.vlm,
            )

            os.unlink(tmp_path)

            def _get(key, default):

                val = result.get(key, [default])

                return val[0] if isinstance(val, list) and val else default

            response.hair_color = str(_get("hair_color", "unknown"))

            response.hair_length = str(_get("hair_length", "unknown"))

            response.glasses = bool(_get("glasses", False))

            response.hat = bool(_get("hat", False))

            response.shirt_color = str(_get("shirt color", "unknown"))

            self.get_logger().info(f"VLM result: {result}")

        except Exception as e:
            self.get_logger().error(f"Failed to describe person: {e}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VlmDescribePeopleService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
