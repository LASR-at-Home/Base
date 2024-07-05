#!/usr/bin/env python3
import rospy
from typing import List
from lasr_vision_clip.clip_utils import load_model, encode_img
from lasr_vision_msgs.srv import (
    ClipImageEncoder,
    ClipImageEncoderResponse,
    ClipImageEncoderRequest,
)
from sensor_msgs.msg import Image
from cv2_img import msg_to_cv2_img


class EncoderService:
    def __init__(self, model_device: str = "cuda") -> None:
        """Caches the clip model.

        Args:
            model_device (str, optional): device to load model onto. Defaults to "cuda".

        """

        self._model = load_model(model_device)
        self._debug_pub = rospy.Publisher(
            "/clip/img_encoder/debug", Image, queue_size=1
        )
        rospy.loginfo("Clip encoder service started")

    def encode_image(
        self, request: ClipImageEncoderRequest
    ) -> ClipImageEncoderResponse:
        """Encodes a given image to a vector.

        Returns:
            ClipImageEncoderResponse: the encoded vector
        """
        raw_image = request.image_raw
        encoded_vector = encode_img(self._model, raw_image)
        encoded_vector = encoded_vector.flatten()
        return ClipImageEncoderResponse(encoded_vector=encoded_vector.tolist())


if __name__ == "__main__":
    rospy.init_node("clip_vqa_service")
    service = EncoderService()
    rospy.Service("/clip/img_encoder", ClipImageEncoder, service.encode_image)
    rospy.spin()
