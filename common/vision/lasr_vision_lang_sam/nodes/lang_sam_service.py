from typing import List

import rospy
import numpy as np
import cv2_img

from PIL import Image

from lang_sam import LangSAM
from lang_sam.utils import draw_image
from lasr_vision_msgs.srv import LangSam, LangSamRequest, LangSamResponse
from lasr_vision_msgs.msg import LangSamDetection
from sensor_msgs.msg import Image as SensorImage


class LangSamService:
    def __init__(self):
        self.debug_publisher = rospy.Publisher(
            "/lasr_vision/lang_sam/debug", SensorImage, queue_size=10
        )
        self._model = LangSAM()
        self._service = rospy.Service("/lasr_vision/lang_sam", LangSam, self._lang_sam)
        rospy.loginfo("/lasr_vision/lang_sam service is ready!")

    def _lang_sam(self, request: LangSamRequest) -> LangSamResponse:
        """Processes a given input image and text prompt to return a list of
        segmentation and bboxes using Grounded SAM.

        Args:
            request (LangSamRequest): Request with fields:
                - image_raw (sensor_msgs/Image): The input image to process.
                - prompt (str): The text prompt to guide the segmentation.
        Returns:
            LangSamResponse: Response with fields:
                - detections (List[LangSamDetection]): List of detected objects with their bounding boxes
                and segmentation masks.
        """

        prompt = request.prompt
        sensor_image = request.image_raw

        # Convert sensor_msgs/Image to PIL Image
        cv_im = cv2_img.msg_to_cv2_img(sensor_image)
        pil_image = Image.fromarray(cv_im)
        print(f"Received image with size: {pil_image.size} and prompt: {prompt}")
        results = self._model.predict(
            [pil_image], [prompt], box_threshold=0.3, text_threshold=0.25
        )
        print(results)

        response_results: List[LangSamDetection] = []

        for result in results:
            no_of_detections = len(result["masks"])
            if no_of_detections == 0:
                rospy.logwarn("No detections found.")
                continue
            image_arr = np.array(pil_image)
            debug_image = draw_image(
                image_arr,
                result["masks"],
                result["boxes"],
                result["scores"],
                result["labels"],
            )
            debug_image_msg = cv2_img.cv2_img_to_msg(debug_image)
            self.debug_publisher.publish(debug_image_msg)
            for det in range(no_of_detections):
                rounded_xyxy = [round(coord) for coord in result["boxes"][det]]
                # Convert xyxy to xywh format
                xywh = [
                    rounded_xyxy[0],
                    rounded_xyxy[1],
                    rounded_xyxy[2] - rounded_xyxy[0],
                    rounded_xyxy[3] - rounded_xyxy[1],
                ]
                response_results.append(
                    LangSamDetection(
                        xywh=xywh,
                        seg_mask=result["masks"][det]
                        .flatten()
                        .astype(int)
                        .tolist(),  # Flatten the mask to a list
                        detection_score=result["scores"][det],
                        seg_mask_score=(
                            result["mask_scores"][det]
                            if no_of_detections > 1
                            else float(result["mask_scores"])
                        ),  # If only one detection, it returns a 0D array
                    )
                )

        response = LangSamResponse(detections=response_results)

        return response


if __name__ == "__main__":
    rospy.init_node("lasr_vision_lang_sam")
    lang_sam_service = LangSamService()
    rospy.spin()
