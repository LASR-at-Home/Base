#!/usr/bin/env python3

# import re
# import rospy
# import rospkg
# import lasr_vision_yolov8 as yolo

# from typing import Dict

# from sensor_msgs.msg import Image
# from visualization_msgs.msg import Marker

# from lasr_vision_msgs.srv import (
#     YoloDetection,
#     YoloDetectionRequest,
#     YoloDetectionResponse,
#     YoloDetection3D,
#     YoloDetection3DRequest,
#     YoloDetection3DResponse,
# )

# # Put ourselves in the model folder
# import os
# import rospkg

# rp = rospkg.RosPack()
# package_path = rp.get_path("lasr_vision_yolov8")
# os.chdir(os.path.abspath(os.path.join(package_path, "models")))

# # Initialise rospy
# rospy.init_node("yolov8_service")

# # Determine variables
# PRELOAD = rospy.get_param("~preload", [])

# for model in PRELOAD:
#     yolo.load_model(model)

# # Prepare publisher
# debug_publishers: Dict[str, rospy.Publisher] = {}
# debug_publisher = rospy.Publisher("/yolov8/debug", Image, queue_size=1)


# def detect(request: YoloDetectionRequest) -> YoloDetectionResponse:
#     """
#     Hand off detection request to yolo library
#     """
#     if request.dataset in debug_publishers:
#         debug_publisher = debug_publishers[request.dataset]
#     else:
#         topic_name = re.sub(r"[\W_]+", "", request.dataset)
#         debug_publisher = rospy.Publisher(
#             f"/yolov8/debug/{topic_name}", Image, queue_size=1
#         )
#     return yolo.detect(request, debug_publisher)


# def detect_3d(request: YoloDetection3DRequest) -> YoloDetection3DResponse:
#     """
#     Hand off detection request to yolo library
#     """
#     if request.dataset in debug_publishers:
#         debug_inference_publisher, debug_point_publisher = debug_publishers[
#             request.dataset
#         ]
#     else:
#         topic_name = re.sub(r"[\W_]+", "", request.dataset)
#         debug_inference_publisher = rospy.Publisher(
#             f"/yolov8/debug/{topic_name}", Image, queue_size=1
#         )
#         debug_point_publisher = rospy.Publisher(
#             f"/yolov8/debug/points", Marker, queue_size=100
#         )

#     return yolo.detect_3d(request, debug_inference_publisher, debug_point_publisher)


# yolo.start_tf_buffer()
# rospy.Service("/yolov8/detect", YoloDetection, detect)
# rospy.Service("/yolov8/detect3d", YoloDetection3D, detect_3d)
# rospy.loginfo("YOLOv8 service started")
# rospy.spin()

from typing import Dict

import os

import PIL.Image
import rospy
import rospkg

import cv2_img

import ultralytics

print(ultralytics.__version__)
import torch
import numpy as np

from lasr_vision_msgs.srv import (
    YoloDetection,
    YoloDetectionRequest,
    YoloDetectionResponse,
    YoloDetection3D,
    YoloDetection3DRequest,
    YoloDetection3DResponse,
)

from lasr_vision_msgs.msg import Detection, Detection3D


class YOLOService:

    _cache: Dict[str, ultralytics.YOLO]

    def __init__(self):

        self._cache = {}
        self._device = rospy.get_param(
            "~device", "cuda:0" if torch.cuda.is_available() else "cpu"
        )

        rospy.Service("/yolov8/detect", YoloDetection, self._detect)
        rospy.Service("/yolov8/detect3d", YoloDetection, self._detect3d)

    def _detect(self, req: YoloDetectionRequest) -> YoloDetectionResponse:
        response = YoloDetectionResponse()

        pil_img = cv2_img.msg_to_pillow_img(req.image_raw)
        results = self._yolo(pil_img, req.model, req.confidence)

        has_masks = results.masks is not None

        for result in results:

            detection = Detection()
            detection.name = result.names[result.boxes.cls.int().item()]
            detection.confidence = result.boxes.conf.item()
            detection.xywh = (
                result.boxes.xywh.round().int().squeeze().cpu().numpy().tolist()
            )

            if has_masks:
                detection.xyseg = (
                    np.array(result.masks.xy).flatten().round().astype(int).tolist()
                )

            response.detected_objects.append(detection)

        return response

    def _detect3d(self, req: YoloDetection3DRequest) -> YoloDetection3DResponse:
        response = YoloDetection3DResponse()

        pil_img = cv2_img.msg_to_pillow_img(req.image_raw)
        results = self._yolo(pil_img, req.model, req.confidence)

        return response

    def _maybe_load_model(self, model_name: str) -> ultralytics.YOLO:
        if model_name in self._cache:
            return self._cache[model_name]

        model = self._cache[model_name] = ultralytics.YOLO(model_name).to(self._device)

        rospy.loginfo(f"Loaded {model_name} model on {self._device}")
        return model

    def _yolo(self, img, model: str, conf: float) -> ultralytics.engine.results.Results:
        yolo = self._maybe_load_model(model)
        results = yolo(img, conf=conf)[0]
        return results


if __name__ == "__main__":
    rospy.init_node("yolov8")
    package_path = rospkg.RosPack().get_path("lasr_vision_yolov8")
    os.chdir(os.path.abspath(os.path.join(package_path, "models")))

    yolov8 = YOLOService()
    import cv2

    im = cv2.imread("/home/jared/robocup/Base/img.jpg")
    img_msg = cv2_img.cv2_img_to_msg(im)
    print(
        yolov8._detect(
            YoloDetectionRequest(image_raw=img_msg, model="yolo11n.pt", confidence=0.5)
        )
    )
    rospy.spin()
