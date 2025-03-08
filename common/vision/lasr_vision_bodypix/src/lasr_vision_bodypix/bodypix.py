from __future__ import annotations

import re
from typing import List

import cv2
import cv2_img
import numpy as np
import tensorflow as tf
from lasr_vision_interfaces.msg import (
    BodyPixMask,
    BodyPixKeypoint,
    BodyPixKeypointNormalized,
)
from lasr_vision_interfaces.srv import (
    BodyPixMaskDetection,
    BodyPixKeypointDetection,
)
from sensor_msgs.msg import Image as SensorImage
from tf_bodypix.api import download_model, BodyPixModelPaths
from tf_bodypix.api import load_model as bp_load_model

BodyPixKeypointDetection_Request = BodyPixKeypointDetection.Request()
BodyPixKeypointDetection_Response = BodyPixKeypointDetection.Response()
BodyPixMaskDetection_Request = BodyPixMaskDetection.Request()
BodyPixMaskDetection_Response = BodyPixMaskDetection.Response()

# model cache
loaded_models = {}


def snake_to_camel(snake_str):
    components = snake_str.split("_")
    return components[0] + "".join(x.title() for x in components[1:])


def camel_to_snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


# def load_model_cached(dataset: str):
#     """
#     Load a model into cache
#     """
#     model = None
#     if dataset in loaded_models:
#         model = loaded_models[dataset]
#     else:
#         if dataset == "resnet50":
#             name = download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
#             model = load_model(name)
#         elif dataset == "mobilenet50":
#             name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_8)
#             model = load_model(name)
#         elif dataset == "mobilenet100":
#             name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_100_STRIDE_8)
#             model = load_model(name)
#         else:
#             model = load_model(dataset)
#         loaded_models[dataset] = model
#     return model


def load_model():
    """
    Load resnet 50 model.
    """
    name = download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
    model = bp_load_model(name)
    return model


# def run_inference(dataset: str, confidence: float, img: SensorImage, logger=None):
def run_inference(confidence: float, img: SensorImage, logger=None):
    """
    Run inference on an image.
    """
    # Decode the image
    if logger:
        logger.info("Decoding")
    img = cv2_img.msg_to_cv2_img(img)

    # Load model
    if logger:
        logger.info("Loading model")
    # model = load_model_cached(dataset)
    model = load_model()

    # Run inference
    if logger:
        logger.info("Running inference")
    result = model.predict_single(img)

    mask = result.get_mask(threshold=confidence)
    if logger:
        logger.info("Inference complete")

    return result, mask


def detect_masks(
    request: BodyPixMaskDetection_Request, debug_publisher=None, logger=None
):
    """
    Run BodyPix inference for mask detection.
    """
    result, mask = run_inference(
        # request.dataset, request.confidence, request.image_raw, logger
        request.confidence, request.image_raw, logger
    )

    masks = []

    for part_name in request.parts:
        part_mask = result.get_part_mask(
            mask=tf.identity(mask), part_names=[part_name]
        ).squeeze()

        if np.max(part_mask) == 0:
            if logger:
                logger.warning(f"No masks found for part {part_name}")
            continue

        bodypix_mask = BodyPixMask()
        bodypix_mask.mask = part_mask.flatten().astype(bool).tolist()
        bodypix_mask.shape = list(part_mask.shape)
        bodypix_mask.part_name = part_name
        masks.append(bodypix_mask)

    # Publish debug visualization if enabled
    if debug_publisher:
        from tf_bodypix.draw import draw_poses

        coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
        poses = result.get_poses()
        coloured_mask = draw_poses(
            coloured_mask.copy(),
            poses,
            keypoints_color=(255, 100, 100),
            skeleton_color=(100, 100, 255),
        )
        debug_publisher.publish(cv2_img.cv2_img_to_msg(coloured_mask))

    response = BodyPixMaskDetection_Response
    response.masks = masks
    return response


def detect_keypoints(
    request: BodyPixKeypointDetection_Request, debug_publisher=None, logger=None
):
    """
    Run BodyPix inference for keypoint detection.
    """
    result, mask = run_inference(
        # request.dataset, request.confidence, request.image_raw, logger
        request.confidence, request.image_raw, logger
    )

    poses = result.get_poses()
    detected_keypoints: List[BodyPixKeypoint] = []
    detected_keypoints_normalized: List[BodyPixKeypointNormalized] = []

    for pose in poses:
        for keypoint in pose.keypoints.values():
            x = int(keypoint.position.x)
            y = int(keypoint.position.y)
            try:
                if not request.keep_out_of_bounds:
                    if x < 0.0 or y < 0.0:
                        continue
                    if x >= mask.shape[1] or y >= mask.shape[0]:
                        continue
            except:
                continue

            if logger:
                logger.info(f"Keypoint {keypoint.part} at {x}, {y} is in mask")

            detected_keypoints.append(
                BodyPixKeypoint(keypoint_name=keypoint.part, x=x, y=y)
            )
            detected_keypoints_normalized.append(
                BodyPixKeypointNormalized(
                    keypoint_name=keypoint.part,
                    x=float(x) / mask.shape[1],
                    y=float(y) / mask.shape[0],
                )
            )

    # Publish debug visualization if enabled
    if debug_publisher:
        from tf_bodypix.draw import draw_poses

        coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
        coloured_mask = draw_poses(
            coloured_mask.copy(),
            poses,
            keypoints_color=(255, 100, 100),
            skeleton_color=(100, 100, 255),
        )

        for keypoint in detected_keypoints:
            cv2.putText(
                coloured_mask,
                f"{keypoint.keypoint_name}",
                (int(keypoint.x), int(keypoint.y)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        debug_publisher.publish(cv2_img.cv2_img_to_msg(coloured_mask))

    response = BodyPixKeypointDetection_Response
    response.keypoints = detected_keypoints
    response.normalized_keypoints = detected_keypoints_normalized

    return response
