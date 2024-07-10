from __future__ import annotations

from typing import List
import rospy
import cv2
import numpy as np

import re
import tensorflow as tf

import cv2_img

from sensor_msgs.msg import Image as SensorImage
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths

from lasr_vision_msgs.msg import BodyPixMask, BodyPixKeypoint, BodyPixKeypointNormalized
from lasr_vision_msgs.srv import (
    BodyPixMaskDetectionRequest,
    BodyPixMaskDetectionResponse,
    BodyPixKeypointDetectionRequest,
    BodyPixKeypointDetectionResponse,
)

import rospkg

# model cache
loaded_models = {}
r = rospkg.RosPack()


def snake_to_camel(snake_str):
    components = snake_str.split("_")
    return components[0] + "".join(x.title() for x in components[1:])


def camel_to_snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()


def load_model_cached(dataset: str):
    """
    Load a model into cache
    """
    model = None
    if dataset in loaded_models:
        rospy.loginfo(f"Using cached {dataset} model")
        model = loaded_models[dataset]
    else:
        if dataset == "resnet50":
            rospy.loginfo("Downloading resnet50 model")
            name = download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16)
            rospy.loginfo("Loading resnet50 model")
            model = load_model(name)
        elif dataset == "mobilenet50":
            rospy.loginfo("Downloading mobilenet50 model")
            name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_8)
            rospy.loginfo("Loading mobilenet50 model")
            model = load_model(name)
        elif dataset == "mobilenet100":
            rospy.loginfo("Downloading mobilenet100 model")
            name = download_model(BodyPixModelPaths.MOBILENET_FLOAT_100_STRIDE_8)
            rospy.loginfo("Loading mobilenet100 model")
            model = load_model(name)
        else:
            model = load_model(dataset)
        rospy.loginfo(f"Loaded {dataset} model into cache")
        loaded_models[dataset] = model
    return model


def run_inference(dataset: str, confidence: float, img: SensorImage):
    # decode the image
    rospy.loginfo("Decoding")
    img = cv2_img.msg_to_cv2_img(img)

    # load model
    rospy.loginfo("Loading model")
    model = load_model_cached(dataset)

    # run inference
    rospy.loginfo("Running inference")
    result = model.predict_single(img)

    mask = result.get_mask(threshold=confidence)
    rospy.loginfo("Inference complete")

    return result, mask


def detect_masks(
    request: BodyPixMaskDetectionRequest,
    debug_publisher: rospy.Publisher = rospy.Publisher(
        "/bodypix/mask_debug", SensorImage, queue_size=1
    ),
) -> BodyPixMaskDetectionResponse:
    """
    Run BodyPix inference on given mask detection request
    """

    result, mask = run_inference(request.dataset, request.confidence, request.image_raw)
    # construct masks response
    masks = []

    # This uses this list of parts:
    # https://github.com/de-code/python-tf-bodypix/blob/develop/tf_bodypix/bodypix_js_utils/part_channels.py#L5

    for part_name in request.parts:
        part_mask = result.get_part_mask(
            mask=tf.identity(mask), part_names=[part_name]
        ).squeeze()

        if np.max(part_mask) == 0:
            rospy.logwarn(f"No masks found for part {part_name}")
            continue

        bodypix_mask = BodyPixMask()
        bodypix_mask.mask = part_mask.flatten().astype(bool).tolist()
        bodypix_mask.shape = list(part_mask.shape)
        bodypix_mask.part_name = part_name
        masks.append(bodypix_mask)

    # publish to debug topic
    if debug_publisher is not None:
        # create coloured mask with poses
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

    response = BodyPixMaskDetectionResponse()
    response.masks = masks
    return response


def detect_keypoints(
    request: BodyPixKeypointDetectionRequest,
    debug_publisher: rospy.Publisher = rospy.Publisher(
        "/bodypix/keypoint_debug", SensorImage, queue_size=1
    ),
) -> BodyPixKeypointDetectionResponse:

    result, mask = run_inference(request.dataset, request.confidence, request.image_raw)

    poses = result.get_poses()

    detected_keypoints: List[BodyPixKeypoint] = []
    detected_keypoints_normalized: List[BodyPixKeypointNormalized] = []

    for pose in poses:
        for keypoint in pose.keypoints.values():
            # Check if keypoint is in the mask
            x = int(keypoint.position.x)
            y = int(keypoint.position.y)
            try:
                # if mask[y, x] == 0:
                #     continue
                if not request.keep_out_of_bounds:
                    if x < 0.0 or y < 0.0:
                        continue
                    if x >= mask.shape[1] or y >= mask.shape[0]:
                        continue
            # Throws an error if the keypoint is out of bounds
            # but not clear what type (some TF stuff)
            except:
                continue
            rospy.loginfo(f"Keypoint {keypoint.part} at {x}, {y} is in mask")
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

    # publish to debug topic
    if debug_publisher is not None:
        # create coloured mask with poses
        from tf_bodypix.draw import draw_poses

        coloured_mask = result.get_colored_part_mask(mask).astype(np.uint8)
        coloured_mask = draw_poses(
            coloured_mask.copy(),
            poses,
            keypoints_color=(255, 100, 100),
            skeleton_color=(100, 100, 255),
        )

        # Add text of keypoints to image
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

    return BodyPixKeypointDetectionResponse(
        keypoints=detected_keypoints, normalized_keypoints=detected_keypoints_normalized
    )
