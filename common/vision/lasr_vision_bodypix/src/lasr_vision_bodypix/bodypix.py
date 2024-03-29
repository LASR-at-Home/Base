from __future__ import annotations

import rospy
import cv2_img
import numpy as np

from PIL import Image
import tensorflow as tf
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths

from sensor_msgs.msg import Image as SensorImage
from lasr_vision_msgs.msg import BodyPixMask
from lasr_vision_msgs.srv import BodyPixDetectionRequest, BodyPixDetectionResponse

# model cache
loaded_models = {}

def load_model_cached(dataset: str) -> None:
    '''
    Load a model into cache
    '''

    model = None
    if dataset in loaded_models:
        model = loaded_models[dataset]
    else:
        if dataset == 'resnet50':
            model = load_model(download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16))
        elif dataset == 'mobilenet50':
            model = load_model(download_model(BodyPixModelPaths.MOBILENET_FLOAT_50_STRIDE_16))
        else:
            model = load_model(dataset)

        rospy.loginfo(f'Loaded {dataset} model')
        loaded_models[dataset] = model
    
    return model

def detect(request: BodyPixDetectionRequest, debug_publisher: rospy.Publisher | None) -> BodyPixDetectionResponse:
    '''
    Run BodyPix inference on given detection request
    '''

    # decode the image
    rospy.loginfo('Decoding')
    img = cv2_img.msg_to_cv2_img(request.image_raw)

    # load model
    rospy.loginfo('Loading model')
    model = load_model_cached(request.dataset)

    # run inference
    rospy.loginfo('Running inference')
    result = model.predict_single(img)
    mask = result.get_mask(threshold=request.confidence)
    rospy.loginfo('Inference complete')

    # construct masks response
    masks = []
    for mask_request in request.masks:
        part_mask = result.get_part_mask(mask=tf.identity(mask), part_names=mask_request.parts).squeeze()

        bodypix_mask = BodyPixMask()
        bodypix_mask.mask = part_mask.flatten().astype(bool)
        bodypix_mask.shape = list(part_mask.shape)
        masks.append(bodypix_mask)

    # construct pose response
    # TODO
    
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
    
    response = BodyPixDetectionResponse()
    response.masks = masks
    return response
