from __future__ import annotations

import rospy
import cv2_img
import numpy as np

from PIL import Image
import tensorflow as tf
from tf_bodypix.api import download_model, load_model, BodyPixModelPaths

from sensor_msgs.msg import Image as SensorImage
from lasr_vision_msgs.msg import BodyPixMask, BodyPixPose
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
            model = load_model('/home/rexy/.keras/tf-bodypix/3fe1b130a0f20e98340612c099b50c18--tfjs-models-savedmodel-bodypix-resnet50-float-model-stride16')
            # model = load_model(download_model(BodyPixModelPaths.RESNET50_FLOAT_STRIDE_16))
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
    # rospy.logwarn(str(type(img)))
    img_height, img_width, _ = img.shape  # Get image dimensions

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

    # construct poses response and neck coordinates
    poses = result.get_poses()
    rospy.loginfo(str(poses))

    neck_coordinates = []
    for pose in poses:
        left_shoulder_keypoint = pose.keypoints.get(5)  # 5 is the typical index for left shoulder
        right_shoulder_keypoint = pose.keypoints.get(6)  # 6 is the typical index for right shoulder

        if left_shoulder_keypoint and right_shoulder_keypoint:
            # If both shoulders are detected, calculate neck as midpoint
            left_shoulder = left_shoulder_keypoint.position
            right_shoulder = right_shoulder_keypoint.position
            neck_x = (left_shoulder.x + right_shoulder.x) / 2
            neck_y = (left_shoulder.y + right_shoulder.y) / 2
            neck_score = (left_shoulder_keypoint.score + right_shoulder_keypoint.score) / 2  # Optional: average score
        elif left_shoulder_keypoint:
            # Only left shoulder detected, use it as neck coordinate
            left_shoulder = left_shoulder_keypoint.position
            neck_x = left_shoulder.x
            neck_y = left_shoulder.y
            neck_score = left_shoulder_keypoint.score
        elif right_shoulder_keypoint:
            # Only right shoulder detected, use it as neck coordinate
            right_shoulder = right_shoulder_keypoint.position
            neck_x = right_shoulder.x
            neck_y = right_shoulder.y
            neck_score = right_shoulder_keypoint.score

        # # Convert neck coordinates to relative positions (0-1)
        # rel_neck_x = neck_x / img_width
        # rel_neck_y = neck_y / img_height

        # pose = BodyPixPose()
        # pose.coord = np.array([rel_neck_x, rel_neck_y]).astype(np.float32)
        # neck_coordinates.append(pose)

        pose = BodyPixPose()
        pose.coord = np.array([neck_x, neck_y]).astype(np.int32)
        neck_coordinates.append(pose)
    
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
    response.poses = neck_coordinates
    return response
