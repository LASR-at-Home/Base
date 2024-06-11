from lasr_vision_msgs.srv import TorchFaceFeatureDetectionDescription, TorchFaceFeatureDetectionDescriptionRequest, TorchFaceFeatureDetectionDescriptionResponse
from lasr_vision_feature_extraction.categories_and_attributes import CategoriesAndAttributes, CelebAMaskHQCategoriesAndAttributes, DeepFashion2GeneralizedCategoriesAndAttributes

from cv2_img import msg_to_cv2_img
from numpy2message import message2numpy
import numpy as np
import cv2
import torch
import rospy
import rospkg
import lasr_vision_feature_extraction
from os import path


def detect(request: TorchFaceFeatureDetectionDescriptionRequest) -> TorchFaceFeatureDetectionDescriptionRequest:
    # decode the image
    rospy.loginfo('Decoding')
    full_frame = msg_to_cv2_img(request.image_raw)
    torso_mask_data, torso_mask_shape, torso_mask_dtype = request.torso_mask_data, request.torso_mask_shape, request.torso_mask_dtype
    head_mask_data, head_mask_shape, head_mask_dtype = request.head_mask_data, request.head_mask_shape, request.head_mask_dtype 
    torso_mask = message2numpy(torso_mask_data, torso_mask_shape, torso_mask_dtype)
    head_mask = message2numpy(head_mask_data, head_mask_shape, head_mask_dtype)
    head_frame = lasr_vision_feature_extraction.extract_mask_region(full_frame, head_mask.astype(np.uint8), expand_x=0.4, expand_y=0.5)
    torso_frame = lasr_vision_feature_extraction.extract_mask_region(full_frame, torso_mask.astype(np.uint8), expand_x=0.2, expand_y=0.0)
    rst_str = lasr_vision_feature_extraction.predict_frame(
        head_frame, torso_frame, full_frame, head_mask, torso_mask, head_predictor=head_predictor, cloth_predictor=cloth_predictor,
    )
    response = TorchFaceFeatureDetectionDescriptionResponse()
    response.description = rst_str
    return response


if __name__ == '__main__':
    # predictor will be global when inited, thus will be used within the function above.
    head_model = lasr_vision_feature_extraction.load_face_classifier_model()
    head_predictor = lasr_vision_feature_extraction.Predictor(head_model, torch.device('cpu'), CelebAMaskHQCategoriesAndAttributes)
    cloth_model = lasr_vision_feature_extraction.load_cloth_classifier_model()
    cloth_model.return_bbox = False  # unify returns
    cloth_predictor = lasr_vision_feature_extraction.Predictor(cloth_model, torch.device('cpu'), DeepFashion2GeneralizedCategoriesAndAttributes)
    rospy.init_node('torch_service')
    rospy.Service('/torch/detect/face_features', TorchFaceFeatureDetectionDescription, detect)
    rospy.loginfo('Torch service started')
    rospy.spin()
