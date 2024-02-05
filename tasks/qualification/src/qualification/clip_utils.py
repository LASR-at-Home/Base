#!/usr/bin/env python3
import torch
import rospy
import cv2
import cv2_img
from copy import deepcopy
from sentence_transformers import SentenceTransformer, util

from sensor_msgs.msg import Image


def load_model(device: str = "cuda"):
    """Load the CLIP model.

    Args:
        model_name (str): the model name
        device (str, optional): the device to use. Defaults to "cuda".

    Returns:
        Any: the model and preprocess function
    """
    model = SentenceTransformer("clip-ViT-B-32")
    return model


def run_clip(model, labels, img):
    """Run the CLIP model.

    Args:
        model (Any): the model
        preprocess (Any): the preprocess function
        labels (List[str]): the labels

    Returns:
        List[float]: the probabilities
    """
    txt = model.encode(labels)
    img = model.encode(img)
    with torch.no_grad():
        cos_scores = util.cos_sim(img, txt)
    return cos_scores


def vqa(model, answers: list[str]) -> tuple[str, torch.Tensor, Image]:
    """Queries

    Args:
        model (_type_): clip model to run inference on
        answers(list[str]): list of possible answers
    returns:
        tuple(str, torch.Tensor, Image): the answer, the scores, and the annotated image msg
    """
    img_msg = rospy.wait_for_message("/xtion/rgb/image_raw", Image)
    img_pil = cv2_img.msg_to_pillow_img(img_msg)

    cos_scores = run_clip(model, answers, img_pil)
    max_score = cos_scores.argmax()
    # get second highest score in tensor
    max_val = deepcopy(cos_scores[0, max_score])
    cos_scores[0, max_score] = 0
    second_max_score = cos_scores.argmax()
    cos_scores[0, max_score] = max_val
    # Annotate the image
    cv2_im = cv2_img.msg_to_cv2_img(img_msg)
    # normalise the scores
    cv2.putText(
        cv2_im,
        f"Most likely caption: {answers[max_score]}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    # add second score below
    cv2.putText(
        cv2_im,
        f"Second most likely caption: {answers[second_max_score]}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )

    img = cv2_img.cv2_img_to_msg(cv2_im)
    return answers[max_score], cos_scores, img
