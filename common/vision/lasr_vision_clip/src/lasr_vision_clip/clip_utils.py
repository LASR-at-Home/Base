#!/usr/bin/env python3
import torch
import rospy
import cv2
import cv2_img
import numpy as np
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
    model = SentenceTransformer("clip-ViT-B-32", device=device)
    return model


def run_clip(
    model: SentenceTransformer, labels: list[str], img: np.ndarray
) -> torch.Tensor:
    """Run the CLIP model.

    Args:
        model (Any): clip model loaded into memory
        labels (List[str]): list of string labels to query image similarity to.
        img (np.ndarray): the image to query

    Returns:
        List[float]: the cosine similarity scores between the image and label embeddings.
    """

    txt = model.encode(labels)
    img = model.encode(img)
    with torch.no_grad():
        cos_scores = util.cos_sim(img, txt)
    return cos_scores


def load_face_model():
    from transformers import AutoImageProcessor, AutoModel

    processor = AutoImageProcessor.from_pretrained("google/vit-base-patch16-224")
    model = AutoModel.from_pretrained("google/vit-base-patch16-224").to("cuda")

    return processor, model


def infer(image, processor, model):
    image = cv2_img.msg_to_cv2_img(image)
    inputs = processor(image, return_tensors="pt").to("cuda")
    outputs = model(**inputs)
    # squeeze and flatten
    outputs.pooler_output = outputs.pooler_output.squeeze(0).flatten()
    return outputs.pooler_output.detach().cpu().numpy()


def encode_img(model, img_msg: Image) -> np.ndarray:
    """Run the CLIP model.

    Args:
        model (Any): clip model loaded into memory
        img (np.ndarray): the image to query

    Returns:
        np.ndarray: the image embedding
    """
    img = cv2_img.msg_to_cv2_img(img_msg)
    return model(img.unsqueeze(0)).detach().numpy()


def query_image(
    img_msg: Image,
    model: SentenceTransformer,
    img_msg: Image,
    answers: list[str],
    annotate: bool = False,
) -> tuple[str, torch.Tensor, Image]:
    """Queries the CLIP model with an image and a set of possible image captions and returns the most likely caption.

    Args:
        img_msg (Image): the image to query
        model (SentenceTransformer): clip model to run inference on, loaded into memory
        answers(list[str]): list of possible answers
        annotate(bool, optional): whether to annotate the image with the most likely, and
        second most likely, caption. Defaults to False.
    returns:
        tuple(str, torch.Tensor, Image): the most likely answer, the scores, and the annotated image msg
    """
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
    if annotate:
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
    return answers[max_score], cos_scores[0, max_score], img
