#!/usr/bin/env python3
import torch
from sentence_transformers import SentenceTransformer, util


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


def run_clip(model, labels, device, img):
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
