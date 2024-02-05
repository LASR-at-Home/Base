#!/usr/bin/env python3
import clip
import torch


def load_model(model_name: str, device: str = "cuda"):
    """Load the CLIP model.

    Args:
        model_name (str): the model name
        device (str, optional): the device to use. Defaults to "cuda".

    Returns:
        Any: the model and preprocess function
    """
    model, preprocess = clip.load(model_name, device=device)
    return model, preprocess


def run_clip(model, preprocess, labels, device, img):
    """Run the CLIP model.

    Args:
        model (Any): the model
        preprocess (Any): the preprocess function
        labels (List[str]): the labels

    Returns:
        List[float]: the probabilities
    """
    txt = clip.tokenize(labels).to(device)
    img = preprocess(img).unsqueeze(0).to(device)
    with torch.no_grad():
        logits_per_image, logits_per_text = model(img, txt)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()
    return probs
