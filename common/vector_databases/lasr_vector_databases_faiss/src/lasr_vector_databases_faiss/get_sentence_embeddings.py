#!/usr/bin/env python3
import torch
import numpy as np
from sentence_transformers import SentenceTransformer

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_model(model_name: str) -> SentenceTransformer:
    """Loads the sentence transformer model
    Args:
        model_name (str): name of the model to load
    Returns:
        sentence_transformers.SentenceTransformer: the loaded model
    """
    return SentenceTransformer(model_name, device=DEVICE)


def parse_txt_file(fp: str) -> list[str]:
    """Parses a txt file into a list of strings,
    where each element is a line in the txt file with the
    newline char stripped.
    Args:
        fp (str): path to the txt file to load
    Returns:
        list[str]: list of strings where each element is a line in the txt file
    """
    sentences = []
    with open(fp, "r", encoding="utf8") as src:
        for line in src:
            # Strip newline char.
            sentences.append(line[:-1])
    return sentences


def get_sentence_embeddings(
    sentence_list: list[str], model: SentenceTransformer
) -> np.ndarray:
    """Converts the list of string sentences into an array of sentence
    embeddings
    Args:
        sentece_list (list[str]): list of string sentences, where each
        entry in the list is assumed to be a separate sentence
        model (SentenceTransformer): model used to perform the embedding.
        Assumes a method called encode that takes a list of strings
        as input.
    Returns:
        np.ndarray: array of shape (n_commands, embedding_dim)
    """

    return model.encode(
        sentence_list,
        convert_to_numpy=True,
        show_progress_bar=True,
        batch_size=256,
        device=DEVICE,
    )
