#!/usr/bin/env python3
import os
import torch
import numpy as np
import matplotlib.pyplot as plt
from sklearn.manifold import TSNE

# import rospy
import faiss  # type: ignore
from sentence_transformers import SentenceTransformer  # type: ignore
from typing import Optional

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


def load_commands(command_path: str) -> list[str]:
    """Loads the commands stored in the given txt file
    into a list of string commands
    Args:
        command_path (str): path to the txt file containing
        the commands -- assumes one command per line.
    Returns:
        list[str]: list of string commands where each entry in the
        list is a command
    """
    command_list = []
    with open(command_path, "r", encoding="utf8") as src:
        for command in src:
            # Strip newline char.
            command_list.append(command[:-1])
    return command_list


def get_sentence_embeddings(
    command_list: list[str], model: SentenceTransformer
) -> np.ndarray:
    """Converts the list of command strings into an array of sentence
    embeddings (where each command is a sentence and each sentence
    is converted to a vector)
    Args:
        command_list (list[str]): list of string commands, where each
        entry in the list is assumed to be a separate command
        model (SentenceTransformer): model used to perform the embedding.
        Assumes a method called encode that takes a list of strings
        as input.
    Returns:
        np.ndarray: array of shape (n_commands, embedding_dim)
    """

    return model.encode(
        command_list,
        convert_to_numpy=True,
        show_progress_bar=True,
        batch_size=256,
        device=DEVICE,
    )


def create_vector_database(command_embeddings: np.ndarray) -> faiss.IndexFlatIP:
    print("Creating vector database")
    index_flat = faiss.IndexFlatIP(command_embeddings.shape[1])
    faiss.normalize_L2(command_embeddings)
    index_flat.add(command_embeddings)
    print("Finished creating vector database")
    return index_flat


def get_command_database(
    index_path: str, command_path: Optional[str] = None
) -> faiss.IndexFlatL2:
    """Gets a vector database containing a list of embedded commands. Creates the database
    if the path does not exist, else, loads it into memory.

    Args:
        index_path (str): Path to an existing faiss Index, or where to save a new one.
        command_path (str, optional): Path of text file containing commands.
        Only required if creating a new database. Defaults to None.

    Returns:
        faiss.IndexFlatL2: faiss Index object containing the embedded commands.
    """

    if not os.path.exists(f"{index_path}.index"):
        # rospy.loginfo("Creating new command vector database")
        assert command_path is not None
        command_list = load_commands(command_path)
        model = SentenceTransformer("all-MiniLM-L6-v2")
        command_embeddings = get_sentence_embeddings(command_list, model)
        print(command_embeddings.shape)
        command_database = create_vector_database(command_embeddings)
        faiss.write_index(command_database, f"{index_path}.index")
        # rospy.loginfo("Finished creating vector database")
    else:
        command_database = faiss.read_index(f"{index_path}.index")

    return command_database


def get_similar_commands(
    command: str,
    index_path: str,
    command_path: str,
    n_similar_commands: int = 100,
    return_embeddings: bool = False,
) -> tuple[list[str], list[float]]:
    """Gets the most similar commands to the given command string
    Args:
        command (str): command to compare against the database
        index_path (str): path to the location to create or retrieve
        the faiss index containing the embedded commands.
        command_path (str): path to the txt file containing the commands
        n_similar_commands (int, optional): number of similar commands to
        return. Defaults to 100.
    Returns:
        list[str]: list of string commands, where each entry in the
        list is a similar command
    """
    command_database = get_command_database(index_path, command_path)
    command_list = load_commands(command_path)
    model = SentenceTransformer("all-MiniLM-L6-v2")
    command_embedding = get_sentence_embeddings([command], model)
    faiss.normalize_L2(command_embedding)
    command_distances, command_indices = command_database.search(
        command_embedding, n_similar_commands
    )
    nearest_commands = [command_list[i] for i in command_indices[0]]

    if return_embeddings:
        all_command_embeddings = get_sentence_embeddings(command_list, model)
        # filter for only nererst commands
        all_command_embeddings = all_command_embeddings[command_indices[0]]
        return (
            nearest_commands,
            list(command_distances[0]),
            all_command_embeddings,
            command_embedding,
        )

    return nearest_commands, list(command_distances[0])


if __name__ == "__main__":
    command = "find Jared and asks if he needs help"
    result, distances, command_embeddings, query_embedding = get_similar_commands(
        command,
        "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/qualification/data/command_index",
        "/home/mattbarker/LASR/lasr_ws/src/lasr-base/tasks/qualification/data/command_list.txt",
        n_similar_commands=1000,
        return_embeddings=True,
    )
    command_embeddings = np.concatenate((command_embeddings, query_embedding))
    tsne = TSNE(
        n_components=2,
        perplexity=3,
        verbose=1,
        n_iter=1000,
        n_iter_without_progress=1000,
    )
    print("Fitting tsne")
    tsne_embed = tsne.fit_transform(command_embeddings)
    # plot the embeddings
    plt.scatter(tsne_embed[:-1, 0], tsne_embed[:-1, 1], label="Commands")
    plt.scatter(tsne_embed[-1, 0], tsne_embed[-1, 1], label="Query")
    # Annotate the query
    plt.annotate(
        command,
        (tsne_embed[-1, 0], tsne_embed[-1, 1]),
    )
    # Set title
    plt.title("t-SNE plot of 1000 closest command embeddings")
    # Label axes
    plt.xlabel("t-SNE Dimension 1")
    plt.ylabel("t-SNE Dimension 2")
    print(f"Command: {command}")
    for i in range(3):
        # decide if it should be th or st
        if i == 0:
            suffix = "st"
        elif i == 1:
            suffix = "nd"
        elif i == 2:
            suffix = "rd"
        else:
            suffix = "th"
        print(
            f"The {i+1}{suffix} closest command is {result[i]} with a similarity of {distances[i]:.2f}"
        )
        # Annotate the points
        plt.annotate(
            result[i],
            (tsne_embed[i, 0], tsne_embed[i, 1]),
        )

    plt.show()
