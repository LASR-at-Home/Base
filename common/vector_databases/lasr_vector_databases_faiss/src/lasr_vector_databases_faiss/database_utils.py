#!/usr/bin/env python3
import os
import numpy as np
import faiss

from typing import Union


def construct_faiss_index(
    index_factory_string: str,
    vector_dim: int,
    normalise: bool = False,
    use_gpu: bool = False,
) -> faiss.Index:
    """Constructs the faiss vector datbase object.

    Args:
        index_factory_string (str): Index factory string
        vector_dim (int): constant dim of each vector to be added to the db.
        normalise (bool, optional): whether to use inner product instead of Euclidean distance.
        Defaults to False.
        use_gpu (bool, optional): whether to move the index to the GPU. Defaults to False.

    Returns:
        faiss.Index: constructed faiss index object.
    """

    metric = faiss.METRIC_INNER_PRODUCT if normalise else faiss.METRIC_L2
    index = faiss.index_factory(vector_dim, index_factory_string, metric)
    if use_gpu:
        index = faiss.index_cpu_to_all_gpus(index)
    return index


def add_vectors_to_index(
    index: faiss.Index,
    vectors: np.ndarray,
    normalise: bool = False,
    add_with_ids: bool = False,
) -> Union[None, np.ndarray]:
    """Adds a set of vectors to the index, optionally normalising vectors
    or adding them with Ids.

    Args:
        index (faiss.Index): index to add the vectors to.
        vectors (np.ndarray): vectors to add to the index of shape (n_vecs, vec_dim)
        normalise (bool, optional): whether to normalise the vectors. Defaults to False.
        add_with_ids (bool, optional): whether to add the vectors with ids. Defaults to False.

    Returns:
        Union[None, np.ndarray]: None or the ids of the vectors added.
    """

    if normalise:
        faiss.normalize_L2(vectors)
    if add_with_ids:
        ids = np.arange(index.ntotal, index.ntotal + vectors.shape[0])
        index.add_with_ids(vectors, ids)
        return ids
    else:
        index.add(vectors)
    return None


def save_index_to_disk(
    index: faiss.Index, index_path: str, overwrite: bool = False
) -> None:
    """Saves the index to disk.

    Args:
        index (faiss.Index): index to save
        index_path (str): path to save the index
        overwrite (bool, optional): whether to overwrite the index if it already exists.
        Defaults to False.
    """
    if os.path.exists(index_path) and not overwrite:
        raise FileExistsError(
            f"Index already exists at {index_path}. Set overwrite=True to replace it."
        )
    faiss.write_index(index, index_path)


def load_vector_database(index_path: str, use_gpu: bool = False) -> faiss.Index:
    """Loads a FAISS Index from the given filepath

    Args:
        index_path (str): path to the index file
        use_gpu (bool, optional): Whether to load the index onto the GPU.
        Defaults to False.

    Returns:
        faiss.Index: FAISS Index object
    """
    print("Loading index from", index_path)
    index = faiss.read_index(index_path)
    print("Loaded index with ntotal:", index.ntotal)
    if use_gpu:
        index = faiss.index_cpu_to_all_gpus(index)
    return index


def query_database(
    index_path: str,
    query_vectors: np.ndarray,
    normalise: bool = False,
    k: int = 1,
) -> tuple[np.ndarray, np.ndarray]:
    """Queries the given index with the given query vectors

    Args:
        index_path (str): path to the index file
        query_vectors (np.ndarray): query vectors of shape (n_queries, vector_dim)
        normalise (bool, optional): Whether to normalise the query vectors.
        Defaults to True.
        k (int, optional): Number of nearest neighbours to return. Defaults to 1.

    Returns:
        tuple[np.ndarray, np.ndarray]: (distances, indices) of the nearest neighbours
        each of shape (n_queries, n_neighbours)
    """
    index = load_vector_database(index_path)
    if normalise:
        faiss.normalize_L2(query_vectors)
    distances, indices = index.search(query_vectors, k)
    return distances, indices
