#!/usr/bin/env python3
import os
import numpy as np
import faiss


def create_vector_database(
    vectors: np.ndarray,
    index_path: str,
    overwrite: bool = False,
    index_type: str = "Flat",
    normalise_vecs: bool = False,
) -> None:
    """Creates a FAISS Index using the factory constructor and the given
    index type, and adds the given vector to the index, and then saves
    it to disk using the given path.

    Args:
        vectors (np.ndarray): vector of shape (n_vectors, vector_dim)
        index_path (str): path to save the index
        overwrite (bool, optional): Whether to replace an existing index
        at the same filepath if it exists. Defaults to False.
        index_type (str, optional): FAISS Index Factory string. Defaults to "IndexFlatIP".
        normalise_vecs (bool, optional): Whether to normalise the vectors before
        adding them to the Index. This converts the IP metric to Cosine Similarity.
        Defaults to False.
    """

    if os.path.exists(index_path) and not overwrite:
        raise FileExistsError(
            f"Index already exists at {index_path}. Set overwrite=True to replace it."
        )

    if normalise_vecs:
        index = faiss.index_factory(
            vectors.shape[1], index_type, faiss.METRIC_INNER_PRODUCT
        )
        faiss.normalize_L2(vectors)
    else:
        index = faiss.index_factory(vectors.shape[1], index_type)
    index.add(vectors)
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
    normalise: bool = True,
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
