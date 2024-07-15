import numpy as np


def numpy2message(np_array: np.ndarray) -> list[bytes, list[int], str]:
    data = np_array.tobytes()
    shape = list(np_array.shape)
    dtype = str(np_array.dtype)
    return data, shape, dtype


def message2numpy(data: bytes, shape: list[int], dtype: str) -> np.ndarray:
    array_shape = tuple(shape)
    array_dtype = np.dtype(dtype)

    deserialized_array = np.frombuffer(data, dtype=array_dtype)
    deserialized_array = deserialized_array.reshape(array_shape)

    return deserialized_array
