This package currently contains two services `txt_index_service` and `txt_query_service`. These services are used to create and search (respectively) a vector database of natural language sentence embeddings.

# Index Service
The Index service is used to create a [FAISS](https://github.com/facebookresearch/faiss) index object containing a set of sentence embeddings, where each sentence is assumed to be a line in a given `.txt` file. This Index object is saved to disk at a specified location, and can be thought of as a Vector Database. 

## Request
The request takes two string parameters: `txt_path` which is the path to the `.txt` file we wish to create sentence embeddings for, where each line in this file is treated as a sentence; and `index_path` which is the path to a `.index` file that will be created by the Service.

## Response
No response is given from this service.

## Example Usage
Please see the `scripts/test_index_service.py` script for a simple example of sending a request to the service.

# Query Service
The query service is used to search the `.index` file created by the Index Service to find the most similar sentences given an input query sentence.

## Request
The request requires four fields:

1. `txt_path` -- this is a `string` that is the path to the txt file that contains the original sentences that the `.index` file was populated with.
2. `index_path` -- this is a `string` that is the path to the `.index` file that was created with the Index Service, on the same txt file as the `txt_path`.
3. `query_sentence` -- this is a `string` that is the sentence that you wish to query the index with and find the most similar sentence.
4. `k` -- this is a `uint8` that is the number of closest sentences you wish to return.

## Response
The response contains two fields:

1. `closest_sentences` -- this is an ordered list of `string`s that contain the closest sentences to the given query sentence.
2. `cosine_similaities` -- this is an ordered list of `float32`s that contain the cosine similarity scores of the closest sentences.

## Example Usage
Please see the `scripts/test_query_service.py` script for a simple example of sending a request to the service.