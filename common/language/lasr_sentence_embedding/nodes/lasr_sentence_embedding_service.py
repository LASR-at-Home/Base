import rospy
import numpy as np

from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

from lasr_msgs.srv import (
    SentenceEmbedding,
    SentenceEmbeddingResponse,
    SentenceEmbeddingRequest,
)


class LASRSentenceEmbeddingService:

    _model: SentenceTransformer
    _service: rospy.Service

    """Service to compute sentence embeddings using LASR."""

    def __init__(self):
        self._model = SentenceTransformer(
            "Lajavaness/bilingual-embedding-small", trust_remote_code=True
        )

        self._service = rospy.Service(
            "/lasr_sentence_embedding/sentence_embedding",
            SentenceEmbedding,
            self._sentence_embedding,
        )
        rospy.loginfo("LASR Sentence Embedding Service started.")

    def _sentence_embedding(
        self, request: SentenceEmbeddingRequest
    ) -> SentenceEmbeddingResponse:
        """Compute sentence embeddings for the given request."""
        response = SentenceEmbeddingResponse()

        sentences = request.sentences
        try:
            # Compute embeddings
            embeddings = self._model.encode(sentences)
            cosine_similarity = cosine_similarity(embeddings)
            val, most_similar_idx = self._max_off_diagonal(embeddings)
            most_similar_1 = sentences[most_similar_idx[0]]
            most_similar_2 = sentences[most_similar_idx[1]]

            response.most_similar = [most_similar_1, most_similar_2]
            response.cosine_similarity = val
        except Exception as e:
            rospy.logerr(f"Error computing sentence embeddings: {e}")

        return response

    def _max_off_diagonal(self, a):
        assert a.ndim == 2 and a.shape[0] == a.shape[1], "Input must be a square matrix"

        # Create a mask that excludes diagonal elements
        mask = ~np.eye(a.shape[0], dtype=bool)

        # Find the index of the max off-diagonal element
        max_idx_flat = np.argmax(a[mask])
        value = a[mask][max_idx_flat]

        # Convert flat index back to 2D index using where
        row, col = np.where(mask)
        return value, (row[max_idx_flat], col[max_idx_flat])
