#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from sentence_transformers import SentenceTransformer
from sklearn.metrics.pairwise import cosine_similarity

from lasr_llm_interfaces.srv import SentenceEmbedding


class LASRSentenceEmbeddingService(Node):

    _model: SentenceTransformer

    """Service to compute sentence embeddings using LASR."""

    def __init__(self):
        super().__init__("lasr_sentence_embedding_service")

        self._model = SentenceTransformer(
            "Lajavaness/bilingual-embedding-small", trust_remote_code=True
        )

        self._service = self.create_service(
            SentenceEmbedding,
            "/lasr_sentence_embedding/sentence_embedding",
            self._sentence_embedding,
        )
        self.get_logger().info("LASR Sentence Embedding Service started.")

    def _sentence_embedding(self, request, response):
        """Compute sentence embeddings for the given request."""
        sentences = request.sentences
        try:
            # Compute embeddings
            embeddings = self._model.encode(sentences)
            sim = cosine_similarity(embeddings)
            val, most_similar_idx = self._max_off_diagonal(sim)
            most_similar_1 = sentences[most_similar_idx[0]]
            most_similar_2 = sentences[most_similar_idx[1]]

            response.most_similar = [most_similar_1, most_similar_2]
            response.cosine_similarity = float(val)

        except Exception as e:
            self.get_logger().error(f"Error computing sentence embeddings: {e}")
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


def main(args=None):
    rclpy.init(args=args)
    node = LASRSentenceEmbeddingService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
