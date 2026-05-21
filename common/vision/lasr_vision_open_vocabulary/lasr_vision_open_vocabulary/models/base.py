from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np


class BaseDetector(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray, queries: List[str], box_threshold: float, text_threshold: float) -> List[Tuple]:
        """Returns list of (label, confidence, x1, y1, x2, y2)."""
        ...
