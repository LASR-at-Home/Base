from typing import List, Tuple

import numpy as np
from ultralytics import YOLOWorld as _YOLOWorld

from .base import BaseDetector


class YoloWorldDetector(BaseDetector):
    def __init__(self, weights: str = "yolov8s-world.pt", device: str = "cuda"):
        self.model = _YOLOWorld(weights)
        self.model.to(device)

    def detect(self, image: np.ndarray, queries: List[str], box_threshold: float, text_threshold: float) -> List[Tuple]:
        self.model.set_classes(list(queries))
        results = self.model.predict(image, conf=box_threshold or 0.01)
        if not results:
            return []

        res = results[0]
        boxes = getattr(res.boxes, 'xyxy', None)
        scores = getattr(res.boxes, 'conf', None)
        classes = getattr(res.boxes, 'cls', None)
        names = getattr(res, 'names', None)

        detections = []
        if boxes is not None:
            for i, box in enumerate(boxes):
                try:
                    x1, y1, x2, y2 = float(box[0]), float(box[1]), float(box[2]), float(box[3])
                    label = str(names[int(classes[i])]) if names and classes is not None else 'object'
                    score = float(scores[i]) if scores is not None else 0.0
                    detections.append((label, score, x1, y1, x2, y2))
                except Exception:
                    continue
        return detections
