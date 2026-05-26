from typing import List, Tuple

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection

from .base import BaseDetector


class GroundingDinoDetector(BaseDetector):
    MODEL_ID = "IDEA-Research/grounding-dino-base"

    def __init__(self, device: str = "cuda"):
        self.device = device
        self.processor = AutoProcessor.from_pretrained(self.MODEL_ID)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(self.MODEL_ID).to(device)

    def detect(self, image: np.ndarray, queries: List[str], box_threshold: float, text_threshold: float) -> List[Tuple]:
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_pil = Image.fromarray(image_rgb)
        text_queries = " ".join([q.lower().strip() + "." for q in queries])

        inputs = self.processor(images=image_pil, text=text_queries, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            input_ids=inputs["input_ids"],
            box_threshold=box_threshold or 0.4,
            text_threshold=text_threshold or 0.4,
            target_sizes=[image_pil.size[::-1]],
        )[0]

        detections = []
        for box, score, label in zip(results.get("boxes", []), results.get("scores", []), results.get("text_labels", [])):
            x1, y1, x2, y2 = box.tolist()
            detections.append((label, float(score), x1, y1, x2, y2))
        return detections
