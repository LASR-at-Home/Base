#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from lasr_vision_interfaces.msg import Detection
from lasr_vision_interfaces.srv import OpenVocabDetect, OpenVocabDetectAndSegment
from lasr_vision_open_vocabulary.models import (
    GroundingDinoDetector,
    YoloWorldDetector,
    VitSam,
)


class OpenVocabNode(Node):
    def __init__(self):
        super().__init__("lasr_vision_open_vocabulary")
        self.declare_parameter(
            "model", "grounding_dino"
        )  # 'grounding_dino' or 'yoloworld'
        self.declare_parameter("model_device", "cuda")
        self.declare_parameter("yoloworld_weights", "yolov8s-world.pt")
        self.declare_parameter("grounding_dino_weights", "")
        self.declare_parameter("use_sam", False)
        self.declare_parameter("sam_encoder_path", "")
        self.declare_parameter("sam_decoder_path", "")

        model_name = self.get_parameter("model").value
        device = self.get_parameter("model_device").value
        weights = self.get_parameter("yoloworld_weights").value
        gd_weights = self.get_parameter("grounding_dino_weights").value
        use_sam = self.get_parameter("use_sam").value
        encoder_path = self.get_parameter("sam_encoder_path").value
        decoder_path = self.get_parameter("sam_decoder_path").value

        # CLIP recognition rerank: detector localises (boxes), CLIP re-labels each
        # crop against a candidate list. OFF by default - enabled per-task via
        # params (e.g. pick_and_place passes clip_rerank:=true + clip_candidates).
        self.declare_parameter("clip_rerank", False)
        self.declare_parameter("clip_candidates", [""])
        self.declare_parameter("clip_model", "openai/clip-vit-base-patch32")
        self.declare_parameter("clip_prompt_template", "a photo of a {}")
        self._clip_rerank = bool(self.get_parameter("clip_rerank").value)
        self._clip_candidates = [
            c for c in (self.get_parameter("clip_candidates").value or []) if c
        ]
        self._clip_model_name = self.get_parameter("clip_model").value
        self._clip_template = self.get_parameter("clip_prompt_template").value
        self._clip_device = device
        self._clip = None  # (model, processor, torch) - lazy
        if self._clip_rerank and self._clip_candidates:
            self.get_logger().info(
                f"CLIP rerank ENABLED ({len(self._clip_candidates)} candidates)"
            )

        self.bridge = CvBridge()
        self.detector = None
        self.vitsam = None

        if model_name == "grounding_dino":
            try:
                src = gd_weights or "HuggingFace"
                self.get_logger().info(f"Loading Grounding DINO from {src} on {device}")
                self.detector = GroundingDinoDetector(
                    device=device, weights_path=gd_weights
                )
                self.get_logger().info("Grounding DINO loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Failed loading Grounding DINO: {e}")
        elif model_name == "yoloworld":
            try:
                self.get_logger().info(f"Loading YOLOWorld ({weights}) on {device}")
                self.detector = YoloWorldDetector(weights=weights, device=device)
                self.get_logger().info("YOLOWorld loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Failed loading YOLOWorld: {e}")
        else:
            self.get_logger().error(
                f'Unknown model: {model_name}. Use "grounding_dino" or "yoloworld".'
            )

        if use_sam and encoder_path and decoder_path:
            try:
                self.get_logger().info(f"Loading EfficientViT-SAM on {device}")
                self.vitsam = VitSam(encoder_path, decoder_path, device=device)
                self.get_logger().info("EfficientViT-SAM loaded successfully")
            except Exception as e:
                self.get_logger().error(f"Failed loading EfficientViT-SAM: {e}")

        self.create_service(OpenVocabDetect, "open_vocab/detect", self.handle_detect)
        self.create_service(
            OpenVocabDetectAndSegment,
            "open_vocab/detect_and_segment",
            self.handle_detect_and_segment,
        )

    def handle_detect(self, request, response):
        if self.detector is None:
            self.get_logger().error("No detector loaded.")
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return response

        detections = self.detector.detect(
            cv_image,
            request.queries,
            request.box_threshold,
            request.text_threshold,
        )

        self.get_logger().info(f"Found {len(detections)} detections")
        do_rerank = self._clip_rerank and bool(self._clip_candidates)
        for label, score, x1, y1, x2, y2 in detections:
            if do_rerank:
                new_label, clip_conf = self._rerank_label(
                    cv_image, x1, y1, x2, y2, label
                )
                if new_label != label:
                    self.get_logger().info(
                        f"CLIP rerank: '{label}' -> '{new_label}'"
                        + (f" ({clip_conf:.2f})" if clip_conf is not None else "")
                    )
                label = new_label
                if clip_conf is not None:
                    score = clip_conf
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            w, h = x2 - x1, y2 - y1
            det = Detection()
            det.name = label
            det.confidence = score
            det.xywh = [int(cx), int(cy), int(w), int(h)]
            response.detections.append(det)

        return response

    # --- CLIP recognition rerank ---
    def _load_clip(self):
        """Lazily load CLIP (model, processor, torch). Disables rerank on failure."""
        if self._clip is not None:
            return self._clip
        try:
            import torch
            from transformers import CLIPModel, CLIPProcessor

            self.get_logger().info(
                f"Loading CLIP ({self._clip_model_name}) on {self._clip_device}..."
            )
            model = (
                CLIPModel.from_pretrained(self._clip_model_name)
                .to(self._clip_device)
                .eval()
            )
            processor = CLIPProcessor.from_pretrained(self._clip_model_name)
            self._clip = (model, processor, torch)
            self.get_logger().info("CLIP loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed loading CLIP - rerank disabled: {e}")
            self._clip_rerank = False
            self._clip = None
        return self._clip

    def _rerank_label(self, cv_image, x1, y1, x2, y2, orig_label):
        """Crop the box and pick the best-matching candidate via CLIP.

        Returns (label, confidence). Falls back to orig_label on any problem.
        """
        clip = self._load_clip()
        if clip is None:
            return orig_label, None
        model, processor, torch = clip

        h, w = cv_image.shape[:2]
        x1 = max(0, min(int(x1), w - 1))
        x2 = max(0, min(int(x2), w))
        y1 = max(0, min(int(y1), h - 1))
        y2 = max(0, min(int(y2), h))
        if x2 - x1 < 2 or y2 - y1 < 2:
            return orig_label, None

        try:
            from PIL import Image as PILImage

            crop = cv2.cvtColor(cv_image[y1:y2, x1:x2], cv2.COLOR_BGR2RGB)
            pil = PILImage.fromarray(crop)
            prompts = [self._clip_template.format(c) for c in self._clip_candidates]
            inputs = processor(
                text=prompts, images=pil, return_tensors="pt", padding=True
            ).to(self._clip_device)
            with torch.no_grad():
                probs = model(**inputs).logits_per_image.softmax(dim=-1)[0]
            best = int(probs.argmax())
            return self._clip_candidates[best], float(probs[best])
        except Exception as e:
            self.get_logger().warn(f"CLIP rerank failed for a crop: {e}")
            return orig_label, None

    def handle_detect_and_segment(self, request, response):
        detect_resp = OpenVocabDetect.Response()
        detect_resp = self.handle_detect(request, detect_resp)
        response.detections = detect_resp.detections

        if not self.vitsam or not detect_resp.detections:
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return response

        bboxes = []
        for det in detect_resp.detections:
            cx, cy, w, h = det.xywh
            bboxes.append([cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2])

        masks, _ = self.vitsam(cv_image, bboxes)

        for i, (mask, det) in enumerate(zip(masks, detect_resp.detections)):
            mask_bin = np.squeeze(mask).astype(np.uint8) * 255
            M = cv2.moments(mask_bin)
            if M["m00"] > 0:
                det.xywh[0] = int(M["m10"] / M["m00"])
                det.xywh[1] = int(M["m01"] / M["m00"])
            response.masks.append(self.bridge.cv2_to_imgmsg(mask_bin, encoding="mono8"))
            response.ids.append(i)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = OpenVocabNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
