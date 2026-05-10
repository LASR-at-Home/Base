#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torchvision.transforms as transforms
import torch.nn.functional as F
from ultralytics import YOLOWorld
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image
from lasr_vision_interfaces.srv import OpenVocabDetect, OpenVocabDetectAndSegment
from lasr_vision_interfaces.msg import Detection
from .efficientvit_inference import SamEncoder, SamDecoder, SamResize


class VitSam:
    def __init__(self, encoder_model, decoder_model):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.decoder = SamDecoder(decoder_model, device=self.device)
        self.encoder = SamEncoder(encoder_model, device=self.device)

    def __call__(self, img, bboxes):
        raw_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        origin_image_size = raw_img.shape[:2]
        img = self._preprocess(raw_img, img_size=512)
        img_embeddings = self.encoder(img)
        boxes = np.array(bboxes, dtype=np.float32)
        masks, _, _ = self.decoder.run(
            img_embeddings=img_embeddings,
            origin_image_size=origin_image_size,
            boxes=boxes,
        )
        return masks, boxes

    def _preprocess(self, x, img_size=512):
        pixel_mean = [123.675 / 255, 116.28 / 255, 103.53 / 255]
        pixel_std = [58.395 / 255, 57.12 / 255, 57.375 / 255]

        x = torch.tensor(x)
        resize_transform = SamResize(img_size)
        x = resize_transform(x).float() / 255
        x = transforms.Normalize(mean=pixel_mean, std=pixel_std)(x)

        h, w = x.shape[-2:]
        th, tw = img_size, img_size
        x = F.pad(x, (0, tw - w, 0, th - h), value=0).unsqueeze(0).numpy()

        return x


class OpenVocabNode(Node):
    def __init__(self):
        super().__init__('open_vocabulary_models')
        self.declare_parameter('use_yoloworld', False)
        self.declare_parameter('use_grounding_dino', True)
        self.declare_parameter('yoloworld_weights', 'yolov8s-world.pt')
        self.declare_parameter('model_device', 'cuda')
        self.declare_parameter('use_sam', False)
        self.declare_parameter('sam_encoder_path', '')
        self.declare_parameter('sam_decoder_path', '')
        self.bridge = CvBridge()

        use_yw = self.get_parameter('use_yoloworld').value
        use_gd = self.get_parameter('use_grounding_dino').value
        weights = self.get_parameter('yoloworld_weights').value
        self.device = self.get_parameter('model_device').value
        use_sam = self.get_parameter('use_sam').value
        encoder_path = self.get_parameter('sam_encoder_path').value
        decoder_path = self.get_parameter('sam_decoder_path').value

        self.yoloworld = None
        self.grounding_dino = None
        self.grounding_dino_processor = None
        self.vitsam = None

        if use_gd:
            try:
                self.get_logger().info(f'Loading Grounding DINO on {self.device}')
                model_id = "IDEA-Research/grounding-dino-base"
                self.grounding_dino_processor = AutoProcessor.from_pretrained(model_id)
                self.grounding_dino = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)
                self.get_logger().info('Grounding DINO loaded successfully')
            except Exception as e:
                self.get_logger().warn(f'Failed loading Grounding DINO: {e}')

        if use_yw:
            try:
                self.get_logger().info(f'Loading YOLOWorld weights: {weights} on {self.device}')
                self.yoloworld = YOLOWorld(weights)
                self.yoloworld.to(self.device)
            except Exception as e:
                self.get_logger().warn(f'Failed loading YOLOWorld: {e}')

        if use_sam and encoder_path and decoder_path:
            try:
                self.get_logger().info(f'Loading EfficientViT-SAM on {self.device}')
                self.vitsam = VitSam(encoder_path, decoder_path)
                self.get_logger().info('EfficientViT-SAM loaded successfully')
            except Exception as e:
                self.get_logger().warn(f'Failed loading EfficientViT-SAM: {e}')

        self.srv = self.create_service(OpenVocabDetect, 'open_vocab/detect', self.handle_detect)
        self.srv2 = self.create_service(OpenVocabDetectAndSegment, 'open_vocab/detect_and_segment', self.handle_detect_and_segment)

    def handle_detect(self, request, response):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return response

        # Try Grounding DINO first
        if self.grounding_dino is not None and self.grounding_dino_processor is not None and request.queries:
            try:
                image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                image_pil = Image.fromarray(image_rgb)
                text_queries = " ".join([cls.lower().strip() + "." for cls in request.queries])

                inputs = self.grounding_dino_processor(images=image_pil, text=text_queries, return_tensors="pt").to(self.device)
                with torch.no_grad():
                    outputs = self.grounding_dino(**inputs)

                results = self.grounding_dino_processor.post_process_grounded_object_detection(
                    outputs,
                    input_ids=inputs["input_ids"],
                    box_threshold=request.box_threshold or 0.4,
                    text_threshold=request.text_threshold or 0.4,
                    target_sizes=[image_pil.size[::-1]]
                )[0]

                self.get_logger().info(f'Grounding DINO found {len(results["boxes"])} boxes')
                boxes = results.get("boxes", [])
                scores = results.get("scores", [])
                text_labels = results.get("text_labels", [])

                for box, score, label in zip(boxes, scores, text_labels):
                    x1, y1, x2, y2 = box.tolist()
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    w, h = x2 - x1, y2 - y1
                    det = Detection()
                    det.name = label
                    det.confidence = float(score)
                    det.xywh = [int(cx), int(cy), int(w), int(h)]
                    response.detections.append(det)
            except Exception as e:
                self.get_logger().error(f'Grounding DINO inference failed: {e}')

        # Fallback to YOLOWorld
        if len(response.detections) == 0 and self.yoloworld is not None and request.queries:
            self.yoloworld.set_classes(list(request.queries))
            try:
                results = self.yoloworld.predict(cv_image, conf=request.box_threshold or 0.01)
                if results and len(results) > 0:
                    res = results[0]
                    boxes = getattr(res.boxes, 'xyxy', None)
                    scores = getattr(res.boxes, 'conf', None)
                    classes = getattr(res.boxes, 'cls', None)
                    names = getattr(res, 'names', None)
                    if boxes is not None:
                        for i, box in enumerate(boxes):
                            try:
                                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                                w, h = x2 - x1, y2 - y1
                                label = str(names[int(classes[i])]) if names and classes is not None else 'object'
                                score = float(scores[i]) if scores is not None else 0.0
                                det = Detection()
                                det.name = label
                                det.confidence = score
                                det.xywh = [cx, cy, w, h]
                                response.detections.append(det)
                            except Exception:
                                continue
            except Exception as e:
                self.get_logger().error(f'YOLOWorld inference failed: {e}')

        return response

    def handle_detect_and_segment(self, request, response):
        detect_resp = OpenVocabDetect.Response()
        detect_resp = self.handle_detect(request, detect_resp)
        response.detections = detect_resp.detections

        if not self.vitsam or not detect_resp.detections:
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return response

        bboxes = []
        for det in detect_resp.detections:
            cx, cy, w, h = det.xywh
            bboxes.append([cx - w//2, cy - h//2, cx + w//2, cy + h//2])

        try:
            masks, _ = self.vitsam(cv_image, bboxes)
        except Exception as e:
            self.get_logger().error(f'VitSam inference failed: {e}')
            return response

        for i, (mask, det) in enumerate(zip(masks, detect_resp.detections)):
            try:
                mask_bin = mask.astype(np.uint8) * 255
                M = cv2.moments(mask_bin)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    det.xywh[0] = cx
                    det.xywh[1] = cy

                mask_msg = self.bridge.cv2_to_imgmsg(mask_bin, encoding='mono8')
                response.masks.append(mask_msg)
                response.ids.append(i)
            except Exception as e:
                self.get_logger().warn(f'Failed to process mask {i}: {e}')
                continue

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

if __name__ == '__main__':
    main()
