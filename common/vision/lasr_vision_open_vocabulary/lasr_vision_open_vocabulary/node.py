#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from lasr_vision_interfaces.msg import Detection
from lasr_vision_interfaces.srv import OpenVocabDetect, OpenVocabDetectAndSegment
from lasr_vision_open_vocabulary.models import GroundingDinoDetector, YoloWorldDetector, VitSam


class OpenVocabNode(Node):
    def __init__(self):
        super().__init__('lasr_vision_open_vocabulary')
        self.declare_parameter('model', 'grounding_dino')  # 'grounding_dino' or 'yoloworld'
        self.declare_parameter('model_device', 'cuda')
        self.declare_parameter('yoloworld_weights', 'yolov8s-world.pt')
        self.declare_parameter('grounding_dino_weights', '')
        self.declare_parameter('use_sam', False)
        self.declare_parameter('sam_encoder_path', '')
        self.declare_parameter('sam_decoder_path', '')

        model_name = self.get_parameter('model').value
        device = self.get_parameter('model_device').value
        weights = self.get_parameter('yoloworld_weights').value
        gd_weights = self.get_parameter('grounding_dino_weights').value
        use_sam = self.get_parameter('use_sam').value
        encoder_path = self.get_parameter('sam_encoder_path').value
        decoder_path = self.get_parameter('sam_decoder_path').value

        self.bridge = CvBridge()
        self.detector = None
        self.vitsam = None

        if model_name == 'grounding_dino':
            try:
                src = gd_weights or 'HuggingFace'
                self.get_logger().info(f'Loading Grounding DINO from {src} on {device}')
                self.detector = GroundingDinoDetector(device=device, weights_path=gd_weights)
                self.get_logger().info('Grounding DINO loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed loading Grounding DINO: {e}')
        elif model_name == 'yoloworld':
            try:
                self.get_logger().info(f'Loading YOLOWorld ({weights}) on {device}')
                self.detector = YoloWorldDetector(weights=weights, device=device)
                self.get_logger().info('YOLOWorld loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed loading YOLOWorld: {e}')
        else:
            self.get_logger().error(f'Unknown model: {model_name}. Use "grounding_dino" or "yoloworld".')

        if use_sam and encoder_path and decoder_path:
            try:
                self.get_logger().info(f'Loading EfficientViT-SAM on {device}')
                self.vitsam = VitSam(encoder_path, decoder_path, device=device)
                self.get_logger().info('EfficientViT-SAM loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed loading EfficientViT-SAM: {e}')

        self.create_service(OpenVocabDetect, 'open_vocab/detect', self.handle_detect)
        self.create_service(OpenVocabDetectAndSegment, 'open_vocab/detect_and_segment', self.handle_detect_and_segment)

    def handle_detect(self, request, response):
        if self.detector is None:
            self.get_logger().error('No detector loaded.')
            return response

        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return response

    
        detections = self.detector.detect(
            cv_image,
            request.queries,
            request.box_threshold,
            request.text_threshold,
        )

        self.get_logger().info(f'Found {len(detections)} detections')
        for label, score, x1, y1, x2, y2 in detections:
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
            w, h = x2 - x1, y2 - y1
            det = Detection()
            det.name = label
            det.confidence = score
            det.xywh = [int(cx), int(cy), int(w), int(h)]
            response.detections.append(det)

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
            bboxes.append([cx - w // 2, cy - h // 2, cx + w // 2, cy + h // 2])

        masks, _ = self.vitsam(cv_image, bboxes)
        

        for i, (mask, det) in enumerate(zip(masks, detect_resp.detections)):
            mask_bin = np.squeeze(mask).astype(np.uint8) * 255
            M = cv2.moments(mask_bin)
            if M['m00'] > 0:
                det.xywh[0] = int(M['m10'] / M['m00'])
                det.xywh[1] = int(M['m01'] / M['m00'])
            response.masks.append(self.bridge.cv2_to_imgmsg(mask_bin, encoding='mono8'))
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


if __name__ == '__main__':
    main()
