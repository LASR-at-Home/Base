#!/usr/bin/env python3
"""Static test for open_vocab/detect_and_segment service.
Captures one image from the camera, calls the service, saves results to /tmp/seg_results/.
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lasr_vision_interfaces.srv import OpenVocabDetectAndSegment
from cv_bridge import CvBridge
import cv2
import numpy as np


OUTPUT_DIR = '/tmp/seg_results'


class TestDetectAndSegment(Node):
    def __init__(self):
        super().__init__('test_detect_and_segment')
        self.cli = self.create_client(OpenVocabDetectAndSegment, 'open_vocab/detect_and_segment')
        self.sub = self.create_subscription(Image, '/head_front_camera/rgb/image_raw', self._on_image, 1)
        self.bridge = CvBridge()
        self.done = False
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        self.get_logger().info(f'Waiting for service and image... Results -> {OUTPUT_DIR}')

    def _on_image(self, msg):
        if self.done:
            return
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available')
            return
        self.done = True
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        req = OpenVocabDetectAndSegment.Request()
        req.image = msg
        req.queries = ['can', 'bottle', 'table', 'person', 'chair']
        req.box_threshold = 0.3
        req.text_threshold = 0.1
        self.cli.call_async(req).add_done_callback(self._on_result)

    def _on_result(self, future):
        resp = future.result()
        img = self.cv_image.copy()

        if not resp.detections:
            self.get_logger().info('No detections')
        else:
            for i, det in enumerate(resp.detections):
                cx, cy = det.xywh[0], det.xywh[1]
                bw, bh = det.xywh[2], det.xywh[3]
                x1, y1 = cx - bw // 2, cy - bh // 2
                x2, y2 = cx + bw // 2, cy + bh // 2

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(img, f'{det.name} {det.confidence:.2f}', (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                self.get_logger().info(f'{det.name}  conf={det.confidence:.2f}  centroid=({cx},{cy})')

                if i < len(resp.masks):
                    mask_img = self.bridge.imgmsg_to_cv2(resp.masks[i], desired_encoding='mono8')
                    contours, _ = cv2.findContours(mask_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(img, contours, -1, (255, 0, 0), 2)
                    cv2.imwrite(os.path.join(OUTPUT_DIR, f'mask_{i}_{det.name}.png'), mask_img)

        cv2.imwrite(os.path.join(OUTPUT_DIR, 'result.jpg'), img)
        self.get_logger().info(f'Saved result to {OUTPUT_DIR}/result.jpg')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TestDetectAndSegment()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
