#!/usr/bin/env python3
"""
Test script for open_vocab/detect and open_vocab/detect_and_segment services.

Usage:
  # Detection from image file:
  python3 test_detect.py --image /path/to/image.jpg person cup table

  # Detection from Tiago camera:
  python3 test_detect.py person cup table

  # Segmentation from image file:
  python3 test_detect.py --image /path/to/image.jpg --segment person cup

  # Segmentation from Tiago camera:
  python3 test_detect.py --segment person cup
"""
import argparse
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lasr_vision_interfaces.srv import OpenVocabDetect, OpenVocabDetectAndSegment


class TestDetect(Node):
    def __init__(self, queries, image_path=None, output_path='/tmp/open_vocab_result.jpg', segment=False):
        super().__init__('test_detect')
        self.queries = queries
        self.image_path = image_path
        self.output_path = output_path
        self.segment = segment
        self.bridge = CvBridge()
        self._latest_image = None

        if segment:
            self.client = self.create_client(OpenVocabDetectAndSegment, '/open_vocab/detect_and_segment')
        else:
            self.client = self.create_client(OpenVocabDetect, '/open_vocab/detect')

        if image_path is None:
            self.create_subscription(Image, '/head_front_camera/rgb/image_raw',
                                     lambda msg: setattr(self, '_latest_image', msg), 1)

    def _get_image(self):
        if self.image_path:
            img = cv2.imread(self.image_path)
            if img is None:
                self.get_logger().error(f'Could not read image: {self.image_path}')
                return None, None
            return img, self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        else:
            self.get_logger().info('Waiting for camera image...')
            while self._latest_image is None:
                rclpy.spin_once(self, timeout_sec=0.5)
            img = self.bridge.imgmsg_to_cv2(self._latest_image, desired_encoding='bgr8')
            return img, self._latest_image

    def run(self):
        self.get_logger().info('Waiting for service...')
        self.client.wait_for_service()

        img, image_msg = self._get_image()
        if img is None:
            return

        if self.segment:
            req = OpenVocabDetectAndSegment.Request()
        else:
            req = OpenVocabDetect.Request()

        req.image = image_msg
        req.queries = self.queries
        req.box_threshold = 0.3
        req.text_threshold = 0.25

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()

        result_img = img.copy()

        # Draw masks if segmentation
        if self.segment and result.masks:
            colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)]
            for i, mask_msg in enumerate(result.masks):
                mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
                color = colors[i % len(colors)]
                overlay = result_img.copy()
                overlay[mask > 0] = color
                result_img = cv2.addWeighted(result_img, 0.6, overlay, 0.4, 0)
                mask_path = self.output_path.replace('.jpg', f'_mask_{i}.jpg').replace('.jpeg', f'_mask_{i}.jpeg').replace('.png', f'_mask_{i}.png')
                cv2.imwrite(mask_path, mask)
                print(f'Saved mask {i} to {mask_path}')

        # Draw bounding boxes
        for det in result.detections:
            x, y, w, h = det.xywh
            x1, y1, x2, y2 = int(x - w/2), int(y - h/2), int(x + w/2), int(y + h/2)
            cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'{det.name}: {det.confidence:.2f}'
            cv2.putText(result_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f'{det.name}: {det.confidence:.2f} xywh={list(det.xywh)}')

        cv2.imwrite(self.output_path, result_img)
        print(f'Saved result to {self.output_path}')


def main():
    parser = argparse.ArgumentParser(description='Test open_vocab detect/segment service')
    parser.add_argument('queries', nargs='+', help='Text queries e.g. person cup table')
    parser.add_argument('--image', type=str, default=None, help='Path to image file (omit to use Tiago camera)')
    parser.add_argument('--output', type=str, default='/tmp/open_vocab_result.jpg', help='Output image path')
    parser.add_argument('--segment', action='store_true', help='Use detect_and_segment service')
    args = parser.parse_args()

    rclpy.init()
    node = TestDetect(queries=args.queries, image_path=args.image, output_path=args.output, segment=args.segment)
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
