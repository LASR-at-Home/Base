#!/usr/bin/env python3
"""
Standalone test for open_vocabulary_models detection.
Publishes a local image and calls the detection service.

Usage:
  ros2 run open_vocabulary_models test_detection --ros-args -p image_path:=/path/to/image.jpg -p queries:="bottle,cup,person"
  # or directly:
  python3 test_detection.py /path/to/image.jpg bottle cup person
"""
import sys
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from lasr_vision_interfaces.srv import OpenVocabDetect
from sensor_msgs.msg import Image


class DetectionTest(Node):
    def __init__(self, image_path, queries):
        super().__init__('detection_test')
        self.bridge = CvBridge()
        self.image_path = image_path
        self.queries = queries
        self.cli = self.create_client(OpenVocabDetect, 'open_vocab/detect')

    def run(self):
        self.get_logger().info(f'Loading image: {self.image_path}')
        img = cv2.imread(self.image_path)
        if img is None:
            self.get_logger().error(f'Cannot load image: {self.image_path}')
            return

        self.get_logger().info(f'Waiting for service open_vocab/detect...')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available')
            return

        req = OpenVocabDetect.Request()
        req.image = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        req.queries = self.queries
        req.box_threshold = 0.3
        req.text_threshold = 0.1

        self.get_logger().info(f'Calling detection with queries: {self.queries}')
        future = self.cli.call_async(req)

        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        resp = future.result()
        self.get_logger().info(f'Found {len(resp.detections)} detections:')
        for det in resp.detections:
            self.get_logger().info(f'  {det.name} ({det.confidence:.2f}) @ {det.xywh}')

        # Show image with bounding boxes
        for det in resp.detections:
            cx, cy, w, h = det.xywh
            x1, y1 = cx - w // 2, cy - h // 2
            x2, y2 = cx + w // 2, cy + h // 2
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f'{det.name} {det.confidence:.2f}',
                       (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow('Detections', img)
        self.get_logger().info('Press any key to close...')
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 test_detection.py <image_path> [query1 query2 ...]')
        print('Example: python3 test_detection.py /tmp/scene.jpg bottle cup person')
        sys.exit(1)

    image_path = sys.argv[1]
    queries = sys.argv[2:] if len(sys.argv) > 2 else ['bottle', 'cup', 'person', 'object']

    rclpy.init()
    node = DetectionTest(image_path, queries)
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
