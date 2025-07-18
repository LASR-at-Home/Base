# !/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge

from lasr_vision_msgs.srv import YoloDetection3D, YoloDetection3DRequest


class BarrierLineDetectorNode:
    """Node that detects barrier lines using YOLO and broadcasts detection flag"""

    def __init__(self):
        # Detection parameters
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.yolo_service = rospy.get_param("~yolo_service", "/yolo/detect3d")
        self.yolo_model = rospy.get_param("~yolo_model", "barrier.pt")
        self.confidence_threshold = rospy.get_param("~confidence_threshold", 0.6)
        self.detection_interval = rospy.get_param("~detection_interval", 1.0)  # Detection frequency in seconds

        # Camera and sensor topics
        self.image_topic = rospy.get_param("~image_topic", "/xtion/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_image_topic", "/xtion/depth_registered/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/xtion/depth_registered/camera_info")

        # Flag broadcast topic
        self.flag_topic = rospy.get_param("~flag_topic", "/barrier_detected")

        # Initialize bridge for image conversion
        self.bridge = CvBridge()

        # Store latest sensor data
        self.latest_image = None
        self.latest_depth = None
        self.latest_info = None

        # Subscribe to camera topics
        rospy.Subscriber(self.image_topic, Image, self.cb_image, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        # Publisher for barrier detection flag
        self.flag_pub = rospy.Publisher(self.flag_topic, Bool, queue_size=1)

        # Wait for YOLO 3D detection service
        rospy.loginfo("Waiting for YOLO service: %s...", self.yolo_service)
        rospy.wait_for_service(self.yolo_service)
        self.yolo_srv = rospy.ServiceProxy(self.yolo_service, YoloDetection3D)
        rospy.loginfo("YOLO 3D service ready.")

        # Start detection timer
        self.timer = rospy.Timer(rospy.Duration(self.detection_interval), self.timer_cb)

        rospy.loginfo("BarrierLineDetectorNode initialized successfully")
        rospy.loginfo("Model: %s, Confidence threshold: %.2f", self.yolo_model, self.confidence_threshold)
        rospy.loginfo("Detection interval: %.1f seconds", self.detection_interval)
        rospy.loginfo("Broadcasting flag on topic: %s", self.flag_topic)

    def cb_image(self, msg):
        """Callback to store latest RGB image"""
        self.latest_image = msg

    def cb_depth(self, msg):
        """Callback to store latest depth image"""
        self.latest_depth = msg

    def cb_info(self, msg):
        """Callback to store latest camera info"""
        self.latest_info = msg

    def timer_cb(self, event):
        """Main timer callback for barrier detection and flag broadcasting"""
        if self.latest_image is None or self.latest_depth is None or self.latest_info is None:
            rospy.logwarn_throttle(10.0, "Waiting for image/depth/camera_info...")
            # Broadcast false flag when no sensor data available
            self.broadcast_flag(False)
            return

        try:
            # Call YOLO 3D detection service
            detections = self.call_yolo_service()

            # Check if any detection meets confidence threshold
            barrier_detected = self.check_barrier_detection(detections)

            # Broadcast detection flag
            self.broadcast_flag(barrier_detected)

        except Exception as e:
            rospy.logwarn("Failed to get YOLO detections: %s", str(e))
            # Broadcast false flag when detection fails
            self.broadcast_flag(False)

    def call_yolo_service(self):
        """Call YOLO 3D detection service with current sensor data"""
        req = YoloDetection3DRequest()
        req.model = self.yolo_model
        req.confidence = 0.1  # Use lower confidence for service call, filter later
        req.filter = []  # No class filtering, let barrier.pt detect whatever it's trained for
        req.image_raw = self.latest_image
        req.depth_image = self.latest_depth
        req.depth_camera_info = self.latest_info
        req.target_frame = self.target_frame

        resp = self.yolo_srv(req)
        return resp.detected_objects

    def check_barrier_detection(self, detections):
        """Check if any detection meets the confidence threshold"""
        high_confidence_detections = []

        for detection in detections:
            # Get confidence score from detection
            confidence = getattr(detection, 'confidence', 0.0)
            class_name = getattr(detection, 'name', 'unknown')

            if confidence >= self.confidence_threshold:
                high_confidence_detections.append(detection)
                rospy.loginfo("High confidence barrier detected: %s (confidence: %.3f)",
                              class_name, confidence)

        # Log detection summary
        total_detections = len(detections)
        valid_detections = len(high_confidence_detections)

        if total_detections > 0:
            rospy.loginfo("Detection summary: %d total, %d above threshold (%.2f)",
                          total_detections, valid_detections, self.confidence_threshold)
        else:
            rospy.logdebug("No barriers detected in current frame")

        return len(high_confidence_detections) > 0

    def broadcast_flag(self, barrier_detected):
        """Broadcast barrier detection flag"""
        flag_msg = Bool()
        flag_msg.data = barrier_detected

        self.flag_pub.publish(flag_msg)

        # Log flag state changes
        if hasattr(self, '_last_flag_state'):
            if self._last_flag_state != barrier_detected:
                rospy.loginfo("Barrier detection flag changed: %s -> %s",
                              self._last_flag_state, barrier_detected)
        else:
            rospy.loginfo("Broadcasting barrier detection flag: %s", barrier_detected)

        self._last_flag_state = barrier_detected


if __name__ == "__main__":
    rospy.init_node("barrier_line_detector_node")
    node = BarrierLineDetectorNode()
    rospy.spin()
