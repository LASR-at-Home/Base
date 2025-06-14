#!/usr/bin/env python3
import rospy
import smach
import threading
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest

from typing import List, Optional


class DetectDict(smach.State):
    def __init__(
        self,
        topic: str = "/xtion/rgb/image_raw",
        model: str = "yolo11n.pt",
        confidence: float = 0.5,
        duration: float = 3.0,  # seconds to collect images
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections"],
        )
        self.topic = topic
        self.model = model
        self.confidence = confidence
        self.duration = duration

        self.bridge = CvBridge()
        self.frames = []
        self.lock = threading.Lock()

        rospy.wait_for_service("/yolo/detect")
        self.yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)

    def image_callback(self, msg: Image):
        with self.lock:
            if len(self.frames) < 100:
                self.frames.append(msg)

    def execute(self, userdata):
        self.frames = []
        sub = rospy.Subscriber(self.topic, Image, self.image_callback)

        rospy.loginfo(f"[DetectDict] Collecting images from {self.topic} for {self.duration} seconds.")
        start_time = time.time()
        while time.time() - start_time < self.duration:
            rospy.sleep(0.1)

        sub.unregister()
        rospy.loginfo(f"[DetectDict] Collected {len(self.frames)} frames.")

        detections = []
        names_seen = set()

        for msg in self.frames:
            try:
                req = YoloDetectionRequest()
                req.image_raw = msg
                req.model = self.model
                req.confidence = self.confidence
                req.filter = []

                res = self.yolo(req)
                for det in res.detected_objects:
                    key = (det.name, tuple(det.xywh))
                    if key in names_seen:
                        continue
                    names_seen.add(key)
                    detections.append({
                        "name": det.name,
                        "confidence": det.confidence,
                        "bbox": det.xywh
                    })

            except rospy.ServiceException as e:
                rospy.logwarn(f"[DetectDict] YOLO service failed: {e}")
                userdata.detections = []  # Ensure overwrite even on failure
                return "failed"

        userdata.detections = detections
        rospy.loginfo(f"[DetectDict] Final detections (len={len(detections)}): {detections}")

        return "succeeded" if detections else "failed"
