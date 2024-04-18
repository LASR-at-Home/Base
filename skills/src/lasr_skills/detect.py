#!/usr/bin/env python3

import rospy
import smach
from sensor_msgs.msg import Image

from lasr_vision_msgs.srv import YoloDetection

from typing import List, Union


class Detect(smach.State):

    def __init__(
        self,
        image_topic: str = "/xtion/rgb/image_raw",
        model: str = "yolov8n.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections"],
        )
        self.image_topic = image_topic
        self.model = model
        self.filter = filter if filter is not None else []
        self.confidence = confidence
        self.nms = nms
        self.yolo = rospy.ServiceProxy("/yolov8/detect", YoloDetection)
        self.yolo.wait_for_service()

    def execute(self, userdata):
        img_msg = rospy.wait_for_message(self.image_topic, Image)
        try:
            result = self.yolo(img_msg, self.model, self.confidence, self.nms)
            result.detected_objects = [
                det for det in result.detected_objects if det.name in self.filter
            ]
            userdata.detections = result
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
