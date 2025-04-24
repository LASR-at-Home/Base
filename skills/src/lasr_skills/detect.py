#!/usr/bin/env python3
from typing import List, Union

import rospy
import smach
from lasr_vision_msgs.srv import YoloDetection


class Detect(smach.State):
    def __init__(
        self,
        model: str = "yolo11n.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["img_msg"],
            output_keys=["detections"],
        )
        self.model = model
        self.filter = filter if filter is not None else []
        self.confidence = confidence
        self.yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)
        self.yolo.wait_for_service()

    def execute(self, userdata):
        img_msg = userdata.img_msg
        try:
            result = self.yolo(img_msg, self.model, self.confidence, self.nms)
            if len(self.filter) > 0:
                result.detected_objects = [
                    det for det in result.detected_objects if det.name in self.filter
                ]
            userdata.detections = result

            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
