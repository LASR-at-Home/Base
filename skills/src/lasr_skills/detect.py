#!/usr/bin/env python3
from typing import List, Union

import rospy
import smach
from lasr_vision_msgs.srv import YoloDetection
from std_msgs.msg import String


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
        self.filter = filter or []
        self.confidence = confidence
        self.yolo = rospy.ServiceProxy("/yolo/detect", YoloDetection)
        self.yolo.wait_for_service()

    def execute(self, userdata):
        img_msg = userdata.img_msg
        try:
            userdata.detections = self.yolo(
                img_msg, self.model, self.confidence, self.filter
            )

            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
