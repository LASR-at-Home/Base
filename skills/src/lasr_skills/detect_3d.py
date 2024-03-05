#!/usr/bin/env python3
import rospy
import smach

from sensor_msgs.msg import PointCloud2
from lasr_vision_msgs.srv import YoloDetection3D

from typing import List


class Detect3D(smach.State):

    def __init__(
        self,
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: List[str] | None = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )
        self.depth_topic = depth_topic
        self.model = model
        self.filter = filter if filter is not None else []
        self.confidence = confidence
        self.nms = nms
        self.yolo = rospy.ServiceProxy("/yolov8/detect3d", YoloDetection3D)
        self.yolo.wait_for_service()

    def execute(self, userdata):
        pcl_msg = rospy.wait_for_message(self.depth_topic, PointCloud2)
        try:
            result = self.yolo(pcl_msg, self.model, self.confidence, self.nms)
            result.detected_objects = [
                det for det in result.detected_objects if det.name in self.filter
            ]
            userdata.detections_3d = result
            return "succeeded"
        except rospy.ServiceException as e:
            rospy.logwarn(f"Unable to perform inference. ({str(e)})")
            return "failed"
