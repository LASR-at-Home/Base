#!/usr/bin/env python3
import smach
from lasr_skills import Detect3DInArea
from shapely.geometry.polygon import Polygon
from typing import List, Union
import rospy


class CountObject(smach.State):
    class CountItems(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["object_count"],
            )

        def count_types(self, detections):
            object_counts = len(detections)
            return object_counts

        def execute(self, userdata):
            filtered_detections = userdata.detections_3d
            rospy.loginfo(filtered_detections)
            object_counts = self.count_types(filtered_detections.detected_objects)
            userdata.object_count = object_counts # output key
            return "succeeded"

    def __init__(
        self,
        area_polygon: Polygon, # input key
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        Object: Union[List[str], None] = None, # input key
        confidence: float = 0.5,
        nms: float = 0.3,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d", "object_dict", "say_text"],
        )

        with self:
            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                Detect3DInArea(
                    depth_topic=depth_topic,
                    model=model,
                    filter=Object,
                    confidence=confidence,
                    nms=nms,
                    Polygon = area_polygon,
                ),
                transitions={"succeeded": "COUNTOBJECTS", "failed": "failed"},
            )

            smach.StateMachine.add(
                "COUNTOBJECTS",
                self.CountItems(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

