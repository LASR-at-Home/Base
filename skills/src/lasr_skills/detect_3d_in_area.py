#!/usr/bin/env python3

import smach

from lasr_skills import Detect3D
from lasr_shapely import LasrShapely


from typing import List, Union


class Detect3DInArea(smach.StateMachine):

    class FilterDetections(smach.State):

        def __init__(self, area_polygon):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections_3d"],
                output_keys=["detections_3d"],
            )
            self.area_polygon = area_polygon
            self.shapely = LasrShapely()

        def execute(self, userdata):
            satisfied_points = self.shapely.are_points_in_polygon_2d(
                userdata.area_polygon,
                [[pose[0], pose[1]] for (_, pose) in userdata.detections_3d],
            ).inside
            filtered_detections = [
                userdata.detections_3d[i]
                for i in range(0, len(userdata.detections_3d))
                if satisfied_points[i]
            ]
            userdata.detections_3d = filtered_detections

            return "succeeded"

    def __init__(
        self,
        area_polygon: List[List[float]],
        depth_topic: str = "/xtion/depth_registered/points",
        model: str = "yolov8n-seg.pt",
        filter: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["detections_3d"],
        )

        with self:

            smach.StateMachine.add(
                "DETECT_OBJECTS_3D",
                Detect3D(
                    depth_topic=depth_topic,
                    model=model,
                    filter=filter,
                    confidence=confidence,
                    nms=nms,
                ),
                transitions={"succeeded": "FILTER_DETECTIONS", "failed": "failed"},
            )
            smach.StateMachine.add(
                "FILTER_DETECTIONS",
                self.FilterDetections(area_polygon),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
