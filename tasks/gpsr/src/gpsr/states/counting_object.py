import smach
import smach_ros
import rospy
from shapely.geometry.polygon import Polygon
from typing import List, Union
from lasr_vision_msgs.srv import YoloDetection, YoloDetectionRequest


class CountItems(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["detections_3d"],
            output_keys=["object_count"],
        )

    def execute(self, userdata):
        detections = userdata.detections_3d
        if not isinstance(detections, list) or len(detections) == 0:
            rospy.logwarn("[CountItems] No detections received.")
            userdata.object_count = 0
            return "succeeded"

        rospy.loginfo(f"[CountItems] Received {len(detections)} detections")
        userdata.object_count = len(detections)
        return "succeeded"


class CountObject(smach.StateMachine):
    def __init__(
        self,
        area_polygon: Polygon,
        model: str = "best.pt",
        objects: Union[List[str], None] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            output_keys=["object_count", "detections_3d"],
        )

        with self:

            def make_yolo_request(userdata, request: YoloDetectionRequest):
                request.model = model
                request.confidence = confidence
                request.nms = nms
                request.filter = objects if objects else []
                request.polygons = [area_polygon]
                return request

            smach.StateMachine.add(
                "DETECT_OBJECTS",
                smach_ros.ServiceState(
                    "/yolo/detect",
                    YoloDetection,
                    request_cb=make_yolo_request,
                    response_slots=["detected_objects"],
                ),
                transitions={
                    "succeeded": "COUNT_OBJECTS",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={
                    "detected_objects": "detections_3d",
                },
            )

            smach.StateMachine.add(
                "COUNT_OBJECTS",
                CountItems(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
