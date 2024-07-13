import smach
import smach_ros
from shapely.geometry.polygon import Polygon
from typing import List, Union
import rospy

from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest
from lasr_vision_msgs.msg import CDRequest


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
            filtered_detections = userdata.detections_3d[0]
            rospy.loginfo(filtered_detections)
            object_counts = self.count_types(filtered_detections.detections_3d)
            userdata.object_count = object_counts  # output key
            return "succeeded"

    def __init__(
        self,
        area_polygon: Polygon,  # input key
        model: str = "yolov8x-seg.pt",
        objects: Union[List[str], None] = None,  # input key
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
                smach_ros.ServiceState(
                    "/vision/cropped_detection",
                    CroppedDetection,
                    request=CroppedDetectionRequest(
                        requests=[
                            CDRequest(
                                method="closest",
                                use_mask=True,
                                yolo_model=model,
                                yolo_model_confidence=confidence,
                                yolo_nms_threshold=nms,
                                return_sensor_reading=False,
                                object_names=objects,
                                polygons=[area_polygon],
                            )
                        ]
                    ),
                    output_keys=["responses"],
                    response_slots=["responses"],
                ),
                transitions={
                    "succeeded": "COUNTOBJECTS",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "COUNTOBJECTS",
                self.CountItems(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
                remapping={"detections_3d": "responses"},
            )
