import smach
import smach_ros
from shapely.geometry.polygon import Polygon
from typing import List, Literal, Optional
from lasr_vision_msgs.srv import CroppedDetection, CroppedDetectionRequest
from lasr_vision_msgs.msg import CDRequest


class ObjectComparison(smach.StateMachine):

    _object_comp_list: List[str] = [
        "biggest",
        "largets",
        "smallest",
        "heaviest",
        "lightest",
        "thinnest",
    ]

    _biggest_list: List[str] = []

    _largest_list: List[str] = []

    _smallest_list: List[str] = []

    _heaviest_list: List[str] = []

    _lightest_list: List[str] = []

    _thinnest_list: List[str] = []

    _query: Literal[
        "biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"
    ]

    def _compare_objects(self, userdata):
        detections = userdata.responses[0].detections_3d
        if self._query == "biggest":
            biggest_object = next(
                (obj for obj in self._biggest_list if obj in detections), None
            )
            if not biggest_object:
                return "failed"
            userdata.query_result = biggest_object
            return "succeeded"
        elif self._query == "largest":
            largest_object = next(
                (obj for obj in self._largest_list if obj in detections), None
            )
            if not largest_object:
                return "failed"
            userdata.query_result = largest_object
            return "succeeded"
        elif self._query == "smallest":
            smallest_object = next(
                (obj for obj in self._smallest_list if obj in detections), None
            )
            if not smallest_object:
                return "failed"
            userdata.query_result = smallest_object
            return "succeeded"
        elif self._query == "heaviest":
            heaviest_object = next(
                (obj for obj in self._heaviest_list if obj in detections), None
            )
            if not heaviest_object:
                return "failed"
            userdata.query_result = heaviest_object
            return "succeeded"
        elif self._query == "lightest":
            lightest_object = next(
                (obj for obj in self._lightest_list if obj in detections), None
            )
            if not lightest_object:
                return "failed"
            userdata.query_result = lightest_object
            return "succeeded"
        elif self._query == "thinnest":
            thinnest_object = next(
                (obj for obj in self._thinnest_list if obj in detections), None
            )
            if not thinnest_object:
                return "failed"
            userdata.query_result = thinnest_object
            return "succeeded"
        else:
            return "failed"

    def __init__(
        self,
        query: Literal[
            "biggest", "largest", "smallest", "heaviest", "lightest", "thinnest"
        ],
        area_polygon: Polygon,  # input key
        model: str = "yolov8x-seg.pt",
        objects: Optional[List[str]] = None,
        confidence: float = 0.5,
        nms: float = 0.3,
    ):

        self._query = query

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], output_keys=["query_result"]
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
                    "succeeded": "COMPARE_OBJECTS",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "COMPARE_OBJECTS",
                smach.CBState(
                    self._compare_objects,
                    outcomes=["succeeded", "failed"],
                    input_keys=["responses"],
                ),
                transitions={"succeeded": "succeeded"},
            )
