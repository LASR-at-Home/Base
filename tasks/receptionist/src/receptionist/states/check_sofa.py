import smach
import smach_ros

from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Point, Polygon

from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    CroppedDetection,
)


class CheckSofa(smach.StateMachine):

    def __init__(
        self,, sofa_area: ShapelyPolygon, max_people_on_sofa: int
    ):

        smach.StateMachine.__init__(
            self,
            outcomes=["has_free_space", "no_free_space"],
        )

        self.sofa_area = sofa_area
        self.max_people_on_sofa = max_people_on_sofa

        with self:

            smach.StateMachine.add(
                "RUN_DETECTIONS",
                smach_ros.ServiceState(
                    "vision/cropped_detection",
                    CroppedDetection,
                    request=CroppedDetectionRequest(
                        method="closest",
                        use_mask=True,
                        yolo_model="yolov8x-seg.pt",
                        yolo_model_confidence=0.5,
                        yolo_nms_threshold=0.3,
                        return_sensor_reading=False,
                        polygons=[
                            Polygon(
                                points=[
                                    Point(
                                        x=point[0],
                                        y=point[1],
                                        z=0.0,
                                    )
                                    for point in sofa_area.exterior.coords
                                ]
                            )
                        ],
                    ),
                ),
                transitions={
                    "has_free_space": "has_free_space",
                    "no_free_space": "no_free_space",
                },
                response_cb=self.detections_cb,
            )

    def detections_cb(self, userdata, response):
        if len(response.detections) == 0:
            return "no_free_space"

        if len(response.detections) >= self.max_people_on_sofa:
            return "no_free_space"

        return "has_free_space"
