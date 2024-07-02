#!/usr/bin/env python3
from typing import List
import rospy
import smach
from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Polygon, Point
from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    CroppedDetection,
)
from lasr_vision_msgs.msg import CDRequest, CDResponse


class RunAndProcessDetections(smach.StateMachine):
    def __init__(
        self,
        seating_area: Polygon,
        detection_service: str = "/vision/cropped_detection",
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transformed_pointclouds"],
            output_keys=["detections"],
        )
        self._detection_service = detection_service
        self._detection_client = rospy.ServiceProxy(
            self._detection_service, CroppedDetection
        )
        self.seating_area = seating_area

    class RunDetections(smach.State):
        def __init__(
            self,
            detection_client: rospy.ServiceProxy,
            seating_area: Polygon,
            method: str = "closest",
            use_mask: bool = True,
            yolo_model: str = "yolov8x-seg.pt",
            yolo_model_confidence: float = 0.5,
            yolo_nms_threshold: float = 0.3,
            return_sensor_reading: bool = False,
            object_names: List[str] = ["person", "chair"],
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["transformed_pointclouds"],
                output_keys=["detections"],
            )

            self.detector = detection_client
            self.seating_area = seating_area
            self.method = method
            self.use_mask = use_mask
            self.yolo_model = yolo_model
            self.yolo_model_confidence = yolo_model_confidence
            self.yolo_nms_threshold = yolo_nms_threshold
            self.return_sensor_reading = return_sensor_reading
            self.object_names = object_names

        def execute(self, userdata):
            try:
                self.detector.wait_for_service()
                request: CroppedDetectionRequest = CroppedDetectionRequest()

                for pointcloud in userdata.transformed_pointclouds:
                    request.requests.append(
                        CDRequest(
                            method=self.method,
                            use_mask=self.use_mask,
                            yolo_model=self.yolo_model,
                            yolo_model_confidence=self.yolo_model_confidence,
                            yolo_nms_threshold=self.yolo_nms_threshold,
                            return_sensor_reading=self.return_sensor_reading,
                            polygons=[self.seating_area],
                            pointcloud=pointcloud,
                            object_names=self.object_names,
                        )
                    )

                result: CroppedDetectionResponse = self.detector(request)
                userdata.detections = result.detections
            except Exception as e:
                rospy.logerr(f"Failed to run detections: {str(e)}")
                return "failed"
            return "succeeded"

    class ProcessDetections(smach.State):
        def __init__(
            self, closesness_distance: float = 0.5, overlap_threshold: float = 0.8
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections"],
                output_keys=["empty_seat_detections", "people_detections"],
            )
            self.closesness_distance = closesness_distance
            self.overlap_threshold = overlap_threshold

        def _euclidian_distance(self, point1: Point, point2: Point):
            return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) + (
                (point1.z - point2.z) ** 2
            ) ** 0.5

        def execute(self, userdata):
            try:
                seat_detections = []
                people_detections = []
                for detection_set in userdata.detections:
                    for detection in detection_set.detections:
                        if detection.name == "chair":
                            add_chair = True
                            for added_chair in seat_detections:
                                if (
                                    self._euclidian_distance(
                                        added_chair.pose.position,
                                        detection.pose.position,
                                    )
                                    < self.closesness_distance
                                ):
                                    add_chair = False
                                    break
                        if add_chair:
                            seat_detections.append(detection)
                        elif detection.name == "person":
                            people_detections.append(detection)
                            add_person = True
                            for added_person in people_detections:
                                if (
                                    self._euclidian_distance(
                                        added_person.pose.position,
                                        detection.pose.position,
                                    )
                                    < self.closesness_distance
                                ):
                                    add_person = False
                                    break
                        if add_person:
                            people_detections.append(detection)

                # Filter out people detections that are on seats
                filtered_seats = []
                for seat in seat_detections:
                    seat_removed = False
                    seat_polygon = ShapelyPolygon(
                        [(point.x, point.y) for point in seat.xyseg]
                    )
                    for person in people_detections:
                        person_polygon = ShapelyPolygon(
                            [(point.x, point.y) for point in person.xyseg]
                        )
                        if (
                            person_polygon.intersection(seat_polygon).area
                            / person_polygon.area
                            >= self.overlap_threshold
                        ):
                            seat_removed = True
                            break
                    if not seat_removed:
                        filtered_seats.append(seat)

                userdata.empty_seat_detections = filtered_seats
                userdata.people_detections = people_detections

            except Exception as e:
                rospy.logerr(f"Failed to process detections: {str(e)}")
                return "failed"
            return "succeeded"
