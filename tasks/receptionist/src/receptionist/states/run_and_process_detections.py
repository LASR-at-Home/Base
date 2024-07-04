#!/usr/bin/env python3
from typing import List
import rospy
import smach
import numpy as np
from receptionist.states import PointCloudSweep
from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Polygon, Point, PointStamped
from std_msgs.msg import Header
from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    CroppedDetection,
)
from lasr_vision_msgs.msg import CDRequest, CDResponse
from lasr_vision_msgs.srv import (
    Recognise,
    RecogniseRequest,
    DetectFaces,
    DetectFacesRequest,
)


def _euclidian_distance(point1: Point, point2: Point):
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) + (
        (point1.z - point2.z) ** 2
    ) ** 0.5


class RunAndProcessDetections(smach.StateMachine):
    def __init__(
        self,
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        detection_service: str = "/vision/cropped_detection",
    ):
        smach.StateMachine.__init__(
            self,
            outcomes=["succeeded", "failed"],
            input_keys=["transformed_pointclouds"],
            output_keys=[
                "empty_seat_detections",
                "people_detections",
                "matched_face_detections",
            ],
        )
        self._detection_service = detection_service
        self._detection_client = rospy.ServiceProxy(
            self._detection_service, CroppedDetection
        )
        self.seating_area = seating_area
        self.sofa_area = sofa_area

        with self:
            smach.StateMachine.add(
                "RUN_DETECTIONS",
                self.RunDetections(
                    detection_client=self._detection_client,
                    seating_area=self.seating_area,
                    sofa_area=self.sofa_area,
                    method="closest",
                    use_mask=True,
                    yolo_model="yolov8x-seg.pt",
                    yolo_model_confidence=0.8,
                    yolo_nms_threshold=0.3,
                    return_sensor_reading=False,
                    object_names=["person", "chair"],
                ),
                transitions={"succeeded": "PROCESS_DETECTIONS", "failed": "failed"},
                remapping={
                    "transformed_pointclouds": "transformed_pointclouds",
                    "detections": "detections",
                },
            )
            smach.StateMachine.add(
                "PROCESS_DETECTIONS",
                self.ProcessDetections(closesness_distance=0.5, overlap_threshold=0.8),
                transitions={"succeeded": "RUN_FACE_DETECTIONS", "failed": "failed"},
                remapping={
                    "detections": "detections",
                    "empty_seat_detections": "empty_seat_detections",
                    "people_detections": "people_detections",
                },
            )
            smach.StateMachine.add(
                "RUN_FACE_DETECTIONS",
                self.RunFaceDetections(),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

    class RunDetections(smach.State):
        def __init__(
            self,
            detection_client: rospy.ServiceProxy,
            seating_area: ShapelyPolygon,
            sofa_area: ShapelyPolygon,
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
            self.sofa_area = sofa_area
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
                    # .loginfo(f"Running detections on pointcloud: {pointcloud}")
                    request.requests.append(
                        CDRequest(
                            method=self.method,
                            use_mask=self.use_mask,
                            yolo_model=self.yolo_model,
                            yolo_model_confidence=self.yolo_model_confidence,
                            yolo_nms_threshold=self.yolo_nms_threshold,
                            return_sensor_reading=self.return_sensor_reading,
                            polygons=[
                                Polygon(
                                    points=[
                                        Point(
                                            x=point[0],
                                            y=point[1],
                                            z=0.0,
                                        )
                                        for point in self.seating_area.exterior.coords
                                    ]
                                ),
                                Polygon(
                                    points=[
                                        Point(
                                            x=point[0],
                                            y=point[1],
                                            z=0.0,
                                        )
                                        for point in self.sofa_area.exterior.coords
                                    ]
                                ),
                            ],
                            pointcloud=pointcloud,
                            object_names=self.object_names,
                        )
                    )
                result: CroppedDetectionResponse = self.detector(request)
                userdata.detections = result.responses
            except Exception as e:
                rospy.logerr(f"Failed to run detections: {str(e)}")
                return "failed"
            return "succeeded"

    class RunFaceDetections(smach.State):
        def __init__(
            self,
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["transformed_pointclouds", "people_detections"],
                output_keys=["matched_face_detections"],
            )

            self._dataset = "receptionist"
            self._recognise = rospy.ServiceProxy("/recognise", Recognise)

        def execute(self, userdata):
            self._recognise.wait_for_service()
            try:
                face_detections = []
                for person_detection in userdata.people_detections:
                    rospy.loginfo(
                        f"Running face detection on {person_detection[0].name}"
                    )
                    img_msg = person_detection[1]
                    recognise_result = self._recognise(img_msg, self._dataset, 0.2)
                    if len(recognise_result.detections) > 0:

                        face_detection = person_detection
                        face_detection[0].name = recognise_result.detections[0].name

                        rospy.loginfo(
                            f"Recognised face as {recognise_result.detections[0].name}"
                        )
                        face_detections.append(face_detection[0])

                userdata.matched_face_detections = face_detections
            except rospy.ServiceException as e:
                rospy.logwarn(f"Unable to perform face detection. ({str(e)})")
                return "failed"
            return "succeeded"

    class ProcessDetections(smach.State):
        def __init__(
            self, closesness_distance: float = 0.15, overlap_threshold: float = 0.8
        ):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["detections"],
                output_keys=[
                    "empty_seat_detections",
                    "people_detections",
                ],
            )
            self.closesness_distance = closesness_distance
            self.overlap_threshold = overlap_threshold

        def execute(self, userdata):
            try:
                seat_detections = []
                people_detections = []
                rospy.loginfo(f"Processing {len(userdata.detections)} detections.")
                for detection_set in userdata.detections:
                    for index, detection in enumerate(detection_set.detections_3d):
                        if (
                            np.isnan(detection.point.x)
                            or np.isnan(detection.point.y)
                            or np.isnan(detection.point.z)
                        ):
                            continue
                        rospy.loginfo(f"Processing detection: {detection.name}")
                        add_person = True
                        add_chair = True
                        if (
                            detection.name == "chair"
                            and detection_set.polygon_ids[index] == 0
                        ):
                            for added_chair in seat_detections:
                                if (
                                    _euclidian_distance(
                                        added_chair[0].point,
                                        detection.point,
                                    )
                                    < self.closesness_distance
                                ):
                                    add_chair = False
                            if add_chair:
                                seat_detections.append(
                                    (detection, detection_set.cropped_imgs[index])
                                )
                                rospy.loginfo(f"Found chair at: {detection.point}")
                        elif detection.name == "person":
                            for added_person in people_detections:
                                if (
                                    _euclidian_distance(
                                        added_person[0].point,
                                        detection.point,
                                    )
                                    < self.closesness_distance
                                ):
                                    add_person = False
                            if add_person:
                                people_detections.append(
                                    (detection, detection_set.cropped_imgs[index])
                                )
                                rospy.loginfo(f"Found person at: {detection.point}")

                # Filter out people detections that are on seats
                filtered_seats = []
                for seat_det in seat_detections:
                    seat = seat_det[0]
                    seat_removed = False
                    seat_seg = np.array([seat.xyseg]).reshape(-1, 2)
                    seat_polygon = ShapelyPolygon(seat_seg)
                    for person in people_detections:
                        person_seg = np.array([person[0].xyseg]).reshape(-1, 2)
                        person_polygon = ShapelyPolygon(person_seg)
                        if (
                            person_polygon.intersection(seat_polygon).area
                            / person_polygon.area
                            >= self.overlap_threshold
                        ):
                            rospy.loginfo(
                                f"Person detected on seat. Removing seat: {seat.point}"
                            )
                            seat_removed = True
                            break
                    if not seat_removed:
                        filtered_seats.append(seat_det)

                # Each is a list of (Detection, Image) pairs.
                userdata.empty_seat_detections = filtered_seats
                userdata.people_detections = people_detections
                rospy.loginfo(
                    f"Found {len(filtered_seats)} empty seats and {len(people_detections)} people."
                )

            except Exception as e:
                rospy.logerr(f"Failed to process detections: {str(e)}")
                return "failed"
            return "succeeded"


if __name__ == "__main__":
    rospy.init_node("run_and_process_detections")
    seat_area = [[-1.3, -0.1], [-1.8, 2.0], [1.9, 2.8], [2.8, 0.4]]
    seat_polygon = Polygon()
    for point in seat_area:
        seat_polygon.points.append(Point(x=point[0], y=point[1], z=0))
    while not rospy.is_shutdown():
        sm = smach.StateMachine(outcomes=["succeeded", "failed"])
        with sm:
            sm.userdata.transformed_pointclouds = []
            smach.StateMachine.add(
                "Sweep",
                PointCloudSweep(
                    sweep_points=[
                        (0.138, 1.2, 0.8),
                        (0.852, 1.29, 0.8),
                        (1.28, 1.48, 0.8),
                    ]
                ),
                transitions={
                    "succeeded": "RunAndProcessDetections",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "RunAndProcessDetections",
                RunAndProcessDetections(seating_area=seat_polygon),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )

        outcome = sm.execute()
        rospy.loginfo(f"Run and Process Detections completed with outcome: {outcome}")
        # wait for user to press enter before running again
        input("Press Enter to run again...")
        rospy.spin()
