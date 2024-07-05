import smach
import smach_ros

from typing import List

from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Point, Polygon, PointStamped
from std_msgs.msg import Header


from lasr_skills import LookToPoint, Say

from lasr_vision_msgs.msg import CDRequest, CDResponse

from lasr_vision_msgs.srv import (
    CroppedDetectionRequest,
    CroppedDetectionResponse,
    CroppedDetection,
    Recognise,
)
import rospy

from receptionist.states import Introduce

import numpy as np


def _euclidian_distance(point1: Point, point2: Point):
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) + (
        (point1.z - point2.z) ** 2
    ) ** 0.5


class IntroduceAndSeatGuest(smach.StateMachine):

    class RecognisePeople(smach.State):

        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["people_detections"],
                output_keys=["matched_face_detections"],
            )
            self._recognise = rospy.ServiceProxy("/recognise", Recognise)
            self._recognise.wait_for_service()
            self._dataset = "receptionist"

        def execute(self, userdata):
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

    class HandleResponse(smach.State):

        def __init__(self):

            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["responses"],
                output_keys=["people_detections", "empty_seat_detections"],
            )
            self.closesness_distance = 0.5
            self.overlap_threshold = 0.8

        def execute(self, userdata):
            try:
                seat_detections = []
                people_detections = []
                response = userdata.responses[0]
                rospy.loginfo(f"Processing {len(response.detections_3d)} detections.")
                for index, detection in enumerate(response.detections_3d):
                    if (
                        np.isnan(detection.point.x)
                        or np.isnan(detection.point.y)
                        or np.isnan(detection.point.z)
                    ):
                        continue
                    rospy.loginfo(f"Processing detection: {detection.name}")

                    if (
                        detection.name == "chair"
                        and response.polygon_ids[index]
                        == 0  # ignore chairs inside the sofa area
                    ):
                        seat_detections.append(
                            (
                                detection,
                                response.cropped_imgs[index],
                                response.polygon_ids[index],
                            )
                        )
                        rospy.loginfo(f"Found chair at: {detection.point}")
                    elif detection.name == "person":
                        people_detections.append(
                            (
                                detection,
                                response.cropped_imgs[index],
                                response.polygon_ids[index],
                            )
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

                # Each is a list of (Detection, Image, PolygonIndex) pairs.
                userdata.empty_seat_detections = filtered_seats
                userdata.people_detections = people_detections
                rospy.loginfo(
                    f"Found {len(filtered_seats)} empty seats and {len(people_detections)} people."
                )

            except Exception as e:
                rospy.logerr(f"Failed to process detections: {str(e)}")
                return "failed"
            return "succeeded"

    class GetLookPoint(smach.State):

        def __init__(self, guest_id: str):
            smach.State.__init__(
                self,
                outcomes=["succeeded"],
                input_keys=["matched_face_detections"],
                output_keys=["look_point"],
            )
            self._guest_id = guest_id

        def execute(self, userdata):
            if len(userdata.matched_face_detections) == 0:
                userdata.look_point = PointStamped()
                return "succeeded"

            for detection in userdata.matched_face_detections:
                if detection.name == self._guest_id:
                    look_point = PointStamped(
                        point=detection.point, header=Header(frame_id="map")
                    )
                    userdata.look_point = look_point
                    return "succeeded"
            userdata.look_point = PointStamped()
            return "succeeded"

    class SelectSeat(smach.State):

        def __init__(self, sofa_position: Point, max_people_on_sofa: int):
            smach.State.__init__(
                self,
                outcomes=["succeeded_sofa", "succeeded_chair", "failed"],
                input_keys=["empty_seat_detections", "people_detections"],
                output_keys=["seat_position"],
            )
            self._sofa_position = sofa_position
            self._max_people_on_sofa = max_people_on_sofa

        def execute(self, userdata):

            num_people_on_sofa = 0

            for detection in userdata.people_detections:

                if detection[2] == 1:  # is inside the sofa area
                    num_people_on_sofa += 1

            if num_people_on_sofa < self._max_people_on_sofa:

                userdata.seat_position = PointStamped(
                    point=self._sofa_position, header=Header(frame_id="map")
                )
                return "succeeded_sofa"

            if len(userdata.empty_seat_detections) > 0:
                userdata.seat_position = PointStamped(
                    point=userdata.empty_seat_detections[0][0].point,
                    header=Header(frame_id="map"),
                )
                return "succeeded_chair"

            return "failed"

    def __init__(
        self,
        guest_id: str,
        guests_to_introduce_to: List[str],
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        sofa_position: Point,
        max_people_on_sofa: int,
    ):

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )

        """
        1. Detect all people and seats
        2. Introduce the guest to the other guests
        3. Seat the guest
        
        """

        rospy.sleep(5.0)

        with self:

            # Detect people and seats
            smach.StateMachine.add(
                "DETECT_PEOPLE_AND_SEATS",
                smach_ros.ServiceState(
                    "/vision/cropped_detection",
                    CroppedDetection,
                    request=CroppedDetectionRequest(
                        requests=[
                            CDRequest(
                                method="closest",
                                use_mask=True,
                                yolo_model="yolov8x-seg.pt",
                                yolo_model_confidence=0.5,
                                yolo_nms_threshold=0.3,
                                return_sensor_reading=False,
                                object_names=["person", "chair"],
                                polygons=[
                                    Polygon(
                                        points=[
                                            Point(
                                                x=point[0],
                                                y=point[1],
                                                z=0.0,
                                            )
                                            for point in seating_area.exterior.coords
                                        ]
                                    ),
                                    Polygon(
                                        points=[
                                            Point(
                                                x=point[0],
                                                y=point[1],
                                                z=0.0,
                                            )
                                            for point in sofa_area.exterior.coords
                                        ]
                                    ),
                                ],
                            )
                        ]
                    ),
                    output_keys=["response"],
                    response_slots=["responses"],
                ),
                transitions={
                    "succeeded": "HANDLE_RESPONSE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"response": "detections"},
            )

            smach.StateMachine.add(
                "HANDLE_RESPONSE",
                self.HandleResponse(),
                transitions={"succeeded": "RECOGNISE_PEOPLE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "RECOGNISE_PEOPLE",
                self.RecognisePeople(),
                transitions={
                    "succeeded": (
                        f"GET_LOOK_POINT_{guests_to_introduce_to[0]}"
                        if len(guests_to_introduce_to) > 0
                        else "succeeded"
                    ),
                    "failed": "failed",
                },
            )

            for i, guest_to_introduce_to in enumerate(guests_to_introduce_to):

                smach.StateMachine.add(
                    f"GET_LOOK_POINT_{guest_to_introduce_to}",
                    self.GetLookPoint(guest_to_introduce_to),
                    transitions={"succeeded": f"LOOK_AT_{guest_to_introduce_to}"},
                )

                smach.StateMachine.add(
                    f"LOOK_AT_{guest_to_introduce_to}",
                    LookToPoint(),
                    transitions={
                        "succeeded": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                        "aborted": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                        "timed_out": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                    },
                    remapping={"pointstamped": "look_point"},
                )

                smach.StateMachine.add(
                    f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                    Introduce(
                        guest_to_introduce=guest_id,
                        guest_to_introduce_to=guest_to_introduce_to,
                    ),
                    transitions={
                        "succeeded": (
                            "SELECT_SEAT"
                            if i == len(guests_to_introduce_to) - 1
                            else f"GET_LOOK_POINT_{guests_to_introduce_to[i+1]}"
                        )
                    },
                )

            smach.StateMachine.add(
                "SELECT_SEAT",
                self.SelectSeat(sofa_position, max_people_on_sofa),
                transitions={
                    "succeeded_sofa": "SAY_SOFA",
                    "succeeded_chair": "SAY_CHAIR",
                    "failed": "SAY_ANY",
                },
            )

            smach.StateMachine.add(
                "SAY_SOFA",
                Say(text="Please sit on the sofa"),
                transitions={
                    "succeeded": "LOOK_AT_SEAT",
                    "preempted": "LOOK_AT_SEAT",
                    "aborted": "LOOK_AT_SEAT",
                },
            )

            smach.StateMachine.add(
                "SAY_CHAIR",
                Say(text="Please sit on the chair that I am looking at"),
                transitions={
                    "succeeded": "LOOK_AT_SEAT",
                    "preempted": "LOOK_AT_SEAT",
                    "aborted": "LOOK_AT_SEAT",
                },
            )

            smach.StateMachine.add(
                "SAY_ANY",
                Say(text="Please sit on any empty seat"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "succeeded",
                    "aborted": "succeeded",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_SEAT",
                LookToPoint(),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "succeeded",
                    "timed_out": "succeeded",
                },
                remapping={"pointstamped": "seat_position"},
            )
