import smach
import smach_ros

from typing import List

from shapely.geometry import Polygon as ShapelyPolygon
from geometry_msgs.msg import Point, Polygon, PointStamped
from std_msgs.msg import Header


from lasr_skills import LookToPoint, Say, PlayMotion, Wait, Detect3DInArea


from lasr_vision_msgs.srv import (
    Recognise,
)
import rospy

from receptionist.states import Introduce

import numpy as np
from copy import deepcopy


def _euclidian_distance(point1: Point, point2: Point):
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5


class IntroduceAndSeatGuest(smach.StateMachine):

    def __init__(
        self,
        guest_id: str,
        guests_to_introduce_to: List[str],
        seating_area: ShapelyPolygon,
        sofa_area: ShapelyPolygon,
        sofa_position: Point,
        max_people_on_sofa: int,
        motions: List[str],
    ):

        smach.StateMachine.__init__(
            self, outcomes=["succeeded", "failed"], input_keys=["guest_data"]
        )
        
        with self:

            class HandleSofaDetections(smach.State):
                def __init__(self):
                    smach.State.__init__(
                        self,
                        outcomes=["succeeded", "failed"],
                        input_keys=[
                            "detections",
                            "image_raw",
                        ],
                        output_keys=["sofa_detections"],
                    )

                def execute(self, userdata):
                    sofa_detections = []
                    # List of Detect3D messages
                    for detection in userdata.detections:
                        sofa_detections.append((detection, userdata.image_raw))
                    userdata.sofa_detections = sofa_detections
                    return "succeeded"

            class DetectPeopleAndSeats(smach.StateMachine):

                class HandleResponse(smach.State):

                    def __init__(self):
                        smach.State.__init__(
                            self,
                            outcomes=["succeeded", "failed"],
                            input_keys=[
                                "detections",
                                "motion_detections",
                                "image_raw",
                            ],
                            output_keys=["detections"],
                        )

                    def execute(self, userdata):

                        # List of Detect3D messages
                        for detection in userdata.motion_detections:
                            userdata.detections.append((detection, userdata.image_raw))
                        return "succeeded"

                def __init__(self, seating_area: ShapelyPolygon, motions: List[str]):
                    smach.StateMachine.__init__(
                        self,
                        outcomes=["succeeded", "failed"],
                        output_keys=["detections"],
                    )
                    self.seating_area = seating_area
                    self.motions = motions

                    with self:

                        self.userdata.detections = []

                        for i, motion in enumerate(self.motions):
                            smach.StateMachine.add(
                                f"PLAY_MOTION_{motion}",
                                PlayMotion(motion_name=motion),
                                transitions={
                                    "succeeded": f"SLEEP_1_{motion}",
                                    "aborted": f"SLEEP_1_{motion}",
                                    "preempted": f"SLEEP_1_{motion}",
                                },
                            )

                            smach.StateMachine.add(
                                f"SLEEP_1_{motion}",
                                Wait(1),
                                transitions={
                                    "succeeded": f"DETECT_PEOPLE_AND_SEATS_{motion}",
                                    "failed": f"DETECT_PEOPLE_AND_SEATS_{motion}",
                                },
                            )

                            smach.StateMachine.add(
                                f"DETECT_PEOPLE_AND_SEATS_{motion}",
                                Detect3DInArea(
                                    area_polygon=self.seating_area,
                                    filter=["person", "chair"],
                                ),
                                transitions={
                                    "succeeded": f"HANDLE_RESPONSE_{motion}",
                                    "failed": "failed",
                                },
                                remapping={
                                    "detections_3d": "motion_detections",
                                    "image_raw": "image_raw",
                                },
                            )

                            smach.StateMachine.add(
                                f"HANDLE_RESPONSE_{motion}",
                                self.HandleResponse(),
                                transitions={
                                    "succeeded": (
                                        f"PLAY_MOTION_{self.motions[i+1]}"
                                        if i < len(self.motions) - 1
                                        else "succeeded"
                                    ),
                                    "failed": "failed",
                                },
                            )

            class HandleResponses(smach.State):

                def __init__(self, expected_detections: List[str]):

                    smach.State.__init__(
                        self,
                        outcomes=["succeeded", "failed"],
                        input_keys=["sofa_detections", "detections"],
                        output_keys=[
                            "empty_seat_detections",
                            "matched_face_detections",
                            "num_people_on_sofa",
                        ],
                    )
                    self.closesness_distance = 0.5
                    self.overlap_threshold = 0.8
                    self._expected_detections = expected_detections
                    self._recognise = rospy.ServiceProxy("/recognise", Recognise)
                    self._recognise.wait_for_service()

                def execute(self, userdata):
                    seat_detections = []
                    matched_face_detections = []

                    def identify(img_msg):
                        recognise_result = self._recognise(img_msg, "receptionist", 0.2)
                        if len(recognise_result.detections) > 0:
                            rospy.loginfo(
                                f"Recognised face as {recognise_result.detections[0].name}"
                            )
                            return recognise_result.detections[0].name
                        rospy.loginfo("No face recognised")
                        return "unknown"

                    """
                    Identify all people in the detections
                    """

                    for index, (detection, img_msg) in enumerate(
                        userdata.sofa_detections
                    ):
                        if (
                            np.isnan(detection.point.x)
                            or np.isnan(detection.point.y)
                            or np.isnan(detection.point.z)
                        ):
                            continue
                        rospy.loginfo(f"Processing detection: {detection.name}")

                        if detection.name == "person":
                            detection.name = identify(img_msg)
                            matched_face_detections.append(detection)
                            rospy.loginfo(f"Identified person as: {detection.name}")

                    rospy.logwarn(
                        f"Seating area detections: {[(d.name, d.point) for d, _ in userdata.detections]}"
                    )
                    for index, (detection, img_msg) in enumerate(userdata.detections):
                        if (
                            np.isnan(detection.point.x)
                            or np.isnan(detection.point.y)
                            or np.isnan(detection.point.z)
                        ):
                            continue
                        rospy.loginfo(f"Processing detection: {detection.name}")

                        if detection.name == "person":
                            detection.name = identify(img_msg)
                            matched_face_detections.append(detection)
                            rospy.loginfo(f"Identified person as: {detection.name}")

                    print("*" * 50)
                    print(([(d.name, d.point) for d in matched_face_detections]))
                    print("*" * 50)

                    # """
                    # Filter out people that are too close to each other
                    # """

                    knowns = [
                        detection
                        for detection in matched_face_detections
                        if detection.name != "unknown"
                    ]

                    knowns = {
                        "host": [
                            detection
                            for detection in knowns
                            if detection.name == "host"
                        ],
                        "guest1": [
                            detection
                            for detection in knowns
                            if detection.name == "guest1"
                        ],
                    }

                    unknowns = [
                        detection
                        for detection in matched_face_detections
                        if detection.name == "unknown"
                    ]

                    filtered_face_detections = []
                    if knowns["host"]:
                        filtered_face_detections.append(
                            max(knowns["host"], key=lambda x: x.confidence)
                        )
                    if knowns["guest1"]:
                        filtered_face_detections.append(
                            max(knowns["guest1"], key=lambda x: x.confidence)
                        )

                    for unknown in unknowns:
                        add = True
                        for detection in filtered_face_detections:
                            if (
                                _euclidian_distance(unknown.point, detection.point)
                                < self.closesness_distance
                            ):
                                add = False
                                break
                        if add:
                            filtered_face_detections.append(unknown)

                    # if we have only one expected detection, and it is not present, assign it to the first unknown
                    if len(self._expected_detections) == 1:
                        if self._expected_detections[0] not in [
                            detection.name for detection in filtered_face_detections
                        ]:
                            rospy.logwarn(f"Expected guest not found..")
                            rospy.logwarn(f"Expected: {self._expected_detections[0]}")
                            rospy.logwarn(
                                [
                                    detection.name
                                    for detection in filtered_face_detections
                                ]
                            )
                            if len(filtered_face_detections) > 0:
                                filtered_face_detections[0].name = (
                                    self._expected_detections[0]
                                )
                            else:
                                rospy.logwarn(
                                    f"Failed to find expected guest {self._expected_detections[0]}"
                                )

                    print("+" * 50)
                    print(([(d.name, d.point) for d in filtered_face_detections]))
                    print("+" * 50)

                    # if we have two expected detections, and one is missing, assign it to the first unknown
                    if len(self._expected_detections) == 2:
                        if self._expected_detections[0] not in [
                            detection.name for detection in filtered_face_detections
                        ] and self._expected_detections[1] in [
                            detection.name for detection in filtered_face_detections
                        ]:
                            if len(filtered_face_detections) > 0:
                                other = next(
                                    detection
                                    for detection in filtered_face_detections
                                    if detection.name == self._expected_detections[1]
                                )

                                other_detections = [
                                    detection
                                    for detection in filtered_face_detections
                                    if detection.name == "unknown"
                                ]
                                if other_detections:
                                    furthest_unknown = max(
                                        other_detections,
                                        key=lambda x: _euclidian_distance(
                                            x.point, other.point
                                        ),
                                    )
                                    furthest_unknown.name = self._expected_detections[0]
                            else:
                                rospy.logwarn(
                                    f"Failed to find expected guest {self._expected_detections[1]}"
                                )
                        elif self._expected_detections[1] not in [
                            detection.name for detection in filtered_face_detections
                        ] and self._expected_detections[0] in [
                            detection.name for detection in filtered_face_detections
                        ]:
                            if len(filtered_face_detections) > 0:
                                other = next(
                                    detection
                                    for detection in filtered_face_detections
                                    if detection.name == self._expected_detections[0]
                                )
                                other_detections = [
                                    detection
                                    for detection in filtered_face_detections
                                    if detection.name == "unknown"
                                ]
                                if other_detections:
                                    furthest_unknown = max(
                                        other_detections,
                                        key=lambda x: _euclidian_distance(
                                            x.point, other.point
                                        ),
                                    )
                                    furthest_unknown.name = self._expected_detections[1]
                            else:
                                rospy.logwarn(
                                    f"Failed to find expected guest {self._expected_detections[0]}"
                                )

                    print("-" * 50)
                    print(([(d.name, d.point) for d in filtered_face_detections]))
                    print("-" * 50)

                    """
                    Extract all seats that are not occupied by people
                    """

                    for index, (detection, img_msg) in enumerate(userdata.detections):
                        if (
                            np.isnan(detection.point.x)
                            or np.isnan(detection.point.y)
                            or np.isnan(detection.point.z)
                        ):
                            continue
                        rospy.loginfo(f"Processing detection: {detection.name}")

                        if detection.name == "chair":
                            seat_detections.append(detection)
                            rospy.loginfo(f"Found chair at: {detection.point}")

                    filtered_seats = []
                    for seat in seat_detections:
                        seat_removed = False
                        seat_seg = np.array([seat.xyseg]).reshape(-1, 2)
                        seat_polygon = ShapelyPolygon(seat_seg)
                        for person in filtered_face_detections:
                            person_seg = np.array([person.xyseg]).reshape(-1, 2)
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
                            filtered_seats.append(seat)

                    rospy.loginfo(
                        f"Filtered face detections: {[detection.name for detection in filtered_face_detections]}"
                    )

                    userdata.empty_seat_detections = filtered_seats
                    userdata.matched_face_detections = filtered_face_detections
                    userdata.num_people_on_sofa = len(userdata.sofa_detections)
                    rospy.loginfo(
                        f"{[(detection.name, detection.point) for detection in filtered_face_detections]}"
                    )
                    return "succeeded"

            class GetLookPoint(smach.State):

                def __init__(self, guest_id: str):
                    smach.State.__init__(
                        self,
                        outcomes=["succeeded", "failed"],
                        input_keys=["matched_face_detections"],
                        output_keys=["look_point"],
                    )
                    self._guest_id = guest_id

                def execute(self, userdata):
                    if len(userdata.matched_face_detections) == 0:
                        userdata.look_point = PointStamped()
                        rospy.logwarn(f"Failed to find guest: {self._guest_id}")
                        return "failed"

                    for detection in userdata.matched_face_detections:
                        if detection.name == self._guest_id:
                            look_point = PointStamped(
                                point=detection.point, header=Header(frame_id="map")
                            )
                            look_point.point.z = 0.75  # fixed height
                            userdata.look_point = look_point
                            return "succeeded"

                    rospy.logwarn(f"Failed to find guest: {self._guest_id}")
                    userdata.look_point = PointStamped()
                    return "succeeded"

            class SelectSeat(smach.State):

                def __init__(self, sofa_position: Point, max_people_on_sofa: int):
                    smach.State.__init__(
                        self,
                        outcomes=[
                            "succeeded_sofa_left",
                            "succeeded_sofa_right",
                            "succeeded_chair",
                            "failed",
                        ],
                        input_keys=[
                            "num_people_on_sofa",
                            "empty_seat_detections",
                            "sofa_detections",
                        ],
                        output_keys=["seat_position"],
                    )
                    self._sofa_position = sofa_position
                    self._max_people_on_sofa = max_people_on_sofa

                def execute(self, userdata):

                    rospy.loginfo(f"Num people on sofa: {userdata.num_people_on_sofa}")
                    if userdata.num_people_on_sofa < self._max_people_on_sofa:
                        try:
                            userdata.seat_position = PointStamped(
                                point=self._sofa_position,
                                header=Header(frame_id="map"),
                            )
                            if userdata.num_people_on_sofa == 0:
                                return "succeeded_sofa_left"
                            else:
                                sat_person_centroid = userdata.sofa_detections[0].point
                                sofa_points = rospy.get_param("/receptionist/sofa_area")
                                bottom_left = sofa_points[0]
                                top_left = sofa_points[1]
                                top_right = sofa_points[2]
                                bottom_right = sofa_points[3]
                                bottom_mid_point = Point(
                                    x=(bottom_left.x + bottom_right.x) / 2,
                                    y=(bottom_left.y + bottom_right.y) / 2,
                                )
                                top_mid_point = Point(
                                    x=(top_left.x + top_right.x) / 2,
                                    y=(top_left.y + top_right.y) / 2,
                                )

                                left_rectangle = ShapelyPolygon(
                                    [
                                        bottom_left,
                                        bottom_mid_point,
                                        top_mid_point,
                                        top_left,
                                    ]
                                )

                                if left_rectangle.contains(sat_person_centroid):
                                    return "succeeded_sofa_right"
                                else:
                                    return "succeeded_sofa_left"
                        except:
                            return "succeeded_sofa_left"

                    if len(userdata.empty_seat_detections) > 0:
                        seat_position = PointStamped(
                            point=userdata.empty_seat_detections[0].point,
                            header=Header(frame_id="map"),
                        )
                        seat_position.point.z = 0.5  # fixed height
                        userdata.seat_position = seat_position

                        return "succeeded_chair"

                    return "failed"

            smach.StateMachine.add(
                "LOOK_AT_SOFA",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=sofa_position, header=Header(frame_id="map")
                    )
                ),
                transitions={
                    "succeeded": "SLEEP_1",
                    "aborted": "SLEEP_1",
                    "timed_out": "SLEEP_1",
                },
            )

            smach.StateMachine.add(
                "SLEEP_1",
                Wait(1),
                transitions={
                    "succeeded": "DETECT_PEOPLE_ON_SOFA",
                    "failed": "DETECT_PEOPLE_ON_SOFA",
                },
            )

            smach.StateMachine.add(
                "DETECT_PEOPLE_ON_SOFA",
                Detect3DInArea(
                    area_polygon=sofa_area,
                    filter=["person"],
                ),
                transitions={
                    "succeeded": "HANDLE_SOFA_DETECTIONS",
                    "failed": "failed",
                },
                remapping={"detections_3d": "detections"},
            )

            smach.StateMachine.add(
                "HANDLE_SOFA_DETECTIONS",
                HandleSofaDetections(),
                transitions={
                    "succeeded": "DETECT_PEOPLE_AND_SEATS",
                    "failed": "failed",
                },
                remapping={
                    "detections": "detections",
                    "sofa_detections": "sofa_detections",
                    "image_raw": "image_raw",
                },
            )

            # Detect people and seats in the seating area
            smach.StateMachine.add(
                "DETECT_PEOPLE_AND_SEATS",
                DetectPeopleAndSeats(seating_area, motions),
                transitions={"succeeded": "LOOK_CENTRE", "failed": "failed"},
            )

            smach.StateMachine.add(
                "LOOK_CENTRE",
                PlayMotion(motion_name="look_centre"),
                transitions={
                    "succeeded": "HANDLE_RESPONSES",
                    "aborted": "HANDLE_RESPONSES",
                    "preempted": "HANDLE_RESPONSES",
                },
            )

            # Handle the responses from the detections
            smach.StateMachine.add(
                "HANDLE_RESPONSES",
                HandleResponses(guests_to_introduce_to),
                transitions={
                    "succeeded": (
                        f"GET_LOOK_POINT_{guests_to_introduce_to[0]}"
                        if len(guests_to_introduce_to) > 0
                        else "succeeded"
                    ),
                    "failed": (
                        f"GET_LOOK_POINT_{guests_to_introduce_to[0]}"
                        if len(guests_to_introduce_to) > 0
                        else "succeeded"
                    ),
                },
            )

            # Introduce the guest to the other guests
            for i, guest_to_introduce_to in enumerate(guests_to_introduce_to):

                # Get the look point for the guest to introduce to
                smach.StateMachine.add(
                    f"GET_LOOK_POINT_{guest_to_introduce_to}",
                    GetLookPoint(guest_to_introduce_to),
                    transitions={
                        "succeeded": f"LOOK_AT_{guest_to_introduce_to}",
                        "failed": f"LOOK_CENTRE_BACKUP_{guest_to_introduce_to}",
                    },
                )

                smach.StateMachine.add(
                    f"LOOK_CENTRE_BACKUP_{guest_to_introduce_to}",
                    PlayMotion(motion_name="look_centre"),
                    transitions={
                        "succeeded": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                        "aborted": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                        "preempted": f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                    },
                )

                # Look at the guest to introduce to
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

                # Introduce the guest to the other guest
                smach.StateMachine.add(
                    f"INTRODUCE_{guest_id}_TO_{guest_to_introduce_to}",
                    Introduce(
                        guest_to_introduce=guest_id,
                        guest_to_introduce_to=guest_to_introduce_to,
                        describe_features=guest_to_introduce_to != "host",
                    ),
                    transitions={
                        "succeeded": f"LOOK_AT_WAITING_GUEST_{guest_id}_{guest_to_introduce_to}",
                    },
                )

                smach.StateMachine.add(
                    f"LOOK_AT_WAITING_GUEST_{guest_id}_{guest_to_introduce_to}",
                    PlayMotion(motion_name="look_very_left"),
                    transitions={
                        "succeeded": f"INTRODUCE_{guest_to_introduce_to}_TO_{guest_id}",
                        "aborted": f"INTRODUCE_{guest_to_introduce_to}_TO_{guest_id}",
                        "preempted": f"INTRODUCE_{guest_to_introduce_to}_TO_{guest_id}",
                    },
                )

                smach.StateMachine.add(
                    f"INTRODUCE_{guest_to_introduce_to}_TO_{guest_id}",
                    Introduce(
                        guest_to_introduce=guest_to_introduce_to,
                        guest_to_introduce_to=guest_id,
                        describe_features=guest_to_introduce_to == "host",
                    ),
                    transitions={
                        "succeeded": (
                            "SELECT_SEAT"
                            if i == len(guests_to_introduce_to) - 1
                            else f"GET_LOOK_POINT_{guests_to_introduce_to[i+1]}"
                        ),
                    },
                )

            # Select a seat for the guest
            smach.StateMachine.add(
                "SELECT_SEAT",
                SelectSeat(sofa_position, max_people_on_sofa),
                transitions={
                    "succeeded_sofa_left": "SAY_SOFA_LEFT",
                    "succeeded_sofa_right": "SAY_SOFA_RIGHT",
                    "succeeded_chair": "SAY_CHAIR",
                    "failed": "LOOK_CENTRE_SEAT",
                },
            )

            smach.StateMachine.add(
                "LOOK_CENTRE_SEAT",
                PlayMotion(motion_name="look_centre"),
                transitions={
                    "succeeded": "SAY_ANY",
                    "aborted": "SAY_ANY",
                    "preempted": "SAY_ANY",
                },
            )

            # Say to sit on the sofa
            smach.StateMachine.add(
                "SAY_SOFA_LEFT",
                Say(text="Please sit on left side of the sofa that I am looking at"),
                transitions={
                    "succeeded": "LOOK_AT_SEAT",
                    "preempted": "LOOK_AT_SEAT",
                    "aborted": "LOOK_AT_SEAT",
                },
            )
            # Say to sit on the sofa
            smach.StateMachine.add(
                "SAY_SOFA_RIGHT",
                Say(
                    text="Please sit on the right side of the sofa that I am looking at"
                ),
                transitions={
                    "succeeded": "LOOK_AT_SEAT",
                    "preempted": "LOOK_AT_SEAT",
                    "aborted": "LOOK_AT_SEAT",
                },
            )

            # Say to sit on the chair
            smach.StateMachine.add(
                "SAY_CHAIR",
                Say(text="Please sit on the chair that I am looking at"),
                transitions={
                    "succeeded": "LOOK_AT_SEAT",
                    "preempted": "LOOK_AT_SEAT",
                    "aborted": "LOOK_AT_SEAT",
                },
            )

            # Say to sit on any empty seat
            smach.StateMachine.add(
                "SAY_ANY",
                Say(text="Please sit on any empty seat"),
                transitions={
                    "succeeded": "WAIT_SEAT",
                    "preempted": "WAIT_SEAT",
                    "aborted": "WAIT_SEAT",
                },
            )

            # Look at the seat
            smach.StateMachine.add(
                "LOOK_AT_SEAT",
                LookToPoint(),
                transitions={
                    "succeeded": "WAIT_SEAT",
                    "aborted": "WAIT_SEAT",
                    "timed_out": "WAIT_SEAT",
                },
                remapping={"pointstamped": "seat_position"},
            )

            smach.StateMachine.add(
                "WAIT_SEAT",
                Wait(3),
                transitions={"succeeded": "succeeded", "failed": "failed"},
            )
