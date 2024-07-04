from typing import List, Tuple
import smach
import smach_ros

from lasr_vision_msgs.srv import Recognise
from geometry_msgs.msg import Pose, Point, PointStamped
from shapely.geometry import Polygon
from lasr_skills import (
    GoToLocation,
    WaitForPersonInArea,
    Say,
    AskAndListen,
    PlayMotion,
    LookToPoint,
)
from std_msgs.msg import Header
from receptionist.states import (
    Introduce,
    SeatGuest,
    FindAndLookAt,
    HandleGuest,
    PointCloudSweep,
    RunAndProcessDetections,
    RecognisePeople,
    CheckSofa,
)


class Receptionist(smach.StateMachine):
    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        seat_pose: Pose,
        sweep_points: List[Point],
        seat_area: Polygon,
        sofa_area: Polygon,
        sofa_point: Point,
        host_data: dict,
        max_people_on_sofa: int = 3,
        face_detection_confidence: float = 0.2,
        known_host: bool = True,
        learn_guest_1: bool = True,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        assert known_host or learn_guest_1, "Must learn at least one guest"

        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.seat_pose = seat_pose
        self.seat_area = seat_area
        # self.sweep_points = sweep_points
        with self:
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {"name": "", "drink": "", "detection": False},
                "guest2": {"name": "", "drink": "", "detection": False},
            }
            self.userdata.confidence = face_detection_confidence
            self.userdata.dataset = "receptionist"
            self.userdata.seat_position = PointStamped()

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of receptionist task. Going to waiting area."),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_1",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_1",
                },
            )

            """
            First guest
            """

            self._goto_waiting_area(guest_id=1)

            smach.StateMachine.add(
                "INTRODUCE_ROBOT",
                Say(
                    text="Hello my name is Tiago, nice to meet you, I shall be your receptionist for today. I will try and be polite by looking at you when I speak, so I hope you will do the same by looking into my eyes whenever possible. First let me get to know you a little bit better."
                ),
                transitions={
                    "succeeded": f"HANDLE_GUEST_1",
                    "aborted": f"HANDLE_GUEST_1",
                    "preempted": f"HANDLE_GUEST_1",
                },
            )

            # """
            # GET GUEST ATTRIBUTES
            # """

            smach.StateMachine.add(
                "HANDLE_GUEST_1",
                HandleGuest("guest1", learn_guest_1),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_1",
                    "failed": "SAY_FOLLOW_GUEST_1",
                },
            )

            self._guide_guest(guest_id=1)

            smach.StateMachine.add(
                "SWEEP_GUEST_1",
                PointCloudSweep(
                    sweep_points=sweep_points,
                ),
                transitions={
                    "succeeded": "RUN_AND_PROCESS_DETECTIONS_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "RUN_AND_PROCESS_DETECTIONS_GUEST_1",
                RunAndProcessDetections(seat_area, sofa_area),
                transitions={
                    "succeeded": "FIND_AND_LOOK_AT_HOST_1",
                    "failed": "FIND_AND_LOOK_AT_HOST_1",
                },
                remapping={"empty_seat_point": "seat_position"},
            )

            # Look at host
            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_HOST_1",
                FindAndLookAt(guest_id=None),  # assume host is the only person there?
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_HOST",
                    "failed": "INTRODUCE_GUEST_1_TO_HOST",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_HOST",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="host"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_1_1",
                    "failed": "LOOK_AT_WAITING_GUEST_1_1",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_1_1",
                PlayMotion(motion_name="look_very_left"),
                transitions={
                    "succeeded": "INTRODUCE_HOST_TO_GUEST_1",
                    "aborted": "INTRODUCE_HOST_TO_GUEST_1",
                    "preempted": "INTRODUCE_HOST_TO_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_HOST_TO_GUEST_1",
                Introduce(guest_to_introduce="host", guest_to_introduce_to="guest1"),
                transitions={
                    "succeeded": "CHECK_SOFA_1",
                    "failed": "CHECK_SOFA_1",
                },
            )

            smach.StateMachine.add(
                "CHECK_SOFA_1",
                CheckSofa(sofa_area, max_people_on_sofa),
                transitions={
                    "has_free_space": "LOOK_AT_SOFA_1",
                    "no_free_space": "SEAT_GUEST_1_CHAIR",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_SOFA_1",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=sofa_point, header=Header(frame_id="map")
                    )
                ),
                transitions={
                    "succeeded": "SAY_GUEST_1_SIT_SOFA",
                    "aborted": "SAY_GUEST_1_SIT_SOFA",
                    "timed_out": "SAY_GUEST_1_SIT_SOFA",
                },
            )

            smach.StateMachine.add(
                "SAY_GUEST_1_SIT_SOFA",
                Say(text="Please sit on the sofa."),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "aborted": "SAY_RETURN_WAITING_AREA",
                    "preempted": "SAY_RETURN_WAITING_AREA",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_1_CHAIR",
                SeatGuest(),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "failed": "SAY_SEAT_GUEST_1_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_SEAT_GUEST_1_FAILED",
                Say(text="Please sit in an empty seat."),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "aborted": "SAY_RETURN_WAITING_AREA",
                    "preempted": "SAY_RETURN_WAITING_AREA",
                },
            )

            smach.StateMachine.add(
                "SAY_RETURN_WAITING_AREA",
                Say(text="Let me go back to the waiting area."),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_2",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_2",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_2",
                },
            )

            # """
            # Guest 2
            # """

            self._goto_waiting_area(2)

            smach.StateMachine.add(
                "HANDLE_GUEST_2",
                HandleGuest("guest2", False),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_2",
                    "failed": "SAY_FOLLOW_GUEST_2",
                },
            )

            self._guide_guest(guest_id=2)

            smach.StateMachine.add(
                "SWEEP_GUEST_2",
                PointCloudSweep(
                    sweep_points=sweep_points,
                ),
                transitions={
                    "succeeded": "RUN_AND_PROCESS_DETECTIONS_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "RUN_AND_PROCESS_DETECTIONS_GUEST_2",
                RunAndProcessDetections(seat_area, sofa_area),
                transitions={
                    "succeeded": "FIND_AND_LOOK_AT_HOST_2",
                    "failed": "FIND_AND_LOOK_AT_HOST_2",
                },
                remapping={"empty_seat_point": "seat_position"},
            )

            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_HOST_2",
                FindAndLookAt(
                    guest_id="host" if known_host else None,
                    mask=["guest1"] if not known_host else None,
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_HOST",
                    "failed": "INTRODUCE_GUEST_2_TO_HOST",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_2_TO_HOST",
                Introduce(guest_to_introduce="guest2", guest_to_introduce_to="host"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_2_1",
                    "failed": "LOOK_AT_WAITING_GUEST_2_1",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_1",
                PlayMotion(motion_name="look_very_left"),
                transitions={
                    "succeeded": "INTRODUCE_HOST_TO_GUEST_2",
                    "aborted": "INTRODUCE_HOST_TO_GUEST_2",
                    "preempted": "INTRODUCE_HOST_TO_GUEST_2",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_HOST_TO_GUEST_2",
                Introduce(guest_to_introduce="host", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "FIND_AND_LOOK_AT_GUEST_1",
                    "failed": "FIND_AND_LOOK_AT_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_GUEST_1",
                FindAndLookAt(
                    guest_id="guest1" if learn_guest_1 else None,
                    mask=["host"] if not learn_guest_1 else None,
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_GUEST_1",
                    "failed": "INTRODUCE_GUEST_2_TO_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_2_TO_GUEST_1",
                Introduce(guest_to_introduce="guest2", guest_to_introduce_to="guest1"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_2_2",
                    "failed": "LOOK_AT_WAITING_GUEST_2_2",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_2",
                PlayMotion(motion_name="look_very_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_GUEST_2",
                    "aborted": "INTRODUCE_GUEST_1_TO_GUEST_2",
                    "preempted": "INTRODUCE_GUEST_1_TO_GUEST_2",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_GUEST_2",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "CHECK_SOFA_2",
                    "failed": "CHECK_SOFA_2",
                },
            )

            smach.StateMachine.add(
                "CHECK_SOFA_2",
                CheckSofa(sofa_area, max_people_on_sofa),
                transitions={
                    "has_free_space": "LOOK_AT_SOFA_2",
                    "no_free_space": "SEAT_GUEST_2_CHAIR",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_SOFA_2",
                LookToPoint(
                    pointstamped=PointStamped(
                        point=sofa_point, header=Header(frame_id="map")
                    )
                ),
                transitions={
                    "succeeded": "SAY_GUEST_2_SIT_SOFA",
                    "aborted": "SAY_GUEST_2_SIT_SOFA",
                    "timed_out": "SAY_GUEST_2_SIT_SOFA",
                },
            )

            smach.StateMachine.add(
                "SAY_GUEST_2_SIT_SOFA",
                Say(text="Please sit on the sofa."),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "aborted": "SAY_RETURN_WAITING_AREA",
                    "preempted": "SAY_RETURN_WAITING_AREA",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_2_CHAIR",
                SeatGuest(),
                transitions={
                    "succeeded": "SAY_GOODBYE",
                    "failed": "SAY_SEAT_GUEST_2_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_SEAT_GUEST_2_FAILED",
                Say(text="Please sit in an empty seat."),
                transitions={
                    "succeeded": "SAY_GOODBYE",
                    "aborted": "SAY_GOODBYE",
                    "preempted": "SAY_GOODBYE",
                },
            )

            """
            Finish
            """
            smach.StateMachine.add(
                "SAY_GOODBYE",
                Say(
                    text="Goodbye fellow humans, I shall be going back where I came from"
                ),
                transitions={
                    "succeeded": "GO_TO_FINISH_LOCATION",
                    "aborted": "failed",
                    "preempted": "GO_TO_FINISH_LOCATION",
                },
            )

            smach.StateMachine.add(
                "GO_TO_FINISH_LOCATION",
                GoToLocation(wait_pose),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "failed": "GO_TO_FINISH_LOCATION",
                },
            )
            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "succeeded",
                },
            )

    def _goto_waiting_area(self, guest_id: int) -> None:
        """Adds the states to go to the waiting area.

        Args:
            guest_id (int): Identifier for the guest.
        """

        smach.StateMachine.add(
            f"GO_TO_WAIT_LOCATION_GUEST_{guest_id}",
            GoToLocation(self.wait_pose),
            transitions={
                "succeeded": f"SAY_WAITING_GUEST_{guest_id}",
                "failed": f"GO_TO_WAIT_LOCATION_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"SAY_WAITING_GUEST_{guest_id}",
            Say(text="I am waiting for a guest."),
            transitions={
                "succeeded": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
                "aborted": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
                "preempted": f"WAIT_FOR_PERSON_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"WAIT_FOR_PERSON_GUEST_{guest_id}",
            WaitForPersonInArea(self.wait_area),
            transitions={
                "succeeded": f"CHECK_GUEST_ID_GUEST_{guest_id}",
                "failed": f"CHECK_GUEST_ID_GUEST_{guest_id}",
            },
        )

        def check_guest_id(ud):
            if guest_id == 2:
                return "guest_2"
            else:
                return "guest_1"

        smach.StateMachine.add(
            f"CHECK_GUEST_ID_GUEST_{guest_id}",
            smach.CBState(check_guest_id, outcomes=["guest_1", "guest_2"]),
            transitions={"guest_2": "HANDLE_GUEST_2", "guest_1": "INTRODUCE_ROBOT"},
        )

    def _guide_guest(self, guest_id: int) -> None:
        """Adds the states to guide a guest to the
        seating area.

        Args:
            guest_id (int): Identifier for the guest.
        """

        smach.StateMachine.add(
            f"SAY_FOLLOW_GUEST_{guest_id}",
            Say(text="Please follow me, I will guide you to the other guests"),
            transitions={
                "succeeded": f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
                "preempted": "failed",
                "aborted": "failed",
            },
        )

        smach.StateMachine.add(
            f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
            GoToLocation(self.seat_pose),
            transitions={
                "succeeded": f"SAY_WAIT_GUEST_{guest_id}",
                "failed": f"GO_TO_SEAT_LOCATION_GUEST_{guest_id}",
            },
        )

        smach.StateMachine.add(
            f"SAY_WAIT_GUEST_{guest_id}",
            Say(text="Please wait here on my left."),
            transitions={
                "succeeded": f"SWEEP_GUEST_{guest_id}",
                "preempted": "failed",
                "aborted": "failed",
            },
        )
