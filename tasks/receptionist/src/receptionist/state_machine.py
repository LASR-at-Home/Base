from typing import List, Tuple

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from lasr_skills import (
    AskAndListen,
    GoToLocation,
    LookToPoint,
    PlayMotion,
    Say,
    Wait,
    WaitForPersonInArea,
)
from lasr_vision_msgs.srv import Recognise
from receptionist.states import HandleGuest, IntroduceAndSeatGuest
from shapely.geometry import Polygon
from std_msgs.msg import Empty, Header


class Receptionist(smach.StateMachine):
    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        seat_pose: Pose,
        search_motions: List[str],
        seat_area: Polygon,
        sofa_area: Polygon,
        sofa_point: Point,
        host_data: dict,
        max_people_on_sofa: int = 3,
        face_detection_confidence: float = 0.2,
        known_host: bool = True,
        learn_guest_1: bool = True,
        sweep: bool = True,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        assert known_host or learn_guest_1, "Must learn at least one guest"

        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.seat_pose = seat_pose
        self.seat_area = seat_area

        with self:
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {"name": "", "drink": "", "detection": False},
                "guest2": {"name": "", "drink": "", "detection": False},
            }
            self.userdata.confidence = face_detection_confidence
            self.userdata.dataset = "receptionist"
            self.userdata.seat_position = PointStamped()

            def wait_cb(ud, msg):
                rospy.loginfo("Received start signal")
                return False

            smach.StateMachine.add(
                "WAIT_START",
                smach_ros.MonitorState(
                    "/receptionist/start",
                    Empty,
                    wait_cb,
                ),
                transitions={
                    "valid": "WAIT_START",
                    "invalid": "SAY_START",
                    "preempted": "WAIT_START",
                },
            )

            smach.StateMachine.add(
                "SAY_START",
                Say(text="Start of receptionist task."),
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
                "INTRODUCE_AND_SEAT_GUEST_1",
                IntroduceAndSeatGuest(
                    "guest1",
                    ["host"],
                    seat_area,
                    sofa_area,
                    sofa_point,
                    max_people_on_sofa,
                    search_motions,
                    sweep=sweep,
                ),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "failed": "SAY_RETURN_WAITING_AREA",
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
                "INTRODUCE_AND_SEAT_GUEST_2",
                IntroduceAndSeatGuest(
                    "guest2",
                    ["host", "guest1"],
                    seat_area,
                    sofa_area,
                    sofa_point,
                    max_people_on_sofa,
                    search_motions,
                    sweep=sweep,
                ),
                transitions={
                    "succeeded": "SAY_GOODBYE",
                    "failed": "SAY_GOODBYE",
                },
            )

            """
            Finish
            """
            smach.StateMachine.add(
                "SAY_GOODBYE",
                Say(text="Enjoy the party!"),
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
            transitions={"guest_2": "HANDLE_GUEST_2", "guest_1": "HANDLE_GUEST_1"},
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
                "succeeded": f"LOOK_EYES_{guest_id}",
                "preempted": "failed",
                "aborted": "failed",
            },
        )

        smach.StateMachine.add(
            f"LOOK_EYES_{guest_id}",
            PlayMotion(motion_name="look_very_left"),
            transitions={
                "succeeded": f"WAIT_{guest_id}_2",
                "preempted": "failed",
                "aborted": "failed",
            },
        )

        smach.StateMachine.add(
            f"WAIT_{guest_id}_2",
            Wait(1),
            transitions={
                "succeeded": f"INTRODUCE_AND_SEAT_GUEST_{guest_id}",
                "failed": f"INTRODUCE_AND_SEAT_GUEST_{guest_id}",
            },
        )
