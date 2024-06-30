import smach

from geometry_msgs.msg import Pose
from shapely.geometry import Polygon
from lasr_skills import (
    GoToLocation,
    WaitForPersonInArea,
    Say,
    AskAndListen,
    PlayMotion,
)
from receptionist.states import (
    ParseNameAndDrink,
    GetGuestAttributes,
    Introduce,
    SeatGuest,
    FindAndLookAt,
    ReceptionistLearnFaces,
    ParseTranscribedInfo,
    HandleGuest,
)


class Receptionist(smach.StateMachine):
    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        seat_pose: Pose,
        seat_area: Polygon,
        sofa_area: Polygon,
        host_data: dict,
        max_people_on_sofa: int = 3,
        face_detection_confidence: float = 0.2,
    ):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])
        self.wait_pose = wait_pose
        self.wait_area = wait_area
        self.seat_pose = seat_pose
        self.seat_area = seat_area
        with self:
            self.userdata.guest_data = {
                "host": host_data,
                "guest1": {"name": ""},
                "guest2": {"name": ""},
            }
            self.userdata.confidence = face_detection_confidence
            self.userdata.dataset = "receptionist"

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

            self._goto_waiting_area(1)
            """ 
            GET GUEST ATTRIBUTES
            """

            smach.StateMachine.add(
                "HANDLE_GUEST_1",
                HandleGuest("guest1"),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_1",
                    "failed": "SAY_FOLLOW_GUEST_1",
                },
            )

            self._guide_guest(1)

            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_HOST_1",
                FindAndLookAt(
                    "host",
                    [
                        [0.0, 0.0],
                        [-0.8, 0.0],
                        [0.8, 0.0],
                    ],
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_HOST",
                    "failed": "SAY_NO_HOST_1",
                },
            )

            smach.StateMachine.add(
                "SAY_NO_HOST_1",
                Say(text="Wow, I can't find the host, I'm sure they're here"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_1_1",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_1_1",
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_HOST",
                    "aborted": "INTRODUCE_GUEST_1_TO_HOST",
                    "preempted": "INTRODUCE_GUEST_1_TO_HOST",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_HOST",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="host"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_1_2",
                    "failed": "LOOK_AT_WAITING_GUEST_1_2",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_1_2",
                PlayMotion(motion_name="look_left"),
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
                    "succeeded": "SEAT_GUEST_1",
                    "failed": "SEAT_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_1",
                SeatGuest(seat_area, sofa_area, max_people_on_sofa),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "failed": "SAY_SEAT_GUEST_1_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_SEAT_GUEST_1_FAILED",
                Say(text="I can't see a free seat, please sit down somewhere."),
                transitions={
                    "succeeded": "SAY_RETURN_WAITING_AREA",
                    "aborted": "SAY_RETURN_WAITING_AREA",
                    "preempted": "SAY_RETURN_WAITING_AREA",
                },
            )

            """
            Guest 2
            """

            smach.StateMachine.add(
                "SAY_RETURN_WAITING_AREA",
                Say(text="Let me go back to the waiting area."),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_2",
                    "aborted": "GO_TO_WAIT_LOCATION_GUEST_2",
                    "preempted": "GO_TO_WAIT_LOCATION_GUEST_2",
                },
            )

            self._goto_waiting_area(2)

            smach.StateMachine.add(
                "HANDLE_GUEST_2",
                HandleGuest("guest2"),
                transitions={
                    "succeeded": "SAY_WAIT_GUEST_2",
                    "failed": "SAY_WAIT_GUEST_2",
                },
            )

            self._guide_guest(2)

            # INTRODUCE GUEST 2 TO HOST

            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_HOST_2",
                FindAndLookAt(
                    "host",
                    [
                        [0.0, 0.0],
                        [-0.8, 0.0],
                        [0.8, 0.0],
                    ],
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_HOST",
                    "failed": "LOOK_AT_WAITING_GUEST_2_1",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_1",
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_HOST",
                    "aborted": "INTRODUCE_GUEST_2_TO_HOST",
                    "preempted": "INTRODUCE_GUEST_2_TO_HOST",
                },
            )

            # Check if host is sat where they are sat
            # Look at the host

            smach.StateMachine.add(
                "INTRODUCE_GUEST_2_TO_HOST",
                Introduce(guest_to_introduce="guest2", guest_to_introduce_to="host"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_2_2",
                    "failed": "LOOK_AT_WAITING_GUEST_2_2",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_2",
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_HOST_TO_GUEST_2",
                    "aborted": "INTRODUCE_GUEST_HOST_TO_GUEST_2",
                    "preempted": "INTRODUCE_GUEST_HOST_TO_GUEST_2",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_HOST_TO_GUEST_2",
                Introduce(guest_to_introduce="host", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "SAY_WAIT_GUEST_2_SEATED_GUEST_1",
                    "failed": "SAY_WAIT_GUEST_2_SEATED_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "SAY_WAIT_GUEST_2_SEATED_GUEST_1",
                Say(text="Can the seated guest look into my eyes please."),
                transitions={
                    "succeeded": "FIND_AND_LOOK_AT_GUEST_1",
                    "preempted": "FIND_AND_LOOK_AT_GUEST_1",
                    "aborted": "FIND_AND_LOOK_AT_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "FIND_AND_LOOK_AT_GUEST_1",
                FindAndLookAt(
                    "guest1",
                    [
                        [0.0, 0.0],
                        [-0.8, 0.0],
                        [0.8, 0.0],
                    ],
                ),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_GUEST_1",
                    "failed": "LOOK_AT_WAITING_GUEST_2_3",
                },
            )

            # Check if host is sat where they are sat
            # Look at the host

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_3",
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_GUEST_1",
                    "aborted": "INTRODUCE_GUEST_2_TO_GUEST_1",
                    "preempted": "INTRODUCE_GUEST_2_TO_GUEST_1",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_2_TO_GUEST_1",
                Introduce(guest_to_introduce="guest2", guest_to_introduce_to="guest1"),
                transitions={
                    "succeeded": "LOOK_AT_WAITING_GUEST_2_4",
                    "failed": "LOOK_AT_WAITING_GUEST_2_4",
                },
            )

            smach.StateMachine.add(
                "LOOK_AT_WAITING_GUEST_2_4",
                PlayMotion(motion_name="look_left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_GUEST_2",
                    "aborted": "INTRODUCE_GUEST_1_TO_GUEST_2",
                    "preempted": "INTRODUCE_GUEST_1_TO_GUEST_2",
                },
            )

            # Look at guest 1
            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_GUEST_2",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "SEAT_GUEST_2",
                    "failed": "SEAT_GUEST_2",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_2",
                SeatGuest(seat_area),
                transitions={
                    "succeeded": "SAY_GOODBYE",
                    "failed": "SAY_SEAT_GUEST_2_FAILED",
                },
            )

            smach.StateMachine.add(
                "SAY_SEAT_GUEST_2_FAILED",
                Say(text="I can't see a free seat, please sit down somewhere."),
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
            Say(text="I am waiting for a guest. Please open the door."),
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
                "succeeded": f"HANDLE_GUEST_{guest_id}",
                "failed": f"HANDLE_GUEST_{guest_id}",
            },
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
            Say(
                text="Please wait here on my left. Can the seated host look into my eyes please."
            ),
            transitions={
                "succeeded": f"FIND_AND_LOOK_AT_HOST_{guest_id}",
                "preempted": "failed",
                "aborted": "failed",
            },
        )

    @smach.cb_interface(input_keys=["person_detection"], output_keys=["pointstamped"])
    def _compute_person_point(self, userdata):
        """Computes the point to look at for the person.

        Args:
            userdata (dict): Userdata containing the guest data.

        Returns:
            PointStamped: The point to look at.
        """
        pass
