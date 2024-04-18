import smach

from geometry_msgs.msg import Pose
from shapely.geometry import Polygon
from lasr_skills import GoToLocation, WaitForPersonInArea, Say, AskAndListen
from receptionist.states import (
    ParseNameAndDrink,
    GetGuestAttributes,
    Introduce,
    SeatGuest,
)


class Receptionist(smach.StateMachine):

    def __init__(
        self,
        wait_pose: Pose,
        wait_area: Polygon,
        seat_pose: Pose,
        seat_area: Polygon,
        host_data: dict,
    ):

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:

            self.userdata.guest_data = {"host": host_data, "guest1": {}, "guest2": {}}

            smach.StateMachine.add(
                "GO_TO_WAIT_LOCATION_GUEST_1",
                GoToLocation(wait_pose),
                transitions={
                    "succeeded": "SAY_WAITING_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING_GUEST_1",
                Say(text="I am waiting for a guest."),
                transitions={
                    "succeeded": "WAIT_FOR_PERSON_GUEST_1",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_PERSON_GUEST_1",
                WaitForPersonInArea(wait_area),
                transitions={
                    "succeeded": "GET_NAME_AND_DRINK_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GET_NAME_AND_DRINK_GUEST_1",
                AskAndListen("What is your name and favourite drink?"),
                transitions={
                    "succeeded": "PARSE_NAME_AND_DRINK_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "PARSE_NAME_AND_DRINK_GUEST_1",
                ParseNameAndDrink("guest1"),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES_GUEST_1",
                    "failed": "failed",
                },
                remapping={"guest_transcription": "transcribed_speech"},
            )

            smach.StateMachine.add(
                "GET_GUEST_ATTRIBUTES_GUEST_1",
                GetGuestAttributes("guest1"),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_FOLLOW_GUEST_1",
                Say(text="Please follow me, I will guide you to the other guests"),
                transitions={
                    "succeeded": "GO_TO_SEAT_LOCATION_GUEST_1",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SEAT_LOCATION_GUEST_1",
                GoToLocation(seat_pose),
                transitions={
                    "succeeded": "SAY_WAIT_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAIT_GUEST_1",
                Say(text="Please wait here on my left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_HOST",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_HOST",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="host"),
                transitions={
                    "succeeded": "INTRODUCE_HOST_TO_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_HOST_TO_GUEST_1",
                Introduce(guest_to_introduce="host", guest_to_introduce_to="guest1"),
                transitions={
                    "succeeded": "SEAT_GUEST_1",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_1",
                SeatGuest(seat_area),
                transitions={
                    "succeeded": "GO_TO_WAIT_LOCATION_GUEST_2",
                    "failed": "failed",
                },
            )

            """
            Guest 2
            """

            smach.StateMachine.add(
                "GO_TO_WAIT_LOCATION_GUEST_2",
                GoToLocation(wait_pose),
                transitions={
                    "succeeded": "SAY_WAITING_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING_GUEST_2",
                Say(text="I am waiting for a guest."),
                transitions={
                    "succeeded": "WAIT_FOR_PERSON_GUEST_2",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_PERSON_GUEST_2",
                WaitForPersonInArea(wait_area),
                transitions={
                    "succeeded": "GET_NAME_AND_DRINK_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GET_NAME_AND_DRINK_GUEST_2",
                AskAndListen("What is your name and favourite drink?"),
                transitions={
                    "succeeded": "PARSE_NAME_AND_DRINK_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "PARSE_NAME_AND_DRINK_GUEST_2",
                ParseNameAndDrink("guest2"),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES_GUEST_2",
                    "failed": "failed",
                },
                remapping={"guest_transcription": "transcribed_speech"},
            )

            smach.StateMachine.add(
                "GET_GUEST_ATTRIBUTES_GUEST_2",
                GetGuestAttributes("guest2"),
                transitions={
                    "succeeded": "SAY_FOLLOW_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_FOLLOW_GUEST_2",
                Say(text="Please follow me, I will guide you to the other guests"),
                transitions={
                    "succeeded": "GO_TO_SEAT_LOCATION_GUEST_2",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SEAT_LOCATION_GUEST_2",
                GoToLocation(seat_pose),
                transitions={
                    "succeeded": "SAY_WAIT_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAIT_GUEST_2",
                Say(text="Please wait here on my left"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_2_TO_EVERYONE",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_2_TO_EVERYONE",
                Introduce(guest_to_introduce="guest2", everyone=True),
                transitions={
                    "succeeded": "INTRODUCE_HOST_TO_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_HOST_TO_GUEST_2",
                Introduce(guest_to_introduce="host", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "INTRODUCE_GUEST_1_TO_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "INTRODUCE_GUEST_1_TO_GUEST_2",
                Introduce(guest_to_introduce="guest1", guest_to_introduce_to="guest2"),
                transitions={
                    "succeeded": "SEAT_GUEST_2",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SEAT_GUEST_2",
                SeatGuest(seat_area),
                transitions={"succeeded": "GO_TO_FINISH_LOCATION", "failed": "failed"},
            )

            """
            Finish
            """
            smach.StateMachine.add(
                "GO_TO_FINISH_LOCATION",
                GoToLocation(wait_pose),
                transitions={
                    "succeeded": "SAY_FINISHED",
                    "failed": "failed",
                },
            )
            smach.StateMachine.add(
                "SAY_FINISHED",
                Say(text="I am done."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
