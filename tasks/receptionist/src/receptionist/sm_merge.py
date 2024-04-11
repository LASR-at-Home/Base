#!/usr/bin/env python3
import smach
import rospy

from geometry_msgs.msg import Pose, Point, Quaternion
from shapely.geometry import Polygon

from receptionist.states import ParseNameAndDrink, GetGuestAttributes, Introduce

from lasr_skills import GoToLocation, WaitForPersonInArea, Say, AskAndListen


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

            self.userdata.guest_id = "guest1"
            self.userdata.guest_data = {"host": host_data, "guest1": {}, "guest2": {}}

            smach.StateMachine.add(
                "GO_TO_WAIT_LOCATION",
                GoToLocation(wait_pose),
                transitions={
                    "succeeded": "SAY_WAITING",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAITING",
                Say(text="I am waiting for a guest."),
                transitions={
                    "succeeded": "WAIT_FOR_PERSON",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "WAIT_FOR_PERSON",
                WaitForPersonInArea(wait_area),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "GET_NAME_AND_DRINK",
                AskAndListen("What is your name and favourite drink?"),
                transitions={
                    "succeeded": "PARSE_NAME_AND_DRINK",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "PARSE_NAME_AND_DRINK",
                ParseNameAndDrink(),
                transitions={
                    "succeeded": "GET_GUEST_ATTRIBUTES",
                    "failed": "failed",
                },
                remapping={"guest_transcription": "transcribed_speech"},
            )

            smach.StateMachine.add(
                "GET_GUEST_ATTRIBUTES",
                GetGuestAttributes(),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_FOLLOW",
                Say(text="Please follow me, I will guide you to the other guests"),
                transitions={
                    "succeeded": "succeeded",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                "GO_TO_SEAT_LOCATION",
                GoToLocation(seat_pose),
                transitions={
                    "succeeded": "SAY_WAIT",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_WAIT",
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
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )


if __name__ == "__main__":
    rospy.init_node("receptionist")

    wait_pose_param = rospy.get_param("/receptionist/wait_pose")
    wait_pose = Pose(
        position=Point(**wait_pose_param["position"]),
        orientation=Quaternion(**wait_pose_param["orientation"]),
    )

    wait_area_param = rospy.get_param("/receptionist/wait_area")
    wait_area = Polygon(wait_area_param)

    seat_pose_param = rospy.get_param("/receptionist/seat_pose")
    seat_pose = Pose(
        position=Point(**seat_pose_param["position"]),
        orientation=Quaternion(**seat_pose_param["orientation"]),
    )

    seat_area_param = rospy.get_param("/receptionist/seat_area")
    seat_area = Polygon(seat_area_param)

    sm = Receptionist(
        wait_pose, wait_area, seat_pose, seat_area, {"name": "John", "drink": "beer"}
    )
    outcome = sm.execute()
