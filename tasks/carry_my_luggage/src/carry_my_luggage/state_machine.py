import smach

from lasr_skills import (
    WaitForPerson,
    DetectPointingDirection,
    Say,
    PlayMotion,
    ReceiveObject,
)

import rospy


class CarryMyLuggage(smach.StateMachine):

    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:
            smach.StateMachine.add(
                "WAIT_FOR_PERSON",
                WaitForPerson(),
                transitions={
                    "succeeded": "DETECT_POINTING_DIRECTION",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_POINTING_DIRECTION",
                DetectPointingDirection(),
                transitions={
                    "succeeded": "SAY_BAG",
                    "failed": "SAY_FAILED_POINTING",
                },
            )

            smach.StateMachine.add(
                "SAY_FAILED_POINTING",
                Say(
                    text="I could not detect the direction that you are pointing. I'll try again."
                ),
                transitions={
                    "succeeded": "DETECT_POINTING_DIRECTION",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_BAG",
                Say(format_str="I need you to give me the bag on your {}."),
                transitions={
                    "succeeded": "RECEIVE_BAG",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "pointing_direction"},
            )

            smach.StateMachine.add(
                "RECEIVE_BAG",
                ReceiveObject(object_name="bag", vertical=False),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
