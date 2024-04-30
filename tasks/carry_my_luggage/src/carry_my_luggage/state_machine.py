import smach

from lasr_skills import (
    WaitForPerson,
    DetectPointingDirection,
    Say,
    PlayMotion,
    ReceiveObject,
)

from lasr_vision_msgs.msg import Direction


class CarryMyLuggage(smach.StateMachine):

    class HandlePointingDirection(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["pointing_direction"],
                output_keys=["pointing_direction_str"],
            )

        def execute(self, userdata):
            if userdata.pointing_direction == Direction.LEFT:
                userdata.pointing_direction_str = "left"
            elif userdata.pointing_direction == Direction.RIGHT:
                userdata.pointing_direction_str = "right"
            else:
                return "failed"
            return "succeeded"

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
                    "succeeded": "HANDLE_POINTING_DIRECTION",
                    "failed": "SAY_FAILED_POINTING",
                },
            )

            smach.StateMachine.add(
                "HANDLE_POINTING_DIRECTION",
                self.HandlePointingDirection(),
                transitions={
                    "succeeded": "SAY_BAG",
                    "failed": "SAY_FAILED_POINTING",
                },
                remapping={
                    "pointing_direction_str": "pointing_direction_str",
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
                remapping={"placeholders": "pointing_direction_str"},
            )

            smach.StateMachine.add(
                "RECEIVE_BAG",
                ReceiveObject(object_name="bag", vertical=False),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
