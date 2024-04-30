import smach

from lasr_skills import WaitForPerson, DetectPointingDirection, Say

from lasr_vision_msgs.msg import Direction


class CarryMyLuggage(smach.StateMachine):

    class HandlePointingDirection(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded"],
                input_keys=["pointing_direction"],
                output_keys=["pointing_direction_str"],
            )

        def execute(self, userdata):
            if userdata.pointing_direction == Direction.LEFT:
                userdata.pointing_direction_str = "to the left"
            elif userdata.pointing_direction == Direction.RIGHT:
                userdata.pointing_direction_str = "to the right"
            else:
                userdata.pointing_direction_str = "forwards"
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
                    "succeeded": "SAY_DIRECTION",
                    "failed": "SAY_FAILED_POINTING",
                },
            )

            smach.StateMachine.add(
                "SAY_DIRECTION",
                Say(format_str="I see you are pointing {}"),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
                remapping={"placeholders": "pointing_direction_str"},
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
