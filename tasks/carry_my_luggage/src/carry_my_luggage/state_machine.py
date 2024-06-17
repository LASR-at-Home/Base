import smach
import smach_ros
from lasr_skills import (
    WaitForPerson,
    Say,
    DetectGesture,
    ReceiveObject,
    HandoverObject,
)
from lasr_skills.vision import GetCroppedImage
from lasr_person_following.msg import FollowAction


class CarryMyLuggage(smach.StateMachine):
    class ProcessPointingDirection(smach.State):
        def __init__(self):
            smach.State.__init__(
                self,
                outcomes=["succeeded", "failed"],
                input_keys=["gesture_detected"],
                output_keys=["pointing_direction"],
            )

        def execute(self, userdata):
            if userdata.gesture_detected == "pointing_to_the_left":
                userdata.pointing_direction = "left"
            elif userdata.gesture_detected == "pointing_to_the_right":
                userdata.pointing_direction = "right"
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
                    "succeeded": "SAY_POINT_TO_BAG",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_POINT_TO_BAG",
                Say(text="Please point to the bag that you wish me to carry."),
                transitions={
                    "succeeded": "GET_IMAGE",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "GET_IMAGE",
                GetCroppedImage(object_name="person", crop_method="closest"),
                transitions={
                    "succeeded": "DETECT_POINTING_DIRECTION",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "DETECT_POINTING_DIRECTION",
                DetectGesture(),
                transitions={
                    "succeeded": "PROCESS_POINTING_DIRECTION",
                    "failed": "SAY_FAILED_POINTING",
                },
            )

            smach.StateMachine.add(
                "PROCESS_POINTING_DIRECTION",
                CarryMyLuggage.ProcessPointingDirection(),
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
                    "succeeded": "GET_IMAGE",
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
                ReceiveObject(object_name="bag", vertical=True),
                transitions={
                    "succeeded": "SAY_FOLLOW",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_FOLLOW",
                Say(text="I will follow you now."),
                transitions={
                    "succeeded": "FOLLOW",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "FOLLOW",
                smach_ros.SimpleActionState(
                    "follow_person",
                    FollowAction,
                ),
                transitions={
                    "succeeded": "SAY_HANDOVER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_HANDOVER",
                Say(text="I will hand over the bag to you now."),
                transitions={
                    "succeeded": "HANDOVER",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )

            smach.StateMachine.add(
                "HANDOVER",
                HandoverObject(object_name="bag", vertical=True),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
