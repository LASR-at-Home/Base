import smach

from geometry_msgs.msg import Pose

from lasr_skills import GoToLocation, Say


class Guide(smach.StateMachine):

    def __init__(self, location_name: str, location_pose: Pose):

        smach.StateMachine.__init__(self, outcomes=["succeeded", "failed"])

        with self:

            smach.StateMachine.add(
                "SAY_FOLLOW_ME",
                Say(text=f"Please follow me, I will guide you to the {location_name}"),
                transitions={
                    "succeeded": "GO_TO_LOCATION",
                    "aborted": "GO_TO_LOCATION",
                    "preempted": "GO_TO_LOCATION",
                },
            )

            smach.StateMachine.add(
                "GO_TO_LOCATION",
                GoToLocation(location_pose),
                transitions={
                    "succeeded": "SAY_DONE",
                    "failed": "failed",
                },
            )

            smach.StateMachine.add(
                "SAY_DONE",
                Say(text=f"We have arrived at the {location_name}."),
                transitions={
                    "succeeded": "succeeded",
                    "aborted": "failed",
                    "preempted": "failed",
                },
            )
