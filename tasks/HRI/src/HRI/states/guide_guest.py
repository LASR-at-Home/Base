import smach
from geometry_msgs.msg import Pose
from lasr_skills import (
    GoToLocation,
    Say,
)


class GuideGuestToLivingRoom(smach.StateMachine):
    def __init__(self, node, guest_id, living_room_pose: Pose):
        super().__init__(outcomes=["succeeded", "failed"])

        self.node = node
        self.guest_id = guest_id
        self.living_room_pose = living_room_pose

        with self:
            smach.StateMachine.add(
                f"SAY_FOLLOW_GUEST_{self.guest_id}",
                Say(
                    node=self.node,
                    text="Please follow me, I will guide you to the other guests",
                ),
                transitions={
                    "succeeded": f"GO_TO_LIVING_ROOM_LOCATION_GUEST_{self.guest_id}",
                    "preempted": "failed",
                    "aborted": "failed",
                },
            )

            smach.StateMachine.add(
                f"GO_TO_LIVING_ROOM_LOCATION_GUEST_{self.guest_id}",
                GoToLocation(node=self.node, location=self.living_room_pose),
                transitions={
                    "succeeded": "succeeded",
                    "failed": "failed",
                },
            )
